#include <vector>
#include <algorithm>
#include <utility>
#include <tuple>
#include <queue>

#include <glm/gtc/type_ptr.hpp>
#include <fmt/format.h>

#include "tsl/geometry/line_segment.hpp"
#include "tsl/evaluation/surface_evaluator.hpp"
#include "tsl/evaluation/subdevision.hpp"
#include "tsl/evaluation/bsplines.hpp"
#include "tsl/util/panic.hpp"
#include "tsl/util/println.hpp"
#include "tsl/algorithm/reduction.hpp"

using std::vector;
using std::min;
using std::max;
using std::move;
using std::tie;
using std::queue;

using glm::value_ptr;
using fmt::format;

namespace tsl {

surface_evaluator::surface_evaluator(tmesh&& mesh):
    config(), mesh(move(mesh)), uv(), dir(), edge_trans(), support(), knots(), handles(), knot_vectors() {
    update_cache();
}

vector<regular_grid> surface_evaluator::eval_per_face(uint32_t res) const {
    vector<regular_grid> out;
    out.reserve(mesh.num_faces());

    // This buffer will be used in the loop to store vertex handles. To reduce allocations we reuse the buffer
    // and start with a estimated size of 10.
    vector<vertex_handle> vertices_buffer;
    vertices_buffer.reserve(10);
    for (const auto& fh: mesh.get_faces()) {
        auto contains_extraordinary_vertex = false;
        auto contains_invalid_valence = false;
        vertices_buffer.clear();
        mesh.get_vertices_of_face(fh, vertices_buffer);
        for (const auto& vh: vertices_buffer) {
            if (mesh.is_extraordinary(vh)) {
                contains_extraordinary_vertex = true;
            }
            // TODO: this will be fixed, when evaluation near borders is implemented; or not: if not, we should throw
            //       a warning!
            if (mesh.get_valence(vh) < 3) {
                contains_invalid_valence = true;
                report_error(format("invalid valence at vertex with handle id: {}", vh.get_idx()));
            }
        }

        if (contains_extraordinary_vertex) {
            if (!contains_invalid_valence) {
                auto grid = eval_subdevision(res, fh);
                out.emplace_back(grid);
            }
        } else {
            auto grid = eval_bsplines(res, fh);
            out.emplace_back(grid);
        }
    }

    return out;

}

regular_grid surface_evaluator::eval_bsplines(uint32_t res, face_handle handle) const {
    auto local_system_max = get_max_coords(handle);
    double u_coord = local_system_max.x;
    double v_coord = local_system_max.y;
    auto u_max = res + 1u;
    auto v_max = res + 1u;
    double step_u = u_coord / res;
    double step_v = v_coord / res;

    regular_grid grid(handle);
    grid.points.reserve(static_cast<size_t>(v_max));
    grid.normals.reserve(static_cast<size_t>(v_max));
    grid.num_points_x = u_max;
    grid.num_points_y = v_max;

    double current_u = 0;
    double current_v = 0;
    for (uint32_t v = 0; v < v_max; ++v) {
        current_u = 0;
        vector<vec3> row;
        vector<vec3> normal_row;
        row.reserve(static_cast<size_t>(u_max));
        normal_row.reserve(static_cast<size_t>(u_max));
        for (uint32_t u = 0; u < u_max; ++u) {
            auto[point, du, dv] = eval_bsplines_point(min(current_u, u_coord), min(current_v, v_coord), handle);
            row.push_back(point);
            normal_row.push_back(normalize(cross(du, dv)));
            current_u += step_u;
        }
        grid.points.push_back(row);
        grid.normals.push_back(normal_row);
        current_v += step_v;
    }
    return grid;
}

array<vec3, 3> surface_evaluator::eval_bsplines_point(double u, double v, face_handle f) const {
    vec3 c(0, 0, 0);
    vec3 cdu(0, 0, 0);
    vec3 cdv(0, 0, 0);
    double d = 0;
    double du = 0;
    double dv = 0;
    // vector of u and v location on the face
    vec2 in(u, v);

    // degree of splines
    const int DEGREE = 3;

    // TODO determine what this is
    const auto& supports = support[f];
    for (const auto& [vertex, idx, trans]: supports) {
        const auto& local_knots = knot_vectors[vertex][idx];

	// value of the control point at this vertex position
        const auto& p = mesh.get_vertex_position(vertex);
	// transform the input u and v coordinates for consistency 
	// (say across cuts)
        auto transformed = trans.apply(in);
        
	// extract the value of the basis function at the given location
        auto u_basis = get_bspline_with_der<DEGREE>(transformed.x, local_knots.u);
        auto v_basis = get_bspline_with_der<DEGREE>(transformed.y, local_knots.v);

	// contribution of this basis (and its associated control point) to the surface evaluation
        c += u_basis.x * v_basis.x * p;
        // normalization term (should be 1 for analysis-suitable T-splines)
	d += u_basis.x * v_basis.x;

	// based on whether a rotation occured, adjust the basis information
	// TODO -- determine what these values are
        switch (trans.r) {
            case 0:
            case 4:
                cdu += u_basis.y * v_basis.x * p * trans.f;
                cdv += u_basis.x * v_basis.y * p * trans.f;
                du += u_basis.y * v_basis.x * trans.f;
                dv += u_basis.x * v_basis.y * trans.f;
                break;
            case 1:
                cdu += u_basis.x * v_basis.y * p * trans.f;
                cdv += (-u_basis.y) * v_basis.x * p * trans.f;
                du += u_basis.x * v_basis.y * trans.f;
                dv += (-u_basis.y) * v_basis.x * trans.f;
                break;
            case 2:
                cdu += (-u_basis.y) * v_basis.x * p * trans.f;
                cdv += u_basis.x * (-v_basis.y) * p * trans.f;
                du += (-u_basis.y) * v_basis.x * trans.f;
                dv += u_basis.x * (-v_basis.y) * trans.f;
                break;
            case 3:
                cdu += u_basis.x * (-v_basis.y) * p * trans.f;
                cdv += u_basis.y * v_basis.x * p * trans.f;
                du += u_basis.x * (-v_basis.y) * trans.f;
                dv += u_basis.y * v_basis.x * trans.f;
                break;
            default:
                panic("unknown rotation in transform!");
        }
    }

    // return point of evaluation (normalized because we assume d != 1)
    // TODO -- product rule of some sort? why does it not include case = 0?
    return {
        c / d,
        ((cdu * d) - (c * du)) / (d * d),
        ((cdv * d) - (c * dv)) / (d * d)
    };
}

regular_grid surface_evaluator::eval_subdevision(uint32_t res, face_handle handle) const {
    auto local_system_max = get_max_coords(handle);
    double u_coord = local_system_max.x;
    double v_coord = local_system_max.y;
    auto u_max = res + 1u;
    auto v_max = res + 1u;
    double step_u = u_coord / res;
    double step_v = v_coord / res;

    regular_grid grid(handle);
    grid.points.reserve(static_cast<size_t>(v_max));
    grid.normals.reserve(static_cast<size_t>(v_max));
    grid.num_points_x = u_max;
    grid.num_points_y = v_max;

    // TODO: This can be cached!
    auto neighbours = get_vertices_for_subd(handle);
    vector<double> x_coords;
    vector<double> y_coords;
    vector<double> z_coords;
    x_coords.reserve(neighbours.size());
    y_coords.reserve(neighbours.size());
    z_coords.reserve(neighbours.size());
    for (const auto& vh: neighbours) {
        auto pos = mesh.get_vertex_position(vh);
        x_coords.push_back(pos.x);
        y_coords.push_back(pos.y);
        z_coords.push_back(pos.z);
    }

    auto extraordinary_vertex = neighbours.front();
    auto valence = mesh.get_valence(extraordinary_vertex);

    double current_u = 0;
    double current_v = 0;
    for (uint32_t v = 0; v < v_max; ++v) {
        current_u = 0;
        vector<vec3> pos_row;
        vector<vec3> normal_row;
        pos_row.reserve(static_cast<size_t>(u_max));
        normal_row.reserve(static_cast<size_t>(u_max));
        for (uint32_t u = 0; u < u_max; ++u) {
            auto ud = min(current_u / u_coord, u_coord);
            auto vd = min(current_v / v_coord, v_coord);
            vec3 point;
            vec3 du;
            vec3 dv;
            subd_eval(
                ud,
                vd,
                2 * valence + 8,
                x_coords.data(),
                y_coords.data(),
                z_coords.data(),
                value_ptr(point),
                value_ptr(du),
                value_ptr(dv),
                nullptr,
                nullptr,
                nullptr
            );
            pos_row.push_back(point);
            normal_row.push_back(normalize(cross(du, dv)));
            current_u += step_u;
        }
        grid.points.push_back(pos_row);
        grid.normals.push_back(normal_row);
        current_v += step_v;
    }
    return grid;
}

const tmesh& surface_evaluator::get_tmesh() const {
    return mesh;
}

vec2 surface_evaluator::get_max_coords(face_handle handle) const {
    vec2 out(0, 0);
    for (const auto& eh: mesh.get_half_edges_of_face(handle)) {
        // Check u direction
        if (dir[eh] % 2 == 0) {
            out.x = max(out.x, uv[eh].x);
        } else {
            // Check v direction
            out.y = max(out.y, uv[eh].y);
        }
    }
    return out;
}

// ========================================================================
// = T-Mesh modifier
// ========================================================================

bool surface_evaluator::remove_edge(edge_handle handle, bool keep_vertices) {
    auto res = mesh.remove_edge(handle, keep_vertices);
    if (res) {
        update_cache();
    }
    return res;
}

size_t surface_evaluator::remove_edges(double percent) {
    auto deleted = ::tsl::remove_edges(mesh, percent);

    if (deleted > 0) {
        update_cache();
    }

    return deleted;
}

vec3& surface_evaluator::get_vertex_pos(vertex_handle handle) {
    return mesh.get_vertex_position(handle);
}

// ========================================================================
// = Helper functions
// ========================================================================

vector<vertex_handle> surface_evaluator::get_vertices_for_subd(face_handle handle) const {
    // Find extraordinary vertex and edge pointing to it.
    optional<pair<vertex_handle, half_edge_handle>> found;
    for (const auto& eh: mesh.get_half_edges_of_face(handle)) {
        auto vh = mesh.get_target(eh);
        if (mesh.is_extraordinary(vh)) {
            found = make_pair(vh, eh);
        }
    }

    if (!found) {
        panic("Subdevision surface evaluation needs a face with one extraordinary vertex!");
    }

    auto extraordinary_vertex = (*found).first;
    auto start_edge = (*found).second;

    // Get vertices in order given by Stam's paper (Fig. 3)
    vector<vertex_handle> out;
    auto valence = mesh.get_valence(extraordinary_vertex);
    out.reserve(2 * valence + 8);

    auto h = start_edge;
    out.push_back(extraordinary_vertex);
    h = mesh.get_next(mesh.get_next(mesh.get_twin(h)));
    auto h2 = h;
    do {
        out.push_back(mesh.get_target(h));
        h = mesh.get_prev(h);
        out.push_back(mesh.get_target(h));
        h = mesh.get_prev(mesh.get_twin(mesh.get_prev(h)));
    } while (h != h2);

    // Points from 7 to 2N+5
    h = mesh.get_next(mesh.get_twin(mesh.get_next(mesh.get_next(mesh.get_next(mesh.get_twin(mesh.get_next(start_edge)))))));
    auto twon5 = mesh.get_target(h);

    h = mesh.get_next(h);
    auto twon4 = mesh.get_target(h);

    h = mesh.get_next(mesh.get_twin(mesh.get_next(h)));
    auto twon3 = mesh.get_target(h);

    // Add 2N+2 and saved vertices
    h = mesh.get_next(mesh.get_twin(mesh.get_next(h)));
    out.push_back(mesh.get_target(h));
    out.push_back(twon3);
    out.push_back(twon4);
    out.push_back(twon5);

    h = mesh.get_next(h);
    out.push_back(mesh.get_target(h));

    h = mesh.get_next(mesh.get_twin(mesh.get_next(h)));
    out.push_back(mesh.get_target(h));

    h = mesh.get_next(mesh.get_twin(mesh.get_next(h)));
    out.push_back(mesh.get_target(h));

    return out;
}

aa_rectangle surface_evaluator::get_parametric_domain(vertex_handle handle, size_t handle_index) const {
    // length of knot intervals in the direction of this handle
    auto sum_knot_vectors1 = knots[handle][handle_index][0] + knots[handle][handle_index][1];

    // Get face
    auto [h, q] = handles[handle][handle_index];
    // whenever this is called, we should be on a parametric face
    auto face_h = mesh.get_face_of_half_edge(h).expect(EXPECT_NO_BORDER);

    // Get neighbouring face
    // -1 means the face -1 cw (+1 ccw)
    auto wrapped_index = (handles[handle].size() + handle_index - 1) % handles[handle].size();
    auto [nh, nq] = handles[handle][wrapped_index];

    double scale_factor = 1;
    // this half-edge is on a border; no scaling factor necessary
    if (!mesh.get_face_of_half_edge(nh)) {
        // this was intentially left empty
    }
    // half-edge is not on a border
    else {
	// determine the face of the next handle
	auto nface_h = mesh.get_face_of_half_edge(nh).expect(EXPECT_NO_BORDER);
	
	// Only calc the scale factor, if we are NOT at a t-joint
	double scale_factor = 1;
	if (face_h != nface_h) {
	    auto separating_edge = mesh.get_half_edge_between(face_h, nface_h).expect(EXPECT_NO_BORDER);
	    scale_factor = expect(mesh.get_knot_factor(separating_edge), EXPECT_NO_BORDER);
	}
    }
    
    // length of the knot intervals in the CCW transverse direction
    auto sum_knot_vectors2 = knots[handle][wrapped_index][0] + knots[handle][wrapped_index][1];
    // rectangle representing the support of this "quadrant" of the basis function
    aa_rectangle r(vec2(0, 0), vec2(sum_knot_vectors1, sum_knot_vectors2 * scale_factor));

    return r;
}

local_knot_vectors surface_evaluator::get_knot_vectors(vertex_handle handle, size_t handle_index) const {

    // TODO: this assumes that there are no borders;
    //       this needs to be adjusted in the case of borders
    // TODO: this assumes that points are not extraordinary... 
    //       address the case of extraordinary point

    // determine the extended valence of the vertex
    int extended_valence = mesh.get_extended_valence(handle);
    // determine if the vertex is on a border
    bool border_vert = mesh.is_border_vertex(handle);

//    if (extended_valence == 4 && !border_vert) {
        return get_regular_internal_knot_vectors(handle, handle_index);
//    }
//    else {
//	panic("Cannot currently operate on the given type of vertex");
//    }
}

local_knot_vectors surface_evaluator::get_regular_internal_knot_vectors(vertex_handle handle, size_t handle_index) const {
    // Note on variable names: a number i at the end means "domain + i" and a _i number means "domain - i"
    
    // extract the halfedge and tag from the handle
    auto [edge_h0, tag0] = handles[handle][handle_index];

    // current call routines demand that this will have a non-empty face
    auto face_h0 = mesh.get_face_of_half_edge(edge_h0).expect(EXPECT_NO_BORDER);

    // Get half edge and face handle for index + 1 cw
    auto index1 = (handles[handle].size() + handle_index + 1) % handles[handle].size();
    auto [edge_h1, tag1] = handles[handle][index1];
    auto face_h1 = mesh.get_face_of_half_edge(edge_h1).expect(EXPECT_NO_BORDER);

    // Get half edge and face handle for index - 1 cw (which is + 1 ccw)
    auto index_1 = (handles[handle].size() + handle_index - 1) % handles[handle].size();
    auto [edge_h_1, tag_1] = handles[handle][index_1];
    auto face_h_1 = mesh.get_face_of_half_edge(edge_h_1).expect(EXPECT_NO_BORDER);

    // Get half edge and face handle for index - 2 cw (which is + 2 ccw)
    auto index_2 = (handles[handle].size() + handle_index - 2) % handles[handle].size();
    auto [edge_h_2, tag_2] = handles[handle][index_2];
    auto face_h_2 = mesh.get_face_of_half_edge(edge_h_2).expect(EXPECT_NO_BORDER);

    // Get two knot values for current handle
    auto knot01 = knots[handle][handle_index][0];
    auto knot02 = knots[handle][handle_index][1];

    // Get transformation from values from hindex + 1 into domain of handle
    double transform_01 = 1;
    if (face_h0 != face_h1) {
        auto edge_bewteen_0_and1 = mesh.get_half_edge_between(face_h0, face_h1).expect(EXPECT_NO_BORDER);
        transform_01 = expect(mesh.get_knot_factor(edge_bewteen_0_and1), EXPECT_NO_BORDER);
    }

    // Get two knot values for handle with index + 1 and transform them into the domain of the current handle
    auto knot11 = knots[handle][index1][0] * transform_01;
    auto knot12 = knots[handle][index1][1] * transform_01;

    // Get transformation from values from hindex - 1 into domain of handle
    double transform_0_1 = 1;
    if (face_h0 != face_h_1) {
        auto edge_bewteen_0_and_1 = mesh.get_half_edge_between(face_h0, face_h_1).expect(EXPECT_NO_BORDER);
        transform_0_1 = expect(mesh.get_knot_factor(edge_bewteen_0_and_1), EXPECT_NO_BORDER);
    }

    // Get two knot values for handle with index - 1 and transform them into the domain of the current handle
    auto knot_11 = knots[handle][index_1][0] * transform_0_1;
    auto knot_12 = knots[handle][index_1][1] * transform_0_1;

    // For the transition from index - 2 into domain of handle we need two transitions: -2 to -1 and -1 to handle
    // Get transformation from values from hindex - 2 into index - 1
    double transform__1_2 = 1;
    if (face_h_1 != face_h_2) {
        auto edge_bewteen__1_and_2 = mesh.get_half_edge_between(face_h_1, face_h_2).expect(EXPECT_NO_BORDER);
        transform__1_2 = expect(mesh.get_knot_factor(edge_bewteen__1_and_2), EXPECT_NO_BORDER);
    }
    auto transform_0_2 = transform_0_1 * transform__1_2;

    // Get two knot values for handle with index - 2 and transform them into the domain of the current handle
    auto knot_21 = knots[handle][index_2][0] * transform_0_2;
    auto knot_22 = knots[handle][index_2][1] * transform_0_2;

    local_knot_vectors out(
        {-knot_21 - knot_22, -knot_21, 0, knot01, knot01 + knot02},
        {-knot11 - knot12, -knot11, 0, knot_11, knot_11 + knot_12}
    );
    return out;
}

//local_knot_vectors surface_evaluator::get_regular_border_knot_vectors(vertex_handle handle, size_t handle_index) const {

//}

void surface_evaluator::update_cache() {
    calc_local_coords();
    calc_edge_trans();
    auto transforms = setup_basis_funs();
    calc_knots();
    calc_support(transforms);
}

void surface_evaluator::report_error(const string& msg) const {
    if (config.panic_at_integrity_violations) {
        panic(msg);
    } else {
        println(msg);
    }
}

// ========================================================================
// = Routines from paper
// ========================================================================

// assign uv coordinates to each face
void surface_evaluator::calc_local_coords() {
    // clear the storage map taking a half-edge handle to a uv coordinate
    uv.clear();
    uv.reserve(mesh.num_half_edges());
    // clear the storage map taking a half-edge handle to a direction 
    // (+u = 0, +v = 1, -u = 2, -v = 3)
    dir.clear();
    dir.reserve(mesh.num_half_edges());

    // iterate on all edges of the face
    for (const auto& fh: mesh.get_faces()) {
	// assume initial knot coordinate of (0,0)
        vec2 c(0, 0);
	// start in the positive u direction
        uint8_t i = 0;

	// iterate over all half-edges of the face
        for (const auto& eh: mesh.get_half_edges_of_face(fh)) {
            // currently, we expect no half-edges with knot interval of 0
	    // TODO -- adjust this!
	    //      -- when adjusted, we may need an index map
	    // This actually may just mean that we need knot intervals defined on the mesh...
            auto k = expect(mesh.get_knot_interval(eh), EXPECT_NO_BORDER);
	    // rotate the vector by pi/2 * i and add to current position 
            c += rotate(i, vec2(k, 0));

	    // store this position
            uv.insert(eh, c);
	    // store the direction
            dir.insert(eh, i);

	    // if the edge handle terminates at a corner, iterate by 90 degrees
	    // TODO -- I do not think that we need an EXPECT_NO_BORDER exception here
	    //      -- probably more like an expected corners exception
            if (expect(mesh.corner(eh), EXPECT_NO_BORDER)) {
                i += 1;
            }
        }
    }
}

void surface_evaluator::calc_edge_trans() {
    // compute the transformation from one edge to the other
    edge_trans.clear();
    edge_trans.reserve(mesh.num_half_edges());

    // iterate over each halfedge
    for (const auto& eh: mesh.get_half_edges()) {
	// find the opposite halfedge
        auto twin = mesh.get_twin(eh);
	// if twin or its opposite are on a border, make the transformation null
	if (!mesh.get_face_of_half_edge(eh) || !mesh.get_face_of_half_edge(twin)) { 
	    vec2 c(0,0);
            edge_trans.insert(eh, transform(1, 0, c));
        }
	// neither the twin nor its opposite are on a border
	else {
	    // assure that knots are defined on both knot and its twin
	    // if so, find scaling from this half-edge to the opposite half-edge
	    auto f = expect(mesh.get_knot_factor(twin), EXPECT_NO_BORDER);
	    // if coordinate systems are the same, the twin will travel in the opposite direction of
	    // the current, so there will be no additional rotation; otherwise, account for rotations
	    auto r = static_cast<uint8_t>((dir[twin] - dir[eh] + 6) % 4);
	    // translation from one coordinate system to the next
	    // TODO -- check that this is correct
	    auto t = uv[mesh.get_prev(twin)] - (f * rotate(r, uv[eh]));
	    // return transformation map across the face
	    edge_trans.insert(eh, transform(f, r, t));
	}
    }
}

basis_fun_trans_map surface_evaluator::setup_basis_funs() {
    basis_fun_trans_map transforms{vector<transform>()};

    // reserve a basis function for each vertex in the mesh
    handles.clear();
    handles.reserve(mesh.num_vertices());
    // transformations on basis functions
    // TODO -- understand purpose here
    transforms.reserve(mesh.num_vertices());

    // iterate over all vertices
    for (const auto& vh: mesh.get_vertices()) {
        // create a vector describing all half-edges going through this
	handles.insert(vh, vector<tuple<half_edge_handle, tag>>());
        handles[vh].reserve(mesh.get_valence(vh));
	// iterate on outgoing halfedges
        for (const auto& eh: mesh.get_half_edges_of_vertex(vh, edge_direction::outgoing)) {
            // if this is on a border, it as on the border
	    if (!mesh.get_face_of_half_edge(eh)) {
	        handles[vh].emplace_back(eh, tag::border);
		vec2 c(0, 0);
		transforms[vh].emplace_back(1, 0, c);
		continue;
	    }
	    // find the transformation that takes the outgoing halfedge from this vertex
	    // on this face to the face's half-edge emanating from (0,0) in the positive u direction
	    handles[vh].emplace_back(eh, tag::positive_u);
            auto r = static_cast<uint8_t>(4 - dir[eh]);
            auto t = -rotate(r, uv[mesh.get_prev(eh)]);
	    // scaling is one because on each face, coordinate systems are consistent
            transforms[vh].emplace_back(1, r, t);

	    // look at the opposite halfedge
            auto twin = mesh.get_twin(eh);
	    // TODO -- determine why this is necessary
	    // if it does not point into a corner, mark the next as negative v
	    // THIS IMPLICITLY ASSUMES AT MOST ONE T-JUNCTION PER FACE; THIS MAY NEED MODIFICATION 
            // Correction: This assumes that T-junctions extensions are described by the negative_v tag
	    if (!expect(mesh.corner(twin), EXPECT_NO_BORDER)) {
                auto next_of_twin = mesh.get_next(twin);
                handles[vh].emplace_back(next_of_twin, tag::negative_v);
		// transformation taking the outgoing halfedge from this vertex on this face to the
		// face's half-edge gong to (0,0) in the negative v direction
                r = static_cast<uint8_t>((4 - dir[next_of_twin] - 1) % 4);
                t = -rotate(r, uv[twin]);
                transforms[vh].emplace_back(1, r, t);
            }
        }
    }

    return transforms;
}

// Extract two knot intervals on either side of the vertex
//   for use in computation of spans and basis functions
// Based on computations from setup_basis_funs(), this will
//   first follow a half-edge on a face and then perform any
//   T-junction operations on the same face, if they exist
// TODO -- Change from hard-coding on degree three polynomials
void surface_evaluator::calc_knots() {
    knots.clear();
    knots.reserve(handles.num_values() * 2);

    for (const auto& vh: handles) {
        knots.insert(vh, vector<array<double, 2>>());
        knots[vh].reserve(mesh.get_valence(vh));
        for (auto [h, q]: handles[vh]) {
            double s = 0;
            uint32_t j = 1;
	    // put a zero value at the back
            knots[vh].emplace_back();
            auto& current_knot = knots[vh].back();

	    // border halfedge; knots are 0,0
	    if (q == tag::border) {
		auto twin = mesh.get_twin(h);
		current_knot[j - 1] += expect(mesh.get_knot_interval(twin), EXPECT_NO_BORDER);
	        j += 1;
		
		auto next = mesh.get_next(h);
		twin = mesh.get_twin(h);
		current_knot[j - 1] += expect(mesh.get_knot_interval(twin), EXPECT_NO_BORDER);	
	        continue;
	    }

            // no edge in u direction (T-joint), so walk “around” face
            // more accurately -- walk to the next corner while accounting for
	    //  -- knot interval displacement (s)
	    //  -- halfedge (h)
	    if (q == tag::negative_v) {
                while (!expect(mesh.from_corner(h), EXPECT_NO_BORDER)) {
                    s += expect(mesh.get_knot_interval(h), EXPECT_NO_BORDER);
                    h = mesh.get_next(h);
                }
            }

            bool skip = false;
	    bool visited_once = false;
            do {
                current_knot[j - 1] += expect(mesh.get_knot_interval(h), EXPECT_NO_BORDER);

                // first intersection on ray encountered
                if (s == 0) {
                    j += 1;
                }
		

                if (j > 2) {
                    skip = true;
                    break;
                }

                h = mesh.get_next(h);
		if (visited_once && j <= 2)
		{
		    const string fail = "Cannot have two t-junctions on the same face in opposite directions";
		    panic(fail);
		}
		visited_once = true;
            } while(!expect(mesh.from_corner(h), EXPECT_NO_BORDER));

            if (skip) {
                continue;
            }

            j = 2;

	    // iterate to the portion of the mesh that corresponds to the current T-junction
            while (s >= expect(mesh.get_knot_interval(h), EXPECT_NO_BORDER)) {
                s -= expect(mesh.get_knot_interval(h), EXPECT_NO_BORDER);
                h = mesh.get_next(h);
            }

	    // flip to the next face
            auto f = expect(mesh.get_knot_factor(h), EXPECT_NO_BORDER);
            h = mesh.get_twin(h);

	    // the next face is on the border, so it has an implicit knot interval of 0
	    if (!mesh.get_face_of_half_edge(h)) {
	        current_knot[j - 1] = 0;
		continue;
	    }
            // iterate to next corner (so the same direction as current knot vector)
            while (!expect(mesh.corner(h), EXPECT_NO_BORDER)) {
                h = mesh.get_next(h);
                s += f * expect(mesh.get_knot_interval(h), EXPECT_NO_BORDER);
            }

            h = mesh.get_next(h);
            skip = false;
	    visited_once = false;
            do {
                current_knot[j - 1] += f * expect(mesh.get_knot_interval(h), EXPECT_NO_BORDER);

                if (s == 0) {
                    skip = true;
                    break;
                }
		if(visited_once)
		{
		    const string fail = "Extended T-junctions intersect; this problem must now be addressed";
		    panic(fail);
		}

		visited_once = true;
                h = mesh.get_next(h);
            } while(!expect(mesh.from_corner(h), EXPECT_NO_BORDER));

            if (skip) {
                continue;
            }
        }
    }
}


void surface_evaluator::calc_support(const basis_fun_trans_map& transforms) {
    support.clear();
    support.reserve(mesh.num_faces());
    knot_vectors.clear();
    knot_vectors.reserve(handles.num_values());

    static const uint8_t DEGREE = 3;

    // Reserve space for (in case of degree = 3 at least 8) faces
    sparse_face_map<bool> tagged(false);
    tagged.reserve(DEGREE * DEGREE);

    dense_face_map<sparse_vertex_map<bool>> added(sparse_vertex_map<bool>(false));

    for (const auto& vh: handles) {
        knot_vectors.insert(vh, vector<local_knot_vectors>());
        size_t handle_index = 0;
        for (const auto& [h, q]: handles[vh]) {
            tagged.clear();

	    // basis functions from halfedges should never be called
	    if (q == tag::border) {
		 handle_index += 1;
		 continue;
	    }
            queue<tuple<half_edge_handle, transform>> bfs_queue;

            bfs_queue.push({h, transforms[vh][handle_index]});
            auto face_h = mesh.get_face_of_half_edge(h).expect(EXPECT_NO_BORDER);
            tagged[face_h] = true;

	    // determine the domain of basis function and its support in CCW direction
            auto r = get_parametric_domain(vh, handle_index);

            // Cache local knot vectors for faster surface evaluation
            // these are computed from the knot intervals on adjacent faces
	    knot_vectors[vh].push_back(get_knot_vectors(vh, handle_index));

            while (!bfs_queue.empty()) {
		// current half-edge and current transformation
                auto [ch, ct] = bfs_queue.front();
                bfs_queue.pop();
		// face of the current half-edge
                auto cface_h = mesh.get_face_of_half_edge(ch).expect(EXPECT_NO_BORDER);
		// add this face to the support if it has not already been added
                if (!support.contains_key(cface_h)) {
                    support.insert(cface_h, vector<tuple<vertex_handle, index, transform>>());
                }

                // Only add the vertex, if it hasn't been added before
                if (!added[cface_h][vh]) {
                    support[cface_h].emplace_back(vh, handle_index, ct);
                    added[cface_h][vh] = true;
                }

                auto g = mesh.get_next(ch);
                while (g != ch) {
                    auto prev_g = mesh.get_prev(g);
                    line_segment l(ct.apply(uv[g]), ct.apply(uv[prev_g]));
                    if (!l.intersects(r)) {
                        g = mesh.get_next(g);
                        continue;
                    }

                    auto twin = mesh.get_twin(g);
                    auto twin_face_h = mesh.get_face_of_half_edge(twin).expect(EXPECT_NO_BORDER);
                    if (tagged[twin_face_h]) {
                        g = mesh.get_next(g);
                        continue;
                    }
                    tagged[twin_face_h] = true;

                    bfs_queue.push({twin, ct.apply(edge_trans[twin])});

                    g = mesh.get_next(g);
                }
            }

            handle_index += 1;
        }
    }
}

}
