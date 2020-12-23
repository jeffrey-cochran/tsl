#include <tuple>
#include <memory>
#include <algorithm>
#include <optional>

#include "tsl/util/println.hpp"
#include "tsl/geometry/tmesh/tmesh.hpp"
#include "tsl/geometry/tmesh/iterator.hpp"
#include "tsl/algorithm/get_vertices.hpp"

using std::make_unique;
using std::min;
using std::get;
using std::make_tuple;
using std::tuple;
using std::nullopt;
using std::find;
using std::make_pair;
using std::find_if;
using std::distance;

namespace tsl {

// ========================================================================
// = Modifier
// ========================================================================
vertex_handle tmesh::add_vertex(vec3 pos, bool is_control_mesh_vertex) {
    vertex v;
    v.pos = pos;
    v.control_mesh_vertex = is_control_mesh_vertex;
    return vertices.push(v);
}

bool tmesh::is_face_insertion_valid(const vector<new_face_vertex>& new_vertices) const {
    // TODO: Check this implementation for edge cases.
    // TODO: This does not detect non-manifold edges!

    // We are working with a quad mesh, so a face needs at least 4 vertices!
    if (new_vertices.size() < 4) {
        return false;
    }

    // Extract the handles from the given `new_face_vertex` and check corner and knot values
    vector<vertex_handle> new_vertex_handles;
    {
        new_vertex_handles.reserve(new_vertices.size());
        size_t num_corners = 0;
        for (const auto& new_vertex: new_vertices) {
            new_vertex_handles.push_back(new_vertex.handle);

            // Check knot values > 0
            if (new_vertex.knot <= 0) {
                return false;
            }
            if (new_vertex.corner) {
                num_corners += 1;
            }
        }
        if (num_corners != 4) {
            return false;
        }
    }

    // If there is already a face, we can't add another one
    if (get_face_between(new_vertex_handles)) {
        return false;
    }

    // Next we check that each vertex is a boundary one (it has at least one
    // boundary edge or no edges at all). We really need this property;
    // otherwise we would create non-manifold vertices and make the mesh non-
    // orientable.
    {
        vector<edge_handle> temp_edges;
        for (const auto& vh: new_vertex_handles) {
            temp_edges.clear();
            get_edges_of_vertex(vh, temp_edges);

            // No edges at all are fine too.
            if (temp_edges.empty()) {
                continue;
            }

            // But if there are some, we need at least one boundary one.
            auto boundary_edge_it = find_if(temp_edges.begin(), temp_edges.end(), [&, this](auto eh) {
                return this->num_adjacent_faces(eh) < 2;
            });
            if (boundary_edge_it == temp_edges.end()) {
                return false;
            }
        }
    }

    // TODO: Check for consistency condition
    // TODO: Check for cycle condition

    return true;
}

bool tmesh::is_face_insertion_valid(const array<vertex_handle, 4>& new_vertices) const {
    return is_face_insertion_valid({
        new_face_vertex(new_vertices[0]),
        new_face_vertex(new_vertices[1]),
        new_face_vertex(new_vertices[2]),
        new_face_vertex(new_vertices[3])
    });
}

face_handle tmesh::add_face(const vector<new_face_vertex>& new_vertices) {
    if (!is_face_insertion_valid(new_vertices)) {
        panic("attempting to add a face which cannot be added!");
    }

    // We have to create the following objects and set their attributes:
    // | type              | attribute | created/set in step |
    // |-------------------|-----------|---------------------|
    // | half_edge (inner) | -         | 1                   |
    // | half_edge (inner) | face      | 2                   |
    // | half_edge (inner) | target    | 1                   |
    // | half_edge (inner) | next      | 4                   |
    // | half_edge (inner) | prev      | 4                   |
    // | half_edge (inner) | corner    | 2                   |
    // | half_edge (inner) | knot      | 2                   |
    // | half_edge (outer) | -         | 1                   |
    // | half_edge (outer) | face      | no need to be set   |
    // | half_edge (outer) | target    | 1                   |
    // | half_edge (outer) | next      | 3                   |
    // | half_edge (outer) | prev      | 3                   |
    // | half_edge (outer) | corner    | no need to be set   |
    // | half_edge (outer) | knot      | no need to be set   |
    // | face              | -         | 2                   |
    // | face              | edge      | 2CHECK BASED CORNER |
    // | vertex            | outgoing  | 2                   |

    // =======================================================================
    // = Create broken edges (Step 1)
    // =======================================================================
    // Handles for the inner edges of the face. The edges represented by those
    // handles do not contain a valid `next` and `prev` pointer yet.
    vector<half_edge_handle> inner_handles;
    {
        inner_handles.reserve(new_vertices.size());
        for (size_t i = 1; i < new_vertices.size(); ++i) {
            auto inner_h = find_or_create_edge_between(new_vertices[i - 1], new_vertices[i]);
            inner_handles.push_back(inner_h);
        }
        auto inner_h = find_or_create_edge_between(new_vertices.back(), new_vertices.front());
        inner_handles.push_back(inner_h);
    }

    // =======================================================================
    // = Create face (Step 2)
    // =======================================================================
    face_handle new_face_h = faces.next_handle();
    {
        // Get half edge based at a corner
        auto found = find_if(new_vertices.begin(), new_vertices.end(), [&](const auto& new_vertex) {
            return new_vertex.corner;
        });
        if (found == new_vertices.end()) {
            panic("A face without four corners cannot be added to the T-Mesh! (Check your corner values.)");
        }
        auto index = distance(new_vertices.begin(), found);
        face f(inner_handles[index]);
        faces.push(f);
    }

    // Set face handle of edges and get handles for outer edges
    vector<half_edge_handle> outer_handles;
    {
        outer_handles.reserve(new_vertices.size());
        for (size_t i = 0; i < inner_handles.size(); ++i) {
            auto vertex_index = i + 1;
            if (i == inner_handles.size() - 1) {
                vertex_index = 0;
            }
            const auto& target = new_vertices[vertex_index];

            auto e_inner_h = inner_handles[i];
            auto& e_inner = get_e(e_inner_h);
            e_inner.face = optional_face_handle(new_face_h);
            e_inner.knot = target.knot;
            e_inner.corner = target.corner;
            e_inner.control_mesh_half_edge = true;
            auto e_outer_h = get_twin(e_inner_h);
            auto& e_outer = get_e(e_outer_h);
            outer_handles.push_back(e_outer_h);
            e_outer.control_mesh_half_edge = true;
        }
    }

    // =======================================================================
    // = Fix next and prev handles (Step 3)
    // =======================================================================
    // Fixing the `next` and `prev` handles is the most difficult part of this method. In
    // order to tackle it we deal with each corner of this face on its own.
    // For each corner we look at the corner-vertex and the in-going and
    // out-going edge (both edges are on the outside of this face!).
    vector<tuple<half_edge_handle, vertex_handle, half_edge_handle>> corners;
    corners.reserve(new_vertices.size());
    corners.emplace_back(outer_handles.front(), new_vertices.front().handle, outer_handles.back());
    for (size_t i = 1; i < new_vertices.size(); ++i)
    {
        corners.emplace_back(outer_handles[i], new_vertices[i].handle, outer_handles[i - 1]);
    }

    // TODO: check for more special cases due to quad faces
    for (const auto& corner: corners) {
        auto e_in_h = get<0>(corner);
        auto e_in_twin_h = get_twin(e_in_h);
        auto vh = get<1>(corner);
        auto e_out_h = get<2>(corner);
        auto e_out_twin_h = get_twin(e_out_h);

        auto& e_in = get_e(e_in_h);
        auto& v = get_v(vh);
        auto& e_out = get_e(e_out_h);

        // For each corner, we have four different cases, depending on whether
        // or not the in/outgoing edges are already part of a face.
        //
        // --> Case (A): neither edge is part of a face (both edges are new)
        if (!e_in.face && !e_out.face)
        {
            // We need to handle the special case of `v` not having an
            // outgoing edge.
            if (v.outgoing)
            {
                // First we need to find two boundary edges adjacent to `v`. One
                // ending at `v`, called e_end, and one starting at `v`, called
                // e_start. Note that `e_end.next == e_start`.
                //
                // Since we already know that `v.outgoing` does exist,
                // `find_edge_around_vertex` returning `none` means that `v` does
                // not have an adjacent boundary edge.
                //
                // A classical HEM can't deal with these kind of so called
                // non-manifold meshes. We don't expect non-manifold
                // meshes, so for now we just panic. Having this restriction to
                // manifold meshes is probably not too bad.
                auto e_end_h = find_edge_around_vertex(vh, [&, this](auto edge_h)
                {
                    return !get_e(edge_h).face;
                }, edge_direction::ingoing).expect("a non-manifold part in the mesh has been found");

                auto e_start_h = get_e(e_end_h).next;
                auto& e_start = get_e(e_start_h);

                e_in.next = e_start_h;
                e_start.prev = e_in_h;

                get_e(e_end_h).next = e_out_h;
                e_out.prev = e_end_h;
            } else
            {
                // `e_in` and `e_out` are the only edges of the vertex.
                e_in.next = e_out_h;
                e_out.prev = e_in_h;
            }
        }
            // --> Case (B): only the ingoing edge is part of a face
        else if (e_in.face && !e_out.face)
        {
            // We know that `v` has at least two outgoing edges (since
            // there is a face adjacent to it).
            //
            // We have to find the edge which `next` handle pointed to the
            // outer edge of the one face we are adjacent to (called
            // `old_next`. This is an inner edge of our face.
            auto eh = get_e(e_in_twin_h).prev;

            get_e(eh).next = e_out_h;
            e_out.prev = eh;
        }
            // --> Case (C): only the outgoing edge is part of a face
        else if (!e_in.face && e_out.face)
        {
            // This is correct, because the old next pointer are still present!
            auto old_out_h = get_e(e_out_twin_h).next;
            e_in.next = old_out_h;
            get_e(old_out_h).prev = e_in_h;
        }
            // --> Case (D): both edges are already part of another face
        else if (e_in.face && e_out.face)
        {
            // Here, two fan blades around `v` will be connected. Both blades
            // need to be in the right order for this to work. The order is
            // given by the `next` and `prev` handles of the edges with the target `v`.
            // By following those handles (and `twin` handles), we can
            // circulate around `v`.
            //
            // When circulating, both fan blades need to be next to each other.
            // If that's not the case, we need to fix a few `next` and `prev` handles. Not
            // being in the right order is caused by case (A), but it can't be
            // avoided there. Only when connecting the blades here, we can know
            // how to create the `next` and `prev` circle.
            if (get_e(e_out_twin_h).next != e_in_twin_h)
            {
                // Here we need to conceptually delete one fan blade from the
                // `next` and `prev` circle around `v` and re-insert it into the right
                // position. We choose to "move" the fan blade starting with
                // `e_in`.
                //
                // We search the edge that points to
                // `e_in.twin`.
                auto inactive_blade_end_h = get_e(e_in_twin_h).prev;

                // Instead of pointing to `e_in.twin`, it needs to "skip" the
                // `e_in` blade and point to the blade afterwards. So we need to
                // find the end of the `e_in` blade. Its next handle is the one
                // `inactive_blade_end` needs to point to.
                auto e_in_blade_end_h = find_edge_around_vertex(
                    e_in_h,
                    [&, this](auto edge_h)
                    {
                        return !get_e(edge_h).face;
                    },
                    edge_direction::ingoing
                ).unwrap();

                // We can finally set the next and prev pointer to skip the `e_in`
                // blade. After this line, circulating around `v` will work
                // but skip the whole `e_in` blade.
                get_e(inactive_blade_end_h).next = get_e(e_in_blade_end_h).next;
                get_e(get_e(e_in_blade_end_h).next).prev = inactive_blade_end_h;

                // Now we need to re-insert it again. Fortunately, this is
                // easier. After this line, the circle is broken, but it will
                // be repaired by repairing the `next` and `prev` handles within the face
                // later.
                get_e(e_in_blade_end_h).next = get_e(e_out_twin_h).next;
                get_e(get_e(e_out_twin_h).next).prev = e_in_blade_end_h;
            }
        }
    }

    // =======================================================================
    // = Set `next` and `prev` handle of inner edges (Step 4)
    // =======================================================================
    // This is an easy step, but we can't
    // do it earlier, since the old `next` and `prev` handles are required by the
    // previous "fix next and prev handles" step.
    auto& e_inner_first = get_e(inner_handles.front());
    e_inner_first.next = inner_handles[1];
    e_inner_first.prev = inner_handles.back();
    for (size_t i = 1; i < (inner_handles.size() - 1); ++i)
    {
        auto& e_inner = get_e(inner_handles[i]);
        e_inner.next = inner_handles[i + 1];
        e_inner.prev = inner_handles[i - 1];
    }
    auto& e_inner_last = get_e(inner_handles.back());
    e_inner_last.next = inner_handles.front();
    e_inner_last.prev = inner_handles[inner_handles.size() - 2];

    // =======================================================================
    // = Set outgoing handles if not set yet (Step 5)
    // =======================================================================
    // Set outgoing-handles if they are not set yet.
    for (size_t i = 0; i < new_vertices.size(); ++i)
    {
        auto& v = get_v(new_vertices[i].handle);
        auto e_inner_h = inner_handles[i];
        if (!v.outgoing)
        {
            v.outgoing = optional_half_edge_handle(e_inner_h);
        }
    }

    return new_face_h;
}

face_handle tmesh::add_face(const array<vertex_handle, 4>& new_vertices) {
    return add_face({
        new_face_vertex(new_vertices[0]),
        new_face_vertex(new_vertices[1]),
        new_face_vertex(new_vertices[2]),
        new_face_vertex(new_vertices[3])
    });
}

bool tmesh::remove_edge(edge_handle handle, bool keep_vertices) {
    // TODO: Check this implementation for edge cases.
    auto [half_edge_1_h, half_edge_2_h] = get_half_edges_of_edge(handle);
    const auto& half_edge_1 = get_e(half_edge_1_h);
    const auto& half_edge_2 = get_e(half_edge_2_h);

    // We may not delete edges near extraordinary vertices
    {
        if (is_extraordinary(half_edge_1.target) || is_extraordinary(half_edge_2.target)) {
            return false;
        }

        // find extraordinary vertices around one vertex
        auto vertices1 = get_extraordinary_vertices_in_regular_rings_around_vertex(*this, half_edge_1.target, 3);
        bool found = false;

        // if one extraordinary vertex is near both vertices of the edge, the edge is too close to that vertex
        // and we are not allowed to remove it
        visit_regular_rings(*this, half_edge_2.target, 3, [this, &vertices1, &found](auto handle, auto ring) {
            if (is_extraordinary(handle)) {
                auto in_vertices = find(vertices1.begin(), vertices1.end(), handle);
                if (vertices1.end() != in_vertices) {
                    found = true;
                    return false;
                }
            }
            return true;
        });

        if (found) {
            return false;
        }
    }

    // we cannot remove edges pointing to vertices which have 2 or less edges connected
    if (get_valence(half_edge_1.target) < 3 || get_valence(half_edge_2.target) < 3) {
        return false;
    }

    // both half edges need to point into a face corner, otherwise we would create not qaudratic faces
    if (!(*from_corner(half_edge_1_h)) || !(*half_edge_1.corner) || !(*from_corner(half_edge_2_h)) || !(*half_edge_2.corner)) {
        return false;
    }

    // if we were told to keep all vertices, we are not allowed to remove edges poiting to t-vertices
    if (keep_vertices && (get_valence(half_edge_1.target)  == 3 || get_valence(half_edge_2.target)  == 3)) {
        return false;
    }

    // TODO: check cylce and consistency conditions, if edge would be removed!

    // we cannot remove border edges
    if (!half_edge_1.face || !half_edge_2.face) {
        return false;
    }
    auto face1_h = half_edge_1.face.unwrap();
    auto face2_h = half_edge_2.face.unwrap();

    // get vertices
    auto vertex_1_h = half_edge_1.target;
    auto vertex_2_h = half_edge_2.target;
    auto& vertex_1 = get_v(vertex_1_h);
    auto& vertex_2 = get_v(vertex_2_h);

    // we have to fix:
    // - prev
    // - next
    // - corner
    // - out (of vertex)
    // - edge (of face)
    // - face (of inner edges of face 1)

    // the face of half edge 1 is going to be deleted by this operation, store all inner edge handles, to fix their
    // face pointer
    auto inner_edges_of_face1 = get_half_edges_of_face(face1_h);

    // fix prev and next
    auto& next_1 = get_e(half_edge_1.next);
    auto& prev_1 = get_e(half_edge_1.prev);
    auto& next_2 = get_e(half_edge_2.next);
    auto& prev_2 = get_e(half_edge_2.prev);

    next_1.prev = half_edge_2.prev;
    next_2.prev = half_edge_1.prev;
    prev_1.next = half_edge_2.next;
    prev_2.next = half_edge_1.next;

    // fix corner
    prev_1.corner = false;
    prev_2.corner = false;

    // fix out (of vertex)
    if (vertex_1.outgoing.unwrap() == half_edge_2_h) {
        vertex_1.outgoing = optional_half_edge_handle(half_edge_1.next);
    }
    if (vertex_2.outgoing.unwrap() == half_edge_1_h) {
        vertex_2.outgoing = optional_half_edge_handle(half_edge_2.next);
    }

    // fix edge (of face 2)
    auto& face2 = get_f(face2_h);
    if (!*from_corner(face2.edge)) {
        // Find a half edge which is based at a corner
        optional_half_edge_handle based_at_corner;
        circulate_in_face(half_edge_2.prev, [&based_at_corner, this](auto handle) {
            if (*from_corner(handle)) {
                based_at_corner = optional_half_edge_handle(handle);
                return false;
            }
            return true;
        });

        face2.edge = based_at_corner.expect("No edge based at a corner found in face!");
    }

    // fix face (of inner edges of face 1)
    for (const auto& eh: inner_edges_of_face1) {
        auto& half_edge = get_e(eh);
        half_edge.face = optional_face_handle(face2_h);
    }

    // actually delete the edge and face 1
    edges.erase(half_edge_1_h);
    edges.erase(half_edge_2_h);
    faces.erase(face1_h);

    // we need to fix invalid edges created because of removed t-edges
    // the situation we end up in looks like this: with vertex_1_h or vertex_2_h beeing v4
    // v0 ===== v1 ===== v2
    // ||       f1       ||
    // ||                ||
    // || he11>    he21> ||
    // v3 ===== v4 ===== v5
    // || <he12    <he22 ||
    // ||                ||
    // ||       f2       ||
    // v6 ===== v7 ===== v8

    // we need to remove v4, he11 and he12 and connect he21 and he22 with v3
    auto vertices_to_check = {vertex_1_h, vertex_2_h};
    for (const auto& vh: vertices_to_check) {
        if (get_valence(vh) == 2) {

            auto v4_h = vh;
            auto v4 = get_v(v4_h);
            auto he12_h = v4.outgoing.unwrap();
            auto he11_h = get_twin(he12_h);
            auto he22_h = get_prev(he12_h);
            auto he21_h = get_twin(he22_h);

            auto he12 = get_e(he12_h);
            auto he11 = get_e(he11_h);
            auto& he21 = get_e(he21_h);
            auto& he22 = get_e(he22_h);

            // we have to fix:
            // - prev
            // - next
            // - corner (of he22)
            // - target (of he22)
            // - knots
            // - out (of v3)
            // - edge (of f1)

            // fix prev and next
            auto& prev_11 = get_e(he11.prev);
            auto& next_12 = get_e(he12.next);
            next_12.prev = he22_h;
            prev_11.next = he21_h;
            he21.prev = he11.prev;
            he22.next = he12.next;

            // fix corner
            he22.corner = true;

            // fix target
            he22.target = he12.target;

            // fix knots
            he22.knot = (*he22.knot) + (*he12.knot);
            he21.knot = (*he21.knot) + (*he11.knot);

            // fix out (of v3)
            auto& v3 = get_v(he12.target);
            if (v3.outgoing.unwrap() == he11_h) {
                v3.outgoing = optional_half_edge_handle(he21_h);
            }

            // fix edge (of face 1)
            auto& f1 = get_f(he11.face.unwrap());
            if (f1.edge == he11_h) {
                f1.edge = he21_h;
            }

            // actually delete the vertex and half edges
            edges.erase(he11_h);
            edges.erase(he12_h);
            vertices.erase(v4_h);
        }
    }

    return true;
}

void tmesh::zero_transverse_boundary_edge_knot_intervals() 
{
    for (const auto& vh: get_vertices()) {
	if(is_border_vertex(vh)) {
	    auto valence = get_valence(vh);
	    // for valence two vertices, change all knots on half-edges which have a face
	    if (valence == 2) {
	        for(const auto& heh: get_half_edges_of_vertex(vh, edge_direction::outgoing)) {
		    if(!is_border(heh)) {
		        auto& he = get_e(heh);
			he.knot = 0;
		    }
		}
		for(const auto& heh: get_half_edges_of_vertex(vh, edge_direction::ingoing)) {
		    if(!is_border(heh)) {
		        auto& he = get_e(heh);
			he.knot = 0;
		    }
		}
	    }
	    else {
	        // iterate and make all half-edges transverse to the boundary have zero knot interval
	        for (const auto& heh: get_half_edges_of_vertex(vh, edge_direction::outgoing)) {
                    if (!is_border(heh) && !is_border(get_twin(heh))) {
	                edge_to_zero_knot_interval(heh);
		    }
	        }
	    }
        }            
    }

}

void tmesh::extend_to_bezier_mesh()
{
    // the mesh has not yet been extended to the Bezier mesh; extend it
    if (!is_bezier_mesh) {

        // store the current t-junctions
        vector<half_edge_handle> t_juncs;
        t_juncs.reserve(num_t_junctions());
        // iterate through each t-junction
        for(const auto& heh: get_half_edges())
        {
            auto temp = from_corner(heh);
            if (temp) {
                if (!(*temp)) {
                    t_juncs.push_back(heh);
                }
            }
        }
        
        // iterate through all t-junctions
        int max_iter_n = 100;
        int curr_iter_n = 0;
        for(auto& heh: t_juncs)
        {
            // extend the T-junction two control bays
            int n_additional_bays = 2;
            curr_iter_n = 0;
            half_edge_handle iter = heh;
            do {
                // extend first set of t-junctions
                if(expect(corner(iter), "corners should be defined on all t-junctions"))
                {
                    // the T-junction has already been merged with a different T-junction
                    // do not perform any additional operations
                    break;
                }

                // extend this t junction one bay
                iter = split_face_at_t_junction(iter, false, true).expect("optinal half edge should be valid if not on boundary");
                if(is_control_half_edge(iter)) {
                    --n_additional_bays;
                }
                
                // iterate to the next half edge
                iter = get_next(iter);
                iter = get_twin(iter);
                iter = get_next(iter);
                iter = get_twin(iter);
                
                // stop iteration if on the border
                if(is_border(iter))
                    break;
                // iterate on max iteration number
                ++curr_iter_n;
            } while(n_additional_bays > 0 && curr_iter_n < max_iter_n);
        }

        // flag this as the Bezier mesh
        is_bezier_mesh = true;
    }
}

// ========================================================================
// = Get numbers
// ========================================================================
size_t tmesh::num_vertices() const
{
    return vertices.num_used();
}

size_t tmesh::num_control_vertices() const
{
    return num_vertices() - num_virtual_vertices();
}

size_t tmesh::num_virtual_vertices() const
{
    return count_virtual_vertices;
}

size_t tmesh::num_t_junctions() const
{
    size_t num = 0;
    for(const auto& heh: get_half_edges())
    {
        auto temp = from_corner(heh);
        if (temp) {
            if (!(*temp)) {
                ++num;
            }
        }
    }
    return num;
}

size_t tmesh::num_faces() const
{
    return faces.num_used();
}

size_t tmesh::num_edges() const
{
    return edges.num_used() / 2;
}

size_t tmesh::num_control_edges() const
{
    return num_edges() - num_virtual_edges();
}

size_t tmesh::num_virtual_edges() const
{
    return count_virtual_half_edges / 2;
}

size_t tmesh::num_half_edges() const
{
    return edges.num_used();
}

size_t tmesh::num_control_half_edges() const
{
    return num_half_edges() - num_virtual_half_edges();
}

size_t tmesh::num_virtual_half_edges() const
{
    return count_virtual_half_edges;
}

uint8_t tmesh::num_adjacent_faces(edge_handle handle) const
{
    auto faces_of_edge = get_faces_of_edge(handle);
    return static_cast<uint8_t>((faces_of_edge[0] ? 1 : 0) + (faces_of_edge[1] ? 1 : 0));
}

size_t tmesh::get_valence(vertex_handle handle) const {
    index valence = 0;
    circulate_around_vertex(handle, [&](auto ingoing_edge_h)
    {
        valence += 1;
        return true;
    });
    return valence;
}

size_t tmesh::get_extended_valence(vertex_handle handle) const {
    uint32_t valence = 0;
    for (const auto& e: get_half_edges_of_vertex(handle, edge_direction::ingoing)) {
        valence += 1;
        // (1) corner is defined on half-edges AND
        // (2) halfedge is not a corner of the given face
        if (corner(e) && !*corner(e)) {
            valence += 1;
        }
    }
    return valence;
}

bool tmesh::is_extraordinary(vertex_handle handle) const {
    auto border_vertex = is_border_vertex(handle);
    if (border_vertex) {
        return get_extended_valence(handle) != 3;
    }
    else {
        return get_extended_valence(handle) != 4;
    }
}

// ========================================================================
// = Get attributes
// ========================================================================
vec3 tmesh::get_vertex_position(vertex_handle handle) const
{
    return get_v(handle).pos;
}

vec3& tmesh::get_vertex_position(vertex_handle handle)
{
    return get_v(handle).pos;
}

optional<double> tmesh::get_knot_interval(half_edge_handle handle) const {
    return get_e(handle).knot;
}

optional<bool> tmesh::facial_parametric_domain_is_degenerate(face_handle handle) const {
    bool found_corner = false;
    bool one_parametric_dimension = false;
    int corner_count = 0;
    for (auto const& heh: get_half_edges_of_face(handle)) {
        // iterate to a corner of the face
	if (!found_corner) {
	    auto temp = from_corner(heh);
	    if (temp) {
	        found_corner = *temp;
	    } else {
	        // no corners are found
		return nullopt;
	    }
	    continue;
	}
	else {
	    auto knot_interval = get_knot_interval(heh);
	    //knot interval is defined
	    if (knot_interval) {
		// knot interval is greater than zer0
		if (*knot_interval > 0) {
		    // non-zero intervals in two parametric dimensions
		    if (one_parametric_dimension && corner_count > 0) {
		        return false;
		    } else {
			// non-zero interval in one parametric dimension
		        one_parametric_dimension = true;
		    }
		}
		// if we reach a corner, iterate
		if (expect(corner(heh), "Cannot have corners only defined in part of the facial domain")) {
		    ++corner_count;
		    if (corner_count == 2) {
		        return true;
		    }
		}
	    }
	    // knot intervals are not defined
	    else {
	        return nullopt;
	    }
	}
    }

    // default (which should never be hit
    panic("Should not ever reach this; knot intervals and corners are ill-defined on the mesh");
    return nullopt;
}

optional<bool> tmesh::corner(half_edge_handle handle) const {
    return get_e(handle).corner;
}

optional<bool> tmesh::from_corner(half_edge_handle handle) const {
    return get_e(get_e(handle).prev).corner;
}

optional<double> tmesh::get_knot_factor(half_edge_handle handle) const {
    auto knot_interval = get_knot_interval(handle);
    auto knot_interval_twin = get_knot_interval(get_twin(handle));
    if (knot_interval && knot_interval_twin) {
	if (*knot_interval_twin == 0) {
	    return 1.0;
	} else {
            return *knot_interval / *knot_interval_twin;
        }
    }
    return nullopt;
}

void tmesh::get_opposite_location(
	half_edge_handle init_heh,
        double init_barycentric_interval,
	half_edge_handle& terminal_heh,
	double& terminal_barycentric_interval) {
    // ensure that the corner information is defined on the input half-edge (i.e. we're on a face)
    if (!corner(init_heh) || !from_corner(init_heh)) {
        panic("Must split on a face, not border half edges");
    }

    // if it is on a corner, iterate to the next corner
    if (expect(from_corner(init_heh), "Must split on a face, not a border half edge") && 
	init_barycentric_interval == 0) {
        terminal_heh = get_prev(init_heh);
	do {
            terminal_heh = get_prev(terminal_heh);
	} while (expect(corner(terminal_heh), "faces should have well-defined corner halfedges"));
	terminal_barycentric_interval = 1;
        return;
    } else if (expect(corner(init_heh), "Must split on a face, not a border half edge (2)") &&
	       init_barycentric_interval == 1) {
        terminal_heh = get_next(init_heh);
	do {
	    terminal_heh = get_next(terminal_heh);
	} while (expect(from_corner(terminal_heh), "faces should have well-defined corner halfedges (2)"));
	terminal_barycentric_interval = 0;
        return;
    }
    double epsilon = 1e-12;
    
    // otherwise, we aren't at a corner; iterate to the opposite location
    // initialize with location's uv coordinate at zero; account for the rest of the half-edge
    double s = (1-init_barycentric_interval) * expect(get_knot_interval(init_heh), "knot intervals must be defined on the interior of faces");

    bool is_zero_interval = expect(get_knot_interval(init_heh), "knot on interior of face should be well-defined") == 0;
    // store knot interval information
    auto iter = init_heh;
    // iterate to a corner while summing intervals in s
    while (!expect(corner(iter),"valid face should have corners defined")) {
        iter = get_next(iter);
	s += expect(get_knot_interval(iter), "knot on interior of face should be well-defined (3)");
    }

    // iterate to the next corner
    do {
        iter = get_next(iter);
    } while (!expect(corner(iter), "valid face should have corners defined"));
    
    // iterate to the parametric domain across from the start location
    double next_interval = expect(get_knot_interval(get_next(iter)), "knot on interior of face should be well-defined (4)");
    while (s - next_interval > epsilon) {
	s -= next_interval;
        iter = get_next(iter);
        next_interval = expect(get_knot_interval(get_next(iter)), "knot on interior of face should be well-defined (5)");
    } while (s > epsilon);

    // the next half-edge contains the opposite edge
    // the next half-edge has zero length, so take this as a 0 interval
    if (next_interval < epsilon) {
        terminal_barycentric_interval = 1;
	terminal_heh = get_next(iter);
	return;
    }
    // find how far in until on the opposite side
    terminal_barycentric_interval = s / next_interval;
    if (terminal_barycentric_interval > 1-2*epsilon) {
        terminal_barycentric_interval = 1;
    }
    terminal_heh = get_next(iter);
    return;
}

// ========================================================================
// = Set attributes
// ========================================================================
bool tmesh::set_knot_interval(half_edge_handle handle, double interval) {
    // do not allow input of intervals on regions in which they are not
    // already defined
    if (!get_knot_interval(handle)) {
        return false;
    } else
    {
        auto& he = get_e(handle);
	he.knot = interval;
	return true;
    }
}

// ========================================================================
// = Follow pointer
// ========================================================================
half_edge_handle tmesh::get_twin(half_edge_handle handle) const {
    return half_edge_handle(handle.get_idx() ^ 1);
}

half_edge_handle tmesh::get_prev(half_edge_handle handle) const
{
    return get_e(handle).prev;
}

half_edge_handle tmesh::get_next(half_edge_handle handle) const
{
    return get_e(handle).next;
}

vertex_handle tmesh::get_target(half_edge_handle handle) const
{
    return get_e(handle).target;
}

optional_half_edge_handle tmesh::get_out(vertex_handle handle) const {
    return get_v(handle).outgoing;
}

half_edge_handle tmesh::get_edge(face_handle handle) const {
    return get_f(handle).edge;
}

// ========================================================================
// = Get vertices
// ========================================================================
vector<vertex_handle> tmesh::get_vertices_of_face(face_handle handle) const
{
    vector<vertex_handle> vertices_out;
    vertices_out.reserve(4);
    get_vertices_of_face(handle, vertices_out);
    return vertices_out;
}

void tmesh::get_vertices_of_face(face_handle handle, vector<vertex_handle>& vertices_out) const
{
    circulate_in_face(handle, [&vertices_out, this](auto eh)
    {
        vertices_out.push_back(get_e(eh).target);
        return true;
    });
}

void tmesh::get_vertices_of_face(face_handle handle, set<vertex_handle>& vertices_out) const
{
    circulate_in_face(handle, [&vertices_out, this](auto eh)
    {
        vertices_out.insert(get_e(eh).target);
        return true;
    });
}

array<vertex_handle, 2> tmesh::get_vertices_of_edge(edge_handle edge_h) const
{
    auto one_edge_h = half_edge_handle::one_half_of(edge_h);
    const auto& one_edge = get_e(one_edge_h);
    return {one_edge.target, get_e(get_twin(one_edge_h)).target};
}

void tmesh::get_vertices_of_edge(edge_handle edge_h, vector<vertex_handle>& vertices_out) const {
    auto one_edge_h = half_edge_handle::one_half_of(edge_h);
    const auto& one_edge = get_e(one_edge_h);
    vertices_out.push_back(one_edge.target);
    vertices_out.push_back(get_e(get_twin(one_edge_h)).target);
}

void tmesh::get_vertices_of_edge(edge_handle edge_h, set<vertex_handle>& vertices_out) const {
    auto one_edge_h = half_edge_handle::one_half_of(edge_h);
    const auto& one_edge = get_e(one_edge_h);
    vertices_out.insert(one_edge.target);
    vertices_out.insert(get_e(get_twin(one_edge_h)).target);
}

array<vertex_handle, 2> tmesh::get_vertices_of_half_edge(half_edge_handle edge_h) const
{
    const auto& one_edge = get_e(edge_h);
    return {one_edge.target, get_e(get_twin(edge_h)).target};
}

bool tmesh::is_border_vertex(vertex_handle vh) const {
    // iterate over all halfedges of this vertex
    for (const auto& heh: get_half_edges_of_vertex(vh, edge_direction::outgoing)) {
        if( is_border(heh)) {
	    return true;
	}
    }
    
    return false;
}

bool tmesh::is_control_vertex(vertex_handle vh) const {
    const auto& v = get_v(vh);
    return v.control_mesh_vertex;
}

// ========================================================================
// = Get faces
// ========================================================================
array<optional_face_handle, 2> tmesh::get_faces_of_edge(edge_handle edge_h) const
{
    auto one_edge_h = half_edge_handle::one_half_of(edge_h);
    const auto& one_edge = get_e(one_edge_h);
    return {one_edge.face, get_e(get_twin(one_edge_h)).face};
}

vector<face_handle> tmesh::get_faces_of_vertex(vertex_handle vh) const {
    vector<face_handle> out;
    get_faces_of_vertex(vh, out);
    return out;
}

void tmesh::get_faces_of_vertex(vertex_handle vh, vector<face_handle>& faces_out) const {
    circulate_around_vertex(vh, [&faces_out, this](auto eh)
    {
        const auto& e = get_e(eh);
        if (e.face) {
            faces_out.push_back(e.face.unwrap());
        }
        return true;
    });
}

optional_face_handle tmesh::get_face_of_half_edge(half_edge_handle edge_h) const
{
    const auto& edge = get_e(edge_h);
    return edge.face;
}

void tmesh::get_neighbours_of_face(face_handle handle, vector<face_handle>& faces_out) const
{
    auto inner_edges = get_half_edges_of_face(handle);
    for (const auto& eh: inner_edges)
    {
        const auto& twin = get_e(get_twin(eh));
        if (twin.face)
        {
            faces_out.push_back(twin.face.unwrap());
        }
    }
}

vector<face_handle> tmesh::get_neighbours_of_face(face_handle handle) const
{
    vector<face_handle> out;
    get_neighbours_of_face(handle, out);
    return out;
}

optional_face_handle tmesh::get_face_between(const vector<vertex_handle>& handles) const {
    // TODO: This is more compilcated! what if a face of v0, v1, v2, v3 exists and we try to create a face
    //       with v0-v6? Search for more edge cases!
    // TODO: Check this implementation for edge cases.
    if (handles.size() < 4) {
        panic("too few vertex handles! A face in a quad mesh needs at least four vertices!");
    }

    // Start with one edge
    const auto ab_edge_h = get_edge_between(handles[0], handles[1]);

    // If the edge already doesn't exist, there won't be a face either.
    if (!ab_edge_h) {
        return optional_face_handle();
    }

    // The two faces of the edge. One of these faces should contain the vertex
    // `c` or there is just no face between the given vertices.
    const auto faces_of_edge = get_faces_of_edge(ab_edge_h.unwrap());

    // Find the face which contains vertex `c`.
    auto face_it = find_if(faces_of_edge.begin(), faces_of_edge.end(), [&, this](auto maybe_face_h) {
        if (!maybe_face_h) {
            return false;
        }
        const auto vertices_of_face = this->get_vertices_of_face(maybe_face_h.unwrap());
        return find(vertices_of_face.begin(), vertices_of_face.end(), handles[2]) != vertices_of_face.end();
    });

    return face_it != faces_of_edge.end() ? *face_it : optional_face_handle();
}

// ========================================================================
// = Get edges
// ========================================================================
bool tmesh::is_border(half_edge_handle handle) const {
    const auto& he = get_e(handle);
    return !he.face;
}

bool tmesh::is_control_half_edge(half_edge_handle handle) const {
    const auto& he = get_e(handle);
    return he.control_mesh_half_edge;
}

bool tmesh::is_control_edge(edge_handle handle) const {
    auto heh = half_edge_handle::one_half_of(handle);
    return is_control_half_edge(heh);
}

void tmesh::get_edges_of_vertex(
    vertex_handle handle,
    vector<edge_handle>& edges_out
) const
{
    circulate_around_vertex(handle, [&edges_out, this](auto eh)
    {
        edges_out.push_back(half_to_full_edge_handle(eh));
        return true;
    });
}

vector<edge_handle> tmesh::get_edges_of_vertex(vertex_handle handle) const
{
    vector<edge_handle> out;
    get_edges_of_vertex(handle, out);
    return out;
}

void tmesh::get_half_edges_of_vertex(
    vertex_handle handle,
    vector<half_edge_handle>& edges_out,
    edge_direction way
) const
{
    circulate_around_vertex(handle, [&edges_out](auto eh)
    {
        edges_out.push_back(eh);
        return true;
    }, way);
}

vector<half_edge_handle> tmesh::get_half_edges_of_vertex(vertex_handle handle, edge_direction way) const
{
    vector<half_edge_handle> out;
    get_half_edges_of_vertex(handle, out, way);
    return out;
}

vector<half_edge_handle> tmesh::get_half_edges_of_face(face_handle face_handle) const
{
    vector<half_edge_handle> edges_out;
    edges_out.reserve(4);
    circulate_in_face(face_handle, [&edges_out](auto eh)
    {
        edges_out.push_back(eh);
        return true;
    });
    return edges_out;
}

half_edge_handle tmesh::get_half_edge_of_edge(edge_handle edge_h) const
{
    return half_edge_handle::one_half_of(edge_h);
}

array<half_edge_handle, 2> tmesh::get_half_edges_of_edge(edge_handle edge_h) const
{
    auto heh = half_edge_handle::one_half_of(edge_h);
    return {heh, get_twin(heh)};
}

optional_edge_handle tmesh::get_edge_between(vertex_handle ah, vertex_handle bh) const
{
    // Go through all edges of vertex `a` until we find an edge that is also
    // connected to vertex `b`.
    for (const auto& eh: get_edges_of_vertex(ah))
    {
        auto endpoints = get_vertices_of_edge(eh);
        if (endpoints[0] == bh || endpoints[1] == bh)
        {
            return optional_edge_handle(eh);
        }
    }
    return optional_edge_handle();
}

optional_half_edge_handle tmesh::get_half_edge_between(vertex_handle ah, vertex_handle bh) const
{
    // Go through all edges of vertex `a` until we find an edge that is also
    // connected to vertex `b`.
    return find_edge_around_vertex(ah, [&, this](auto current_edge_h) {
        return get_e(current_edge_h).target == bh;
    }, edge_direction::outgoing);
}

optional_half_edge_handle tmesh::get_half_edge_between(face_handle ah, face_handle bh) const
{
    optional_half_edge_handle out;
    circulate_in_face(ah, [&, this](auto eh)
    {
        const auto& twin = get_e(get_twin(eh));
        if (twin.face && twin.face.unwrap() == bh)
        {
            out = optional_half_edge_handle(eh);
            return false;
        }
        return true;
    });

    return out;
}

// ========================================================================
// = Iterator helper
// ========================================================================
tmesh_iterator_ptr<vertex_handle> tmesh::vertices_begin() const
{
    return tmesh_iterator_ptr<vertex_handle>(
        make_unique<tmesh_fev_iterator<vertex_handle, vertex>>(vertices.begin())
    );
}

tmesh_iterator_ptr<vertex_handle> tmesh::vertices_end() const
{
    return tmesh_iterator_ptr<vertex_handle>(
        make_unique<tmesh_fev_iterator<vertex_handle, vertex>>(vertices.end())
    );
}

tmesh_iterator_ptr<face_handle> tmesh::faces_begin() const
{
    return tmesh_iterator_ptr<face_handle>(
        make_unique<tmesh_fev_iterator<face_handle, face>>(faces.begin())
    );
}

tmesh_iterator_ptr<face_handle> tmesh::faces_end() const
{
    return tmesh_iterator_ptr<face_handle>(
        make_unique<tmesh_fev_iterator<face_handle, face>>(faces.end())
    );
}

tmesh_iterator_ptr<half_edge_handle> tmesh::half_edges_begin() const
{
    return tmesh_iterator_ptr<half_edge_handle>(
        make_unique<tmesh_fev_iterator<half_edge_handle, half_edge>>(edges.begin())
    );
}

tmesh_iterator_ptr<half_edge_handle> tmesh::half_edges_end() const
{
    return tmesh_iterator_ptr<half_edge_handle>(
        make_unique<tmesh_fev_iterator<half_edge_handle, half_edge>>(edges.end())
    );
}

tmesh_iterator_ptr<edge_handle> tmesh::edges_begin() const
{
    return tmesh_iterator_ptr<edge_handle>(
        make_unique<tmesh_edge_iterator>(edges.begin(), *this)
    );
}

tmesh_iterator_ptr<edge_handle> tmesh::edges_end() const
{
    return tmesh_iterator_ptr<edge_handle>(
        make_unique<tmesh_edge_iterator>(edges.end(), *this)
    );
}

tmesh_face_iterator_proxy tmesh::get_faces() const
{
    return tmesh_face_iterator_proxy(*this);
}

tmesh_half_edge_iterator_proxy tmesh::get_half_edges() const
{
    return tmesh_half_edge_iterator_proxy(*this);
}

tmesh_edge_iterator_proxy tmesh::get_edges() const
{
    return tmesh_edge_iterator_proxy(*this);
}

tmesh_vertex_iterator_proxy tmesh::get_vertices() const
{
    return tmesh_vertex_iterator_proxy(*this);
}

// ========================================================================
// = Private helper methods
// ========================================================================
half_edge& tmesh::get_e(half_edge_handle handle)
{
    return edges[handle];
}

const half_edge& tmesh::get_e(half_edge_handle handle) const
{
    return edges[handle];
}

face& tmesh::get_f(face_handle handle)
{
    return faces[handle];
}

const face& tmesh::get_f(face_handle handle) const
{
    return faces[handle];
}

vertex& tmesh::get_v(vertex_handle handle)
{
    return vertices[handle];
}

const vertex& tmesh::get_v(vertex_handle handle) const
{
    return vertices[handle];
}

void tmesh::edge_to_zero_knot_interval(half_edge_handle handle)
{
    auto twin_handle = get_twin(handle);
    // change the knot intervals on handle and handle's twin
    auto& hedge = get_e(handle);
    auto& twin = get_e(twin_handle);
    
    hedge.knot = 0;
    twin.knot = 0;
}

half_edge_handle tmesh::find_or_create_edge_between(new_face_vertex from_h, new_face_vertex to_h) {
    return find_or_create_edge_between(from_h.handle, to_h.handle);
}

half_edge_handle tmesh::find_or_create_edge_between(vertex_handle from_h, vertex_handle to_h) {
    auto found_edge = get_half_edge_between(from_h, to_h);
    if (found_edge) {
        return found_edge.unwrap();
    } else {
        return add_edge_pair(from_h, to_h).first;
    }
}

pair<half_edge_handle, half_edge_handle> tmesh::add_edge_pair(vertex_handle v1h, vertex_handle v2h) {
    // This method adds two new half edges, called "a" and "b".
    //
    //  +----+  --------(a)-------->  +----+
    //  | v1 |                        | v2 |
    //  +----+  <-------(b)---------  +----+

    // Create incomplete/broken edges and edge handles. By the end of this
    // method, they are less invalid.
    half_edge a;
    half_edge b;

    // Add edges to our edge list.
    auto ah = edges.push(a);
    auto bh = edges.push(b);
    auto& a_inserted = get_e(ah);
    auto& b_inserted = get_e(bh);

    // Assign half-edge targets
    a_inserted.target = v2h;
    b_inserted.target = v1h;

    return make_pair(ah, bh);
}

edge_handle tmesh::half_to_full_edge_handle(half_edge_handle handle) const {
    auto twin = get_twin(handle);
    // return the handle with the smaller index of the given half edge and its twin
    return edge_handle(min(twin.get_idx(), handle.get_idx()));
}

optional_half_edge_handle tmesh::split_face_at_t_junction(half_edge_handle handle, bool split_to_control_mesh, bool extend_virtual_vertices) {
    // ----------------------------------------
    // Preprocess for validity
    // ----------------------------------------

    // do not operate on border half edges
    if (is_border(handle)) {
        return optional_half_edge_handle(); // an empty handle
    }
    // check that the input half edge is a t-junction.
    // (1) corners are defined on the mesh
    // (2) this half edge handle is a corner 
    else if (corner(handle) && *corner(handle)) {
        return optional_half_edge_handle(); // an empty handle
    }
    // the edge is a T-junction, but it is virtual
    // do not extend if we are instructed not to extend
    else if(!extend_virtual_vertices && !is_control_vertex(get_target(handle))) {
        return optional_half_edge_handle(); // an empty handle
    }

    // the half edge points to a T-junction and the T-junction is to be extended
    double eps = 1e-12;

    // ------------------------------------------
    // Iterate to the opposite side of T-junction
    // ------------------------------------------
	
    bool is_zero_interval;
    // map the face of this half-edge to a t-junction extension face
    auto fh = get_face_of_half_edge(handle).expect("the specified half-edge must have a face");
    // note this t-junction as associated with this face

    // store knot interval information
    is_zero_interval = expect(get_knot_interval(handle), "knot on interior of face should be well-defined (1)") == 0;
    auto iter = get_next(handle);
    auto s = expect(get_knot_interval(iter), "knot on interior of face should be well-defined (2)"); 
    // iterate to a corner
    while (!expect(corner(iter), "valid face should have corners defined")) {
        iter = get_next(iter);
        s += expect(get_knot_interval(iter), "knot on interior of face should be well-defined (3)");
    }

    // iterate to the next corner
    iter = get_next(iter);
    auto transverse_s = expect(get_knot_interval(iter), "knot in transverse direction should be well-defined (1)");
    while (!expect(corner(iter), "valid face should have corners defined")) {
		iter = get_next(iter);
        transverse_s += expect(get_knot_interval(iter), "knot in transverse direction should be well-defined (2)");
    } 
    // iterate to the parametric domain across from the T-junction
    iter = get_next(iter);
    auto t = expect(get_knot_interval(iter), "knot on interior of face should be well-defined (4)");
    // stop when half-edge reaches or passes the target domain
    while (t < s + eps) {
		// decrement the parametric domain
		s -= t;
	    // iterate to the next half edge
		iter = get_next(iter); 
		// find length of the parametric domain
		t = expect(get_knot_interval(iter), "knot on interior of face should be well-defined (4.5)");
    }

    // ----------------------------------------
    // Split the topology
    // ----------------------------------------

    // if we're within epsilon of zero and the initial knot interval was not zero,
    // store the opposite vertex
    // TODO -- this assumes that we never have two T-junctions separated by a zero
    //         knot interval on a face... zero knot intervals are expected next to corners
    if (fabs(s-t) <= eps && !is_zero_interval) {
		// mark this t-junction as exactly reaching an opposite vertex
    
        // do nothing
    }
    // otherwise, we're in the interior of an existing edge and we do not align with an
    // existing T-junction on the opposite side	
    else {
        // convert from interval to barycentric coordinates
        auto minval = s-t;
        auto maxval = s;
        // we are looking for the point 0 in the interval [s-t, s] and converting this location
        //    to \xi in [0, 1]
        // this translates into the following
        auto xi = minval / (minval + maxval);
        // now that we have converted to barycentric coordinates, split the edge
        iter = split_edge_at_interval(iter, xi, split_to_control_mesh);
        // TODO -- split the face from original half edge's target vertex to the split edge
    }

    // TODO account for corners of half-edges appropriately on these new, split edges
    split_t_face_at_half_edges(handle, iter, split_to_control_mesh, transverse_s);
    // return the half edge pointing to the vertex extending the input t-junction
    return optional_half_edge_handle(iter);
}

half_edge_handle tmesh::split_edge_at_interval(half_edge_handle handle, double barycentric_interval, bool is_control_mesh_vertex) {
    if (barycentric_interval < 0) {
        panic("Cannot process negative barycentric interval");
    } else if (barycentric_interval > 1) {
        panic("Cannot extrapolate edge interval");
    }

    expect(get_knot_interval(handle), "Cannot split a half-edge which does not have a knot interval");

    // CREATE NEW VERTEX
    // find the location of the new vertex location
    auto to_vh = get_target(handle);
    auto from_vh = get_target(get_twin(handle));

    vec3 pos;
    if (barycentric_interval == 0) {
        pos = (0.75) * get_vertex_position(from_vh) + (0.25) * get_vertex_position(to_vh);
    } else if (barycentric_interval == 1) {
        pos = (0.25) * get_vertex_position(from_vh) + (0.75) * get_vertex_position(to_vh);
    } else {
        pos = (1-barycentric_interval) * get_vertex_position(from_vh) + barycentric_interval * get_vertex_position(to_vh);
    }
    // add the vertex with the location
    auto split_vh = add_vertex(pos, is_control_mesh_vertex);
    if (!is_control_mesh_vertex) {
        ++count_virtual_vertices;
    }


    //    ------------init_opp--------------------->
    //  x--------------------------------------------x
    //  v1  <---------init_he----------------------- v2
    //
    //  Goes to
    //
    //
    //       new_opp              init_opp
    //     -------------->   ---------------------->
    //  x------------------x-------------------------x
    //  v1 <------------- v3 <---------------------- v2
    //       new_he               init_he

    // CREATE NEW HALFEDGE DATA
    auto new_heh = find_or_create_edge_between(split_vh,to_vh);
    auto init_twin_h = get_twin(handle);

    auto& init_he = get_e(handle);
    auto& init_opp = get_e(init_twin_h);
    auto& new_he = get_e(new_heh);
    auto& new_opp = get_e(get_twin(new_heh));

    // Fix new vertex data
    auto& split_v = get_v(split_vh);
    split_v.outgoing = optional_half_edge_handle(new_heh);

    // adjust half-edge data for the new half-edges
    // face data
    if (get_face_of_half_edge(handle)) {
        new_he.face = optional_face_handle(get_face_of_half_edge(handle));
    }
    if (get_face_of_half_edge(init_twin_h)) {
        new_opp.face = optional_face_handle(get_face_of_half_edge(init_twin_h));
    }
    // target data already complete
    // next data
    new_he.next = init_he.next;
    new_opp.next = init_twin_h;
    // previous data
    new_he.prev = handle;
    new_opp.prev = init_opp.prev;
    // corner data
    if (init_he.corner) {
        new_he.corner = expect(init_he.corner, "error in expect function");
    }
    new_opp.corner = false;
    
    // knot interval data
    new_he.knot = (1-barycentric_interval) * expect(init_he.knot, "knot interval should be well-defined");
    if (init_opp.knot) {
        new_opp.knot = (1-barycentric_interval) * expect(init_opp.knot, "error in expect function (2)");
    }
    new_he.control_mesh_half_edge = init_he.control_mesh_half_edge;
    if (!new_he.control_mesh_half_edge) {
        ++count_virtual_half_edges;
    }
    new_opp.control_mesh_half_edge = init_opp.control_mesh_half_edge;
    if (!new_opp.control_mesh_half_edge) {
        ++count_virtual_half_edges;
    }

    // adjust half-edge data for the existing edges
    auto& next_he = get_e(get_next(handle));
    next_he.prev = new_heh;
    auto& opp_prev_he = get_e(get_prev(init_twin_h));
    opp_prev_he.next = get_twin(new_heh);

    // modify outgoing if necessary on existing vertex
    if (get_out(to_vh)) {
        if (get_out(to_vh).unwrap() == init_twin_h) {
            auto& v = get_v(get_target(handle));
            v.outgoing = optional_half_edge_handle(get_twin(new_heh));
        }
    }

    // finally, modify the original half-edges
    init_he.next = new_heh;
    init_he.target = split_vh;
    init_he.corner = false;
    init_he.knot = barycentric_interval * expect(init_he.knot, "knot interval should be well-defined (2)");

    init_opp.prev = get_twin(new_heh);
    if (init_opp.knot) {
        init_opp.knot = barycentric_interval * expect(init_opp.knot, "error in expect function (3)");
    }

    // return the half edge whose target points to the created vertex
    // (on the same face as the original half edge)
    return handle;
}

void tmesh::split_t_face_at_half_edges(half_edge_handle h1, half_edge_handle h2, bool split_to_control_mesh, double interval_length)
{
    //   -------------->  ------------->             ----------------->   -------------->
    //  -----------------x---------------           --------------------x-----------------
    //   <--------------  <-------------             <----------------- | <--------------
    //                                                                ^ | |
    //                                                                | | |
    //                                                                | | |
    //                                         To                     | | |
    //                                                                | | |
    //                                                                | | |
    //                                                                | | v
    //   -------------->  ------------->              ----------------> | -------------->
    //  -----------------x---------------            -------------------x-----------------
    //   <--------------  <-------------              <----------------   <--------------
    //

    // ensure proper input
    if(!expect(corner(h1), "handle 1 should have corners defined")) 
        panic("Handle 1 should point to a T-junction");
    if(!expect(corner(h2), "handle 2 should have corners defined"))
        panic("Handle 2 should point to a T-junction");

    // extract the T-junctions
    auto init_t_vh = get_target(h1);
    auto end_t_vh = get_target(h2);

    // CREATE NEW HALFEDGE DATA
    auto new_heh = find_or_create_edge_between(init_t_vh, end_t_vh);
    auto new_opp_h = get_twin(new_heh);

    auto& new_he = get_e(new_heh);
    auto& new_opp = get_e(new_opp_h);

    // add virtual edges if input is not to control mesh
    if(!split_to_control_mesh) {
        count_virtual_half_edges += 2;
        new_he.control_mesh_half_edge = false;
        new_opp.control_mesh_half_edge = false;
    }
    else {
        new_he.control_mesh_half_edge = true;
        new_opp.control_mesh_half_edge = true;
    }

    // adjust the halfedges of the split faces to be corners
    new_he.corner = true;
    new_opp.corner = true;

    // create/adjust face data for the newly-created half edges
    face f(new_heh);
    face_handle new_face_h = faces.push(f);
    new_he.face = optional_face_handle(new_face_h);

    face_handle curr_face_h = get_face_of_half_edge(h2).expect("half-edge handle 2 should have a well-defined face");
    auto& curr_face = get_f(curr_face_h);
    curr_face.edge = new_opp_h;
    new_opp.face = optional_face_handle(curr_face_h);

    // insert knot information to the new edges
    new_he.knot = interval_length;
    new_opp.knot = interval_length;

    // adjust adjacency information for the half-edges
    auto& new_he_next = get_e(get_next(h2));
    auto& new_he_prev = get_e(h1);
    auto& new_opp_next = get_e(get_next(h1));
    auto& new_opp_prev = get_e(h2);

    new_he.next = get_next(h2);
    new_he.prev = h1;

    new_opp.next = get_next(h1);
    new_opp.prev = h2;

    new_he_next.prev = new_heh;
    new_he_prev.next = new_heh;

    new_opp_next.prev = new_opp_h;
    new_opp_prev.next = new_opp_h;

    // adjust all face handles for half edges in the new face
    auto iter = new_heh;
    do {
        auto& iter_he = get_e(iter);
        iter_he.face = new_he.face;
        iter = get_next(iter);
    } while(iter != new_heh);
}

}
