#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tsl/evaluation/surface_evaluator.hpp"
#include "tsl_tests/evaluation/surface_evaluator_fixtures.hpp"
#include "tsl/io/obj.hpp"
#include "tsl/geometry/tmesh/tmesh.hpp"

using namespace tsl;

namespace tsl_tests {

TEST_F(SurfaceEvaluatorWithSingleFaceTest,KnotVectors) {
    auto knot_vector_map = evaluator.get_knot_vectors();
    // ensure that there are four knot vectors -- one for each of the vertices
    EXPECT_EQ(4, knot_vector_map.num_values());

    // iterate on each vertex handle; there should be one handle with a
    // well-defined basis function; the other (potential) handle is skipped
    int num_empty;
    for (const auto& vh: mesh.get_vertices()) {
        const auto local_knots = knot_vector_map[vh];
	num_empty = 0;
	EXPECT_EQ(2, local_knots.size());
        
	// check the local knot vectors to be all the same
	for (int i = 0; i < local_knots.size(); ++i) {
	    if (local_knots[i].u.size() == 0 && local_knots[i].v.size() == 0) {
		++num_empty;
	    }
	    else
	    {
                EXPECT_NE(0, local_knots[i].u.size());
		EXPECT_NE(0, local_knots[i].v.size());

		EXPECT_EQ(0, local_knots[i].u[0]);
		EXPECT_EQ(0, local_knots[i].u[1]);
		EXPECT_EQ(0, local_knots[i].u[2]);
		EXPECT_EQ(1, local_knots[i].u[3]);
		EXPECT_EQ(1, local_knots[i].u[4]);
		
		EXPECT_EQ(0, local_knots[i].v[0]);
		EXPECT_EQ(0, local_knots[i].v[1]);
		EXPECT_EQ(0, local_knots[i].v[2]);
		EXPECT_EQ(1, local_knots[i].v[3]);
		EXPECT_EQ(1, local_knots[i].v[4]);
	    }
	}
	EXPECT_EQ(1, num_empty);
    }
}

TEST_F(SurfaceEvaluatorMinimalInterpolantPatchTest, KnotVectors) {
    // ensure that the knot vectors on the vertices are correct

    auto knot_vector_map = evaluator.get_knot_vectors();
//    auto handles = evaluator.ensure// ensure that there are four knot vectors -- one for each of the vertices
    EXPECT_EQ(16, knot_vector_map.num_values());

    // iterate on each vertex handle; there should be one handle with a
    // well-defined basis function; the other (potential) handle is skipped
    int num_empty;
    std::vector<double> ref_u, ref_v;
    int v_counter = 0;
    for (const auto& vh: mesh.get_vertices()) {
	const auto local_knots = knot_vector_map[vh];
	num_empty = 0;
	EXPECT_EQ(mesh.get_valence(vh), local_knots.size());

	// check the local knot vectors
	int counter = 0;
	for (const auto& heh: mesh.get_half_edges_of_vertex(vh, edge_direction::outgoing)) {
	    if (mesh.is_border(heh)) {
		EXPECT_EQ(0, local_knots[counter].u.size());
		EXPECT_EQ(0, local_knots[counter].v.size());
		++counter;
		continue;
	    }
	    else if (mesh.get_valence(vh) == 2 && mesh.get_valence(mesh.get_target(heh)) == 3) {
		ref_u = {0,0,0,0,1};
		ref_v = {0,0,0,0,1};
	    }
	    else if (mesh.get_valence(vh) == 3 && mesh.get_valence(mesh.get_target(heh)) == 2) {
	        ref_u = {-1,-1,0,0,0};
		ref_v = {0,0,0,0,1};
	    }
	    else if (mesh.get_valence(vh) == 3 && mesh.get_valence(mesh.get_target(heh)) == 3) {
		ref_u = {0,0,0,1,1};
		ref_v = {0,0,0,0,1};
	    }
	    else if (mesh.get_valence(vh) == 3 && mesh.get_valence(mesh.get_target(heh)) == 4) {
	        ref_u = {0,0,0,0,1};
		if (mesh.get_valence(mesh.get_target(mesh.get_prev(mesh.get_prev(heh)))) == 2) {
		    ref_v = {-1,-1,0,0,0};
		} else {
		    ref_v = {0,0,0,1,1};
		}
	    }
	    else if (mesh.get_valence(vh) == 4 && mesh.get_valence(mesh.get_target(heh)) == 3) {
	        ref_u = {-1,-1,0,0,0};
		if (mesh.get_valence(mesh.get_target(mesh.get_next(heh))) == 2) {
		    ref_v = {-1,-1,0,0,0};
		} else {
		    ref_v = {0,0,0,1,1,1};
		}
	    }
	    else if (mesh.get_valence(vh) == 4 && mesh.get_valence(mesh.get_target(heh)) == 4) {
                ref_u = {0,0,0,1,1};
		if (mesh.get_valence(mesh.get_target(mesh.get_next(heh))) == 3) {
		    ref_v = {-1,-1,0,0,0};
		} else {
		    ref_v = {0,0,0,1,1};
		}
	    }

	    for(int i = 0; i < 5; ++i) {
	        EXPECT_EQ(ref_u[i], local_knots[counter].u[i]);
	        EXPECT_EQ(ref_v[i], local_knots[counter].v[i]);
	    }

//	    for(int i = 0; i < 5; ++i) {
//
//		 std::cout << v_counter << " " << counter<< " " << i << " " << local_knots[counter].u[i] << std::endl;
//		 std::cout << v_counter << " " << counter<< " " << i << " " << local_knots[counter].v[i] << std::endl;
//	    }
	    ++counter;
	}
        ++v_counter;
    }
}

TEST_F(SurfaceEvaluatorMinimalInterpolantPatchTest, ParametricDomains) {
    // get the parametric domains for each basis function handle
    auto handles = evaluator.get_basis_function_handles();
    int count = 0;
    for (const auto& vh: handles) {
        size_t handle_idx = 0;
	for (const auto& [h, q]: handles[vh]) {
	    if (q == tag::border) {
		EXPECT_TRUE(mesh.is_border(h));
	        EXPECT_THROW(evaluator.get_parametric_domain_access(vh, handle_idx), panic_exception);
	    }
	    else {
		EXPECT_FALSE(mesh.is_border(h));
		auto r = evaluator.get_parametric_domain_access(vh, handle_idx);
//	        std::cout << "vert " << count << ", index " << handle_idx << "--> " << r.top_right.x << ", " << r.top_right.y << std::endl;
	    }
	    ++handle_idx;
	}
	++count;
    }
}

TEST_F(SurfaceEvaluatorMinimalInterpolantPatchTest, DISABLED_Supports) {
    auto support = evaluator.get_support_map();
    // supports of basis function on different faces
    for (const auto& fh: mesh.get_faces()) {
        EXPECT_EQ(16, support[fh].size());
    }
}

}
