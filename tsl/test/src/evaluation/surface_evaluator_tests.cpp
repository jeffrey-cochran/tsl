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
    // ensure that there are four knot vectors -- one for each of the vertices
    EXPECT_EQ(16, knot_vector_map.num_values());

    // iterate on each vertex handle; there should be one handle with a
    // well-defined basis function; the other (potential) handle is skipped
    int num_empty;
    std::vector<double> ref_u, ref_v;
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
	    }
	    else if(mesh.get_valence(vh) == 2 && mesh.get_valence(mesh.get_target(heh)) == 3) {
		ref_u = {0,0,0,0,1};
		ref_v = {0,0,0,0,1};
		for(int i = 0; i < 5; ++i) {
		     EXPECT_EQ(ref_u[i], local_knots[counter].u[i]);
		     EXPECT_EQ(ref_v[i], local_knots[counter].v[i]);
		}
	    }

	    ++counter;
	}
    }
}

}
