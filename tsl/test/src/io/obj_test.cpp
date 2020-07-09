#include <optional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tsl/geometry/tmesh/tmesh.hpp"
#include "tsl_tests/io/obj_load.hpp"

using namespace tsl;

namespace tsl_tests {

TEST_F(TmeshLoadSingleFace,Counts) {
    EXPECT_EQ(4, mesh.num_vertices());
    EXPECT_EQ(1, mesh.num_faces());
}

TEST_F(TmeshLoadSingleFace, Border) {
    // check to make sure that circulators work on the face
    int face_half_edges = 0;
    for (const auto& fh: mesh.get_faces()) {
	for (const auto& heh: mesh.get_half_edges_of_face(fh)) {
	    ++face_half_edges;
	    EXPECT_TRUE(mesh.get_face_of_half_edge(heh));
	    // ensure that the objects have corners
	    EXPECT_TRUE(mesh.corner(heh));
	    // ensure that each half-edge points to a corner
	    EXPECT_TRUE(mesh.corner(heh).value_or(false));
	    // ensure that knot intervals are well-defined
	    EXPECT_TRUE(mesh.get_knot_interval(heh));
	    EXPECT_EQ(1, mesh.get_knot_interval(heh).value_or(2));

            auto twin = mesh.get_twin(heh);
	    // option for face should be false (no face)
	    EXPECT_FALSE(mesh.get_face_of_half_edge(twin));
	    // option for corner should be false
	    EXPECT_FALSE(mesh.corner(twin));
	    // option for knot interval should be false
	    EXPECT_FALSE(mesh.get_knot_interval(twin));
	}
    }
    EXPECT_EQ(4, face_half_edges);

}

TEST_F(TmeshLoad3x3InterpolatoryPatch, Count) {
    // check to make sure the correct number of faces and vertices
    EXPECT_EQ(9, mesh.num_faces());
    EXPECT_EQ(16, mesh.num_vertices());
    EXPECT_EQ(24, mesh.num_edges());
    EXPECT_EQ(48, mesh.num_half_edges());

}

TEST_F(TmeshLoad3x3InterpolatoryPatch, KnotCounts) {
    // Ensure the correct number of each knotted half-edge
    int no_knot = 0;
    int zero_knot = 0;
    int one_knot = 0;

    int tmp;
    for (const auto& heh: mesh.get_half_edges()) {
        if (!mesh.get_knot_interval(heh))
	    ++no_knot;
	else {
	    tmp = mesh.get_knot_interval(heh).value_or(-1);
            if (tmp == 0)
		++zero_knot;
	    else if ( tmp == 1)
		++one_knot;
	}
    }

    EXPECT_EQ(12, no_knot);
    EXPECT_EQ(24, zero_knot);
    EXPECT_EQ(12, one_knot);
}

TEST_F(TmeshLoad3x3InterpolatoryPatch, Knots) {
    int valence;
    // iterate over vertices and check the knot information of their half-edges
    for (const auto& vh: mesh.get_vertices()) {
        valence = mesh.get_valence(vh);
	for (const auto& heh: mesh.get_half_edges_of_vertex(vh, edge_direction::outgoing)) {
	    if (valence == 2) {
	        if (mesh.get_face_of_half_edge(heh)) {
	            EXPECT_EQ(0.0, mesh.get_knot_interval(heh).value_or(-1));
	        }
	    } else if (valence == 3) {
	        if (mesh.is_border(heh) || mesh.is_border(mesh.get_twin(heh)) ) {
		    if (!mesh.is_border(heh)) {
			if (mesh.get_valence(mesh.get_target(heh)) == 2) {
			    EXPECT_EQ(0.0, mesh.get_knot_interval(heh).value_or(-6));
			} else {
			    EXPECT_EQ(1.0, mesh.get_knot_interval(heh).value_or(-2));
		        }
		    }
		} else {
		    EXPECT_EQ(0.0, mesh.get_knot_interval(heh).value_or(-3));
		}
	    } else {
		if (mesh.get_valence(mesh.get_target(heh)) < 4) {
		    EXPECT_EQ(0.0, mesh.get_knot_interval(heh).value_or(-4));
		} else {
		    EXPECT_EQ(1.0, mesh.get_knot_interval(heh).value_or(-5));
		}
	    }
	}
    }
}

TEST_F(TmeshLoad3x3InterpolatoryPatch, ScaleFactors) {
    for (const auto& heh: mesh.get_half_edges()) {
	// check if the knot factor is well-defined (no border halfedges)
	if (mesh.get_knot_factor(heh)) {
            EXPECT_EQ(1.0, mesh.get_knot_factor(heh).value_or(-1));
	}
    }
}

}
