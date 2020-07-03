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

}
