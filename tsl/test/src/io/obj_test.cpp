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

}
