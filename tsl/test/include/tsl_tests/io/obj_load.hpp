#ifndef TEST_INCLUDE_TSL_TESTS_IO_OBJ_LOAD_HPP
#define TEST_INCLUDE_TSL_TESTS_IO_OBJ_LOAD_HPP

#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tsl/geometry/tmesh/tmesh.hpp"
#include "tsl/io/obj.hpp"

using std::string;
using namespace tsl;

namespace tsl_tests {

class TmeshLoadSingleFace : public ::testing::Test {
protected:
    void SetUp() override;

    tmesh mesh;
};

}


#endif //TEST_INCLUDE_TSL_TESTS_IO_OBJ_LOAD_HPP
