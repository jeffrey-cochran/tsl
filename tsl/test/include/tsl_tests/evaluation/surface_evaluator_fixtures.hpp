#ifndef TEST_INCLUDE_TSL_TESTS_EVALUATION_SURFACE_EVALUATOR_FIXTURES_HPP
#define TEST_INCLUDE_TSL_TESTS_EVALUATION_SURFACE_EVALUATOR_FIXTURES_HPP

#include <vector>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tsl/geometry/tmesh/handles.hpp"
#include "tsl/geometry/tmesh/tmesh.hpp"
#include "tsl/io/obj.hpp"
#include "tsl/evaluation/surface_evaluator.hpp"

using std::vector;
using std::string;

using namespace tsl;

namespace tsl_tests {

class SurfaceEvaluatorWithTfaceTest : public ::testing::Test {
public:
    SurfaceEvaluatorWithTfaceTest() : evaluator(tmesh()), vertex_handles(), face_handles() {}
protected:
    void SetUp() override;

    surface_evaluator evaluator;
    vector<vertex_handle> vertex_handles;
    vector<face_handle> face_handles;
};

class SurfaceEvaluatorWithSingleFaceTest : public ::testing::Test {
public:
    SurfaceEvaluatorWithSingleFaceTest() : evaluator(tmesh()) {}
protected:
    void SetUp() override;

    tmesh mesh;
    surface_evaluator evaluator;

};

}

#endif //TEST_INCLUDE_TSL_TESTS_EVALUATION_SURFACE_EVALUATOR_FIXTURES_HPP
