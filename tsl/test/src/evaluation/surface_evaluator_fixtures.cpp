#include "tsl/evaluation/surface_evaluator.hpp"
#include "tsl_tests/evaluation/surface_evaluator_fixtures.hpp"

using namespace tsl;

namespace tsl_tests {

void SurfaceEvaluatorWithTfaceTest::SetUp() {}

void SurfaceEvaluatorWithSingleFaceTest::SetUp() {
    string path = "meshes/";
    string fpath = path;
    fpath += "1x1_square.obj";

    evaluator = surface_evaluator(read_obj_into_tmesh(fpath));
    
    mesh = evaluator.get_tmesh();
}

}
