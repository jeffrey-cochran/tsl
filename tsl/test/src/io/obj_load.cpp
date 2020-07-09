#include "tsl_tests/io/obj_load.hpp"

using namespace tsl;

namespace tsl_tests {

void TmeshLoadSingleFace::SetUp() {
    string path = "test_meshes/";
    string fpath = path;
    fpath += "1x1_square.obj";
    mesh = read_obj_into_tmesh_no_boundary_interpolant(fpath);
}

void TmeshLoad3x3InterpolatoryPatch::SetUp() {
    string path = "test_meshes/";
    string fpath = path;
    fpath += "3x3_square.obj";
    mesh = read_obj_into_boundary_interpolant_tmesh(fpath);
}

}
