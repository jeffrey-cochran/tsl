#include "tsl_tests/io/obj_load.hpp"

using namespace tsl;

namespace tsl_tests {

void TmeshLoadSingleFace::SetUp() {
    string path = "meshes/";
    string fpath = path;
    fpath += "1x1_square.obj";
    mesh = read_obj_into_tmesh(fpath);
}

}
