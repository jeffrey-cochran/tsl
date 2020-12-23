#include "tsl_tests/geometry/tmesh/tmesh_fixtures.hpp"

namespace tsl_tests {

void TmeshTestWithCubeData::SetUp() {
    // =============================================
    // FRONT SIDE
    // =============================================
    //
    //       (v4) ====== (v1) ====== (v0)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f1    ||   f0     ||
    //        ||          ||          ||
    //       (v5) ====== (v2) ====== (v3)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f3    ||   f2     ||
    //        ||          ||          ||
    //       (v8) ====== (v7) ====== (v6)

    // f0
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v0, v1, v2, v3}));
    vertex_handles.insert(vertex_handles.end(), {v0, v1, v2, v3});

    // f1
    auto v4 = mesh.add_vertex({0, 0, 0}, true);
    auto v5 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v1, v4, v5, v2}));
    vertex_handles.insert(vertex_handles.end(), {v4, v5});

    // f2
    auto v6 = mesh.add_vertex({0, 0, 0}, true);
    auto v7 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v3, v2, v7, v6}));
    vertex_handles.insert(vertex_handles.end(), {v6, v7});

    // f3
    auto v8 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v2, v5, v8, v7}));
    vertex_handles.insert(vertex_handles.end(), {v8});

    // =============================================
    // LEFT SIDE
    // =============================================
    //
    //       (v11) ===== (v9) ====== (v4)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f5    ||   f4     ||
    //        ||          ||          ||
    //       (v12) ===== (v10) ===== (v5)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f7    ||   f6     ||
    //        ||          ||          ||
    //       (v14) ===== (v13) ===== (v8)

    // f4
    auto v9 = mesh.add_vertex({0, 0, 0}, true);
    auto v10 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v4, v9, v10, v5}));
    vertex_handles.insert(vertex_handles.end(), {v9, v10});

    // f5
    auto v11 = mesh.add_vertex({0, 0, 0}, true);
    auto v12 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v9, v11, v12, v10}));
    vertex_handles.insert(vertex_handles.end(), {v11, v12});

    // f6
    auto v13 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v5, v10, v13, v8}));
    vertex_handles.insert(vertex_handles.end(), {v13});

    // f7
    auto v14 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v10, v12, v14, v13}));
    vertex_handles.insert(vertex_handles.end(), {v14});

    // =============================================
    // BACK SIDE
    // =============================================
    //
    //       (v17) ===== (v15) ===== (v11)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f9    ||   f8     ||
    //        ||          ||          ||
    //       (v18) ===== (v16) ===== (v12)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f11   ||   f10    ||
    //        ||          ||          ||
    //       (v20) ===== (v19) ===== (v14)

    // f8
    auto v15 = mesh.add_vertex({0, 0, 0}, true);
    auto v16 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v11, v15, v16, v12}));
    vertex_handles.insert(vertex_handles.end(), {v15, v16});

    // f9
    auto v17 = mesh.add_vertex({0, 0, 0}, true);
    auto v18 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v15, v17, v18, v16}));
    vertex_handles.insert(vertex_handles.end(), {v17, v18});

    // f10
    auto v19 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v12, v16, v19, v14}));
    vertex_handles.insert(vertex_handles.end(), {v19});

    // f11
    auto v20 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v16, v18, v20, v19}));
    vertex_handles.insert(vertex_handles.end(), {v20});

    // =============================================
    // RIGHT SIDE
    // =============================================
    //
    //       (v0) ====== (v21) ===== (v17)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f13   ||   f12    ||
    //        ||          ||          ||
    //       (v3) ====== (v22) ===== (v18)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f15   ||   f14    ||
    //        ||          ||          ||
    //       (v6) ====== (v23) ===== (v20)

    // f12
    auto v21 = mesh.add_vertex({0, 0, 0}, true);
    auto v22 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v17, v21, v22, v18}));
    vertex_handles.insert(vertex_handles.end(), {v21, v22});

    // f13
    face_handles.push_back(mesh.add_face({v21, v0, v3, v22}));

    // f14
    auto v23 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v18, v22, v23, v20}));
    vertex_handles.insert(vertex_handles.end(), {v23});

    // f15
    face_handles.push_back(mesh.add_face({v22, v3, v6, v23}));

    // =============================================
    // TOP SIDE
    // =============================================
    //
    //                   BACK
    //
    //       (v11) ===== (v15) ===== (v17)
    //        ||          ||          ||
    //        ||          ||          ||
    //     L  ||    f17   ||   f16    ||   R
    //     E  ||          ||          ||   I
    //     F (v9) ====== (v24) ===== (v21) G
    //     T  ||          ||          ||   H
    //        ||          ||          ||   T
    //        ||    f19   ||   f18    ||
    //        ||          ||          ||
    //       (v4) ====== (v1) ====== (v0)
    //
    //                   FRONT

    // f16
    auto v24 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v17, v15, v24, v21}));
    vertex_handles.insert(vertex_handles.end(), {v24});

    // f17
    face_handles.push_back(mesh.add_face({v15, v11, v9, v24}));

    // f18
    face_handles.push_back(mesh.add_face({v21, v24, v1, v0}));

    // f19
    face_handles.push_back(mesh.add_face({v24, v9, v4, v1}));

    // =============================================
    // BOTTOM SIDE
    // =============================================
    //
    //                   FRONT
    //
    //       (v8) ====== (v7) ====== (v6)
    //        ||          ||          ||
    //        ||          ||          ||
    //     L  ||    f21   ||   f20    ||   R
    //     E  ||          ||          ||   I
    //     F (v13) ===== (v25) ===== (v23) G
    //     T  ||          ||          ||   H
    //        ||          ||          ||   T
    //        ||    f23   ||   f22    ||
    //        ||          ||          ||
    //       (v14) ===== (v19) ===== (v20)
    //
    //                   BACK

    // f20
    auto v25 = mesh.add_vertex({0, 0, 0}, true);
    face_handles.push_back(mesh.add_face({v6, v7, v25, v23}));
    vertex_handles.insert(vertex_handles.end(), {v25});

    // f21
    face_handles.push_back(mesh.add_face({v7, v8, v13, v25}));

    // f22
    face_handles.push_back(mesh.add_face({v23, v25, v19, v20}));

    // f23
    face_handles.push_back(mesh.add_face({v25, v13, v14, v19}));
}

void TmeshTestWithTfaceTest::SetUp() {
    // Create tmesh with t face:
    //         v0 ==== v1 ==== v2
    //         ||      ||      ||
    //         ||  f0  ||  f1  ||
    //         ||      ||      ||
    //         v3 ==== v4 ==== v5
    //         ||              ||
    //         ||      f2      ||
    //         ||              ||
    //         v6 ============ v7

    auto& hem = mesh;
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));
    vertex_handles.push_back(hem.add_vertex(vec3(0, 0, 0), true));

    face_handles.push_back(hem.add_face({vertex_handles[1], vertex_handles[0], vertex_handles[3], vertex_handles[4]}));
    face_handles.push_back(hem.add_face({vertex_handles[2], vertex_handles[1], vertex_handles[4], vertex_handles[5]}));
    face_handles.push_back(hem.add_face({
                                        new_face_vertex(vertex_handles[5]),
                                        new_face_vertex(vertex_handles[4], false, 1.0),
                                        new_face_vertex(vertex_handles[3]),
                                        new_face_vertex(vertex_handles[6]),
                                        new_face_vertex(vertex_handles[7])
                                        }));
}

void TmeshTestAsGrid::SetUp() {
    // Create tmesh:
    //         v0 ===== v1 ===== v2 ===== v3 ===== v4 ===== v5 ===== v6 ===== v7 ===== v8
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f0   ||  f1   ||  f2   ||  f3   ||  f4   ||  f5   ||  f6   ||  f7   ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v9 ===== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f8   ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v18 ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f16  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v27 ==== v?? ==== v?? ==== v30 ==== v31 ==== v32 ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f24  ||  f??  ||  f??  ||  f27  ||  f28  ||  f29  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v36 ==== v?? ==== v?? ==== v39 ==== v40 ==== v41 ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f32  ||  f??  ||  f??  ||  f35  ||  f36  ||  f??  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v45 ==== v?? ==== v?? ==== v48 ==== v49 ==== v50 ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f40  ||  f??  ||  f??  ||  f43  ||  f44  ||  f??  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v54 ==== v?? ==== v?? ==== v57 ==== v58 ==== v59 ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f48  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v63 ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f56  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v72 ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v??
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         ||  f64  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f??  ||  f71  ||
    //         ||       ||       ||       ||       ||       ||       ||       ||       ||
    //         v81 ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v?? ==== v89

    // Add 30 vertices
    for (uint32_t i = 0; i <= 89; ++i) {
        vertex_handles.push_back(mesh.add_vertex(vec3(0, 0, 0), true));
    }

    uint32_t width = 9;
    uint32_t height = 10;
    for (uint32_t j = 0; j < (width * (height - 1)); j += width) {
        for (uint32_t i = j; i < (j + width - 1); ++i) {
            face_handles.push_back(mesh.add_face({vertex_handles[i], vertex_handles[i+width], vertex_handles[i+width+1], vertex_handles[i+1]}));
        }
    }
}

void TmeshTestSplittingOperations::SetUp() {
    // Create the following structure
    // v4---------v5
    // |    2      |
    // |         1 |
    // | 1         |
    // |    2      |
    // v3---------v2
    // |    1      |
    // |         1 |
    // | 1         |
    // |    1      |
    // v0---------v1
    // where numbers are the knot intervals on the faces
    vertex_handles.push_back(mesh.add_vertex(vec3(0,0,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,0,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,3,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(0,3,0), true));

    vertex_handles.push_back(mesh.add_vertex(vec3(0,9,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,9,0), true));

    face_handles.push_back(mesh.add_face({vertex_handles[0], vertex_handles[1], vertex_handles[2], vertex_handles[3]}));
    face_handles.push_back(mesh.add_face({vertex_handles[3], vertex_handles[2], vertex_handles[5], vertex_handles[4]}));

    // find the halfedge handle from vertex 3 to vertex 2
    auto heh = mesh.get_half_edge_between(vertex_handles[3],vertex_handles[2]).unwrap();
    mesh.set_knot_interval(heh, 2);
    mesh.set_knot_interval(mesh.get_next(mesh.get_next(heh)), 2);
}

void TmeshTestBezierMeshExtraction::SetUp() {
    // Create the following structure
    // v14-----------v15-----------------------v16
    //  |             |                         |
    //  |     f7      |            f8           |
    //  |             |                         |
    //  |             |                         |
    // v11-----------v12-----------------------v13
    //  |             |                         |
    //  |     f5      |            f6           |
    //  |             |                         |
    //  |             |                         |
    // v8------------v9------------------------v10
    //  |             |                         |
    //  |     f3      |            f4           |
    //  |             |                         |
    //  |             |                         |
    // v4------------v5------------v6----------v7
    //  |             |             |           |
    //  |     f0      |      f1     |    f2     |
    //  |             |             |           |
    //  |             |             |           |
    // v0------------v1------------v2----------v3
    
    vertex_handles.push_back(mesh.add_vertex(vec3(0,0,0), true)); // v0
    vertex_handles.push_back(mesh.add_vertex(vec3(1,0,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(2,0,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,0,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(0,1,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(1,1,0), true)); // v5
    vertex_handles.push_back(mesh.add_vertex(vec3(2,1,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,1,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(0,2,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(1,2,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,2,0), true)); // v10
    vertex_handles.push_back(mesh.add_vertex(vec3(0,3,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(1,3,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(3,3,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(0,4,0), true));
    vertex_handles.push_back(mesh.add_vertex(vec3(1,4,0), true)); // v15
    vertex_handles.push_back(mesh.add_vertex(vec3(3,4,0), true));

    // face handle definition
    face_handles.push_back(mesh.add_face({vertex_handles[0], vertex_handles[1], vertex_handles[5], vertex_handles[4]}));
    face_handles.push_back(mesh.add_face({vertex_handles[1], vertex_handles[2], vertex_handles[6], vertex_handles[5]}));
    face_handles.push_back(mesh.add_face({vertex_handles[2], vertex_handles[3], vertex_handles[7], vertex_handles[6]}));
    face_handles.push_back(mesh.add_face({vertex_handles[4], vertex_handles[5], vertex_handles[9], vertex_handles[8]}));
    face_handles.push_back(mesh.add_face({
                                        new_face_vertex(vertex_handles[5]),
                                        new_face_vertex(vertex_handles[6], false, 1.0),
                                        new_face_vertex(vertex_handles[7]),
                                        new_face_vertex(vertex_handles[10]),
                                        new_face_vertex(vertex_handles[9], true, 2.0)
                                        }));
    face_handles.push_back(mesh.add_face({vertex_handles[8], vertex_handles[9], vertex_handles[12], vertex_handles[11]}));
    face_handles.push_back(mesh.add_face({
                                        new_face_vertex(vertex_handles[9]),
                                        new_face_vertex(vertex_handles[10], true, 2.0),
                                        new_face_vertex(vertex_handles[13]),
                                        new_face_vertex(vertex_handles[12], true, 2.0)
                                        }));
    face_handles.push_back(mesh.add_face({vertex_handles[11], vertex_handles[12], vertex_handles[15], vertex_handles[14]}));
    face_handles.push_back(mesh.add_face({
                                        new_face_vertex(vertex_handles[12]),
                                        new_face_vertex(vertex_handles[13], true, 2.0),
                                        new_face_vertex(vertex_handles[16]),
                                        new_face_vertex(vertex_handles[15], true, 2.0)
                                        }));

}

}
