#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <fmt/core.h>

#include "tsl/geometry/tmesh/tmesh.hpp"
#include "tsl_tests/geometry/tmesh/tmesh_fixtures.hpp"

using namespace tsl;
using fmt::print;

namespace tsl_tests {

class TmeshTest : public ::testing::Test {
protected:
    tmesh mesh;
};

TEST_F(TmeshTest, IsFaceInsertionValid) {
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);
    auto v4 = mesh.add_vertex({0, 0, 0}, true);
    auto v5 = mesh.add_vertex({0, 0, 0}, true);

    // Test number of vertices
    EXPECT_FALSE(mesh.is_face_insertion_valid(vector<new_face_vertex>()));
    EXPECT_FALSE(mesh.is_face_insertion_valid({
        new_face_vertex(v1),
        new_face_vertex(v2),
        new_face_vertex(v3)
    }));
    EXPECT_TRUE(mesh.is_face_insertion_valid({v1, v2, v3, v4}));

    // Test corner combinations
    EXPECT_FALSE(mesh.is_face_insertion_valid({
        new_face_vertex(v1, false, 1.0),
        new_face_vertex(v2, true, 1.0),
        new_face_vertex(v3, true, 1.0),
        new_face_vertex(v4, true, 1.0)
    }));
    EXPECT_TRUE(mesh.is_face_insertion_valid({
        new_face_vertex(v1, true, 1.0),
        new_face_vertex(v2, false, 1.0),
        new_face_vertex(v3, true, 1.0),
        new_face_vertex(v4, true, 1.0),
        new_face_vertex(v5, true, 1.0)
    }));

    // Test knot values
    EXPECT_FALSE(mesh.is_face_insertion_valid({
        new_face_vertex(v1, true, 0.0),
        new_face_vertex(v2, false, 1.0),
        new_face_vertex(v3, true, 1.0),
        new_face_vertex(v4, true, 1.0),
        new_face_vertex(v5, true, 1.0)
    }));
}

TEST_F(TmeshTest, IsFaceInsertionValidSameVertices) {
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);
    auto v4 = mesh.add_vertex({0, 0, 0}, true);

    EXPECT_TRUE(mesh.is_face_insertion_valid({v1, v2, v3, v4}));

    // Add the following face (f1):
    //
    //       (v2) ====== (v1)
    //        ||          ||
    //        ||          ||
    //        ||    f1    ||
    //        ||          ||
    //       (v3) ====== (v4)
    mesh.add_face({v1, v2, v3, v4});

    EXPECT_FALSE(mesh.is_face_insertion_valid({v1, v2, v3, v4}));
    EXPECT_FALSE(mesh.is_face_insertion_valid({v2, v1, v4, v3}));
    EXPECT_FALSE(mesh.is_face_insertion_valid({v1, v1, v3, v4}));
    EXPECT_FALSE(mesh.is_face_insertion_valid({v1, v1, v1, v1}));
}

TEST_F(TmeshTest, IsFaceInsertionValidTmeshConstraints) {
    // TODO: This has to be checked in `tmesh::add_face` when border edges are implemented
    // Cycle condition (check `tsplines::tmesh::check_integrity/check_regularity_around_vertex` in git history)
    // Consistency condition
    // Extraordinary checks:
    // - three rings regular around extraordinary vertex
    // - same knot intervall on all half edges
    // - valence == 4 within rings
    // - simply connected in rings
}

TEST_F(TmeshTest, AddFace) {
    auto v1 = new_face_vertex(mesh.add_vertex({0, 0, 0}, true), false, 1.0);
    auto v2 = new_face_vertex(mesh.add_vertex({0, 0, 0}, true), true, 1.0);
    auto v3 = new_face_vertex(mesh.add_vertex({0, 0, 0}, true), true, 1.0);
    auto v4 = new_face_vertex(mesh.add_vertex({0, 0, 0}, true), true, 1.0);
    auto v5 = new_face_vertex(mesh.add_vertex({0, 0, 0}, true), true, 1.0);

    EXPECT_TRUE(mesh.is_face_insertion_valid({v1, v2, v3, v4, v5}));

    // Add the following face (f1):
    //
    //       (v2) ====(v1)===== (v5)
    //        ||                 ||
    //        ||                 ||
    //        ||       f1        ||
    //        ||                 ||
    //       (v3) ============= (v4)
    EXPECT_EQ(face_handle(0), mesh.add_face({v1, v2, v3, v4, v5}));
}

TEST_F(TmeshTest, GetVertexPosition) {
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({1, 2, 3}, true);

    {
        const auto& mesh_const = mesh;
        auto v1_pos = mesh_const.get_vertex_position(v1);
        EXPECT_EQ(vec3(0, 0, 0), v1_pos);
    }

    auto& v2_pos = mesh.get_vertex_position(v2);
    EXPECT_EQ(vec3(1, 2, 3), v2_pos);

    v2_pos.x = 42;
    EXPECT_EQ(vec3(42, 2, 3), v2_pos);
}

TEST_F(TmeshTest, GetVerticesOfFace) {
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);

    auto v4 = mesh.add_vertex({0, 0, 0}, true);
    auto v5 = mesh.add_vertex({0, 0, 0}, true);

    vector<face_handle> face_handles;

    // Add the following face (f0):
    //
    //       (v1) ====== (v0)
    //        ||          ||
    //        ||          ||
    //        ||    f0    ||
    //        ||          ||
    //       (v2) ====== (v3)
    face_handles.push_back(mesh.add_face({v0, v1, v2, v3}));

    ASSERT_THAT(mesh.get_vertices_of_face(face_handles[0]), ::testing::UnorderedElementsAre(v0, v1, v2, v3));

    // Add the following face (f1):
    //
    //       (v4) ====== (v1) ====== (v0)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f1    ||    f0    ||
    //        ||          ||          ||
    //       (v5) ====== (v2) ====== (v3)
    face_handles.push_back(mesh.add_face({v1, v4, v5, v2}));

    ASSERT_THAT(mesh.get_vertices_of_face(face_handles[0]), ::testing::UnorderedElementsAre(v0, v1, v2, v3));
    ASSERT_THAT(mesh.get_vertices_of_face(face_handles[1]), ::testing::UnorderedElementsAre(v1, v2, v4, v5));
}

// TODO: Implement orientability check in `tmesh::add_face()`
TEST_F(TmeshTest, Orientability) {
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);

//    auto v4 = mesh.add_vertex({0, 0, 0});
//    auto v5 = mesh.add_vertex({0, 0, 0});

    mesh.add_face({v0, v1, v2, v3});

    // This should panic, because it will break the orientability of the mesh
//    ASSERT_THROW(mesh.add_face({v1, v2, v4, v5}), panic_exception);
}

TEST_F(TmeshTest, VertexIterator) {
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);

    vector<vertex_handle> expected_handles;
    expected_handles.insert(expected_handles.begin(), {v0, v1, v2, v3});
    vector<vertex_handle> vertex_handles;
    for (const auto& vh: mesh.get_vertices()) {
        vertex_handles.push_back(vh);
    }

    ASSERT_THAT(vertex_handles, ::testing::UnorderedElementsAreArray(expected_handles));
}

TEST_F(TmeshTest, FaceIterator) {
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);

    auto v4 = mesh.add_vertex({0, 0, 0}, true);
    auto v5 = mesh.add_vertex({0, 0, 0}, true);
    auto v6 = mesh.add_vertex({0, 0, 0}, true);
    auto v7 = mesh.add_vertex({0, 0, 0}, true);

    // Create the following mesh:
    //
    //       (v4) ====== (v1) ====== (v0)
    //        ||          ||          ||
    //        ||          ||          ||
    //        ||    f1    ||    f0    ||
    //        ||          ||          ||
    //       (v5) ====== (v2) ====== (v3)
    //                    ||          ||
    //                    ||          ||
    //                    ||    f2    ||
    //                    ||          ||
    //                   (v6) ====== (v7)
    vector<face_handle> expected_handles;
    expected_handles.push_back(mesh.add_face({v0, v1, v2, v3}));
    expected_handles.push_back(mesh.add_face({v1, v4, v5, v2}));
    expected_handles.push_back(mesh.add_face({v3, v2, v6, v7}));

    EXPECT_EQ(3, mesh.num_faces());

    vector<face_handle> face_handles;
    for (const auto& fh: mesh.get_faces()) {
        face_handles.push_back(fh);
    }

    ASSERT_THAT(face_handles, ::testing::UnorderedElementsAreArray(expected_handles));
}

TEST_F(TmeshTest, HalfEdgeIterator) {
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);

    // Add the following face (f0):
    //
    //       (v1) ====== (v0)
    //        ||          ||
    //        ||          ||
    //        ||    f0    ||
    //        ||          ||
    //       (v2) ====== (v3)
    mesh.add_face({v0, v1, v2, v3});
    vector<vertex_handle> expected_handles;
    expected_handles.insert(expected_handles.begin(), {v0, v0, v1, v1, v2, v2, v3, v3});

    vector<vertex_handle> vertex_handles;
    for (const auto& eh: mesh.get_half_edges()) {
        vertex_handles.push_back(mesh.get_vertices_of_half_edge(eh)[0]);
    }

    ASSERT_THAT(vertex_handles, ::testing::UnorderedElementsAreArray(expected_handles));
}

TEST_F(TmeshTest, EdgeIterator) {
    auto v0 = mesh.add_vertex({0, 0, 0}, true);
    auto v1 = mesh.add_vertex({0, 0, 0}, true);
    auto v2 = mesh.add_vertex({0, 0, 0}, true);
    auto v3 = mesh.add_vertex({0, 0, 0}, true);

    // Add the following face (f0):
    //
    //       (v1) ====== (v0)
    //        ||          ||
    //        ||          ||
    //        ||    f0    ||
    //        ||          ||
    //       (v2) ====== (v3)
    mesh.add_face({v0, v1, v2, v3});
    vector<vertex_handle> expected_handles;
    expected_handles.insert(expected_handles.begin(), {v0, v1, v2, v3});

    vector<vertex_handle> vertex_handles;
    for (const auto& eh: mesh.get_edges()) {
        vertex_handles.push_back(mesh.get_vertices_of_edge(eh)[0]);
    }

    ASSERT_THAT(vertex_handles, ::testing::UnorderedElementsAreArray(expected_handles));
}

TEST_F(TmeshTestWithCubeData, IsFaceInsertionValidNonManifoldTripleEdge) {
    // Create non manifold triple edge
    auto vte0 = mesh.add_vertex({0, 0, 0}, true);
    auto vte1 = mesh.add_vertex({0, 0, 0}, true);
    auto v0 = vertex_handles[0];
    auto v3 = vertex_handles[3];

    ASSERT_THROW(mesh.add_face({v0, v3, vte0, vte1}), panic_exception);
}

TEST_F(TmeshTestWithCubeData, GetVerticesOfFace) {
    auto vertices = mesh.get_vertices_of_face(face_handles[0]);
    auto v0 = vertex_handles[0];
    auto v1 = vertex_handles[1];
    auto v2 = vertex_handles[2];
    auto v3 = vertex_handles[3];
    ASSERT_THAT(vertices, ::testing::UnorderedElementsAre(v0, v1, v2, v3));
}

TEST_F(TmeshTestWithCubeData, NumFacesEdgesVertices) {
    EXPECT_EQ(24, mesh.num_faces());
    EXPECT_EQ(26, mesh.num_vertices());
    EXPECT_EQ(48, mesh.num_edges());
    EXPECT_EQ(96, mesh.num_half_edges());
}

TEST_F(TmeshTestWithCubeData, GetVerticesOfEdgeAndGetEdgeBetween) {
    auto v2 = vertex_handles[2];
    auto v3 = vertex_handles[3];
    auto edge_handle = mesh.get_edge_between(v2, v3);
    auto vertices = mesh.get_vertices_of_edge(edge_handle.unwrap());
    ASSERT_THAT(vertices, ::testing::UnorderedElementsAre(v2, v3));
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgeBetween) {
    auto v2 = vertex_handles[2];
    auto v3 = vertex_handles[3];
    auto edge_handle = mesh.get_half_edge_between(v2, v3);
    auto vertices = mesh.get_vertices_of_half_edge(edge_handle.unwrap());
    ASSERT_THAT(vertices, ::testing::UnorderedElementsAre(v2, v3));
}

TEST_F(TmeshTestWithCubeData, GetFacesOfEdge) {
    auto v2 = vertex_handles[2];
    auto v3 = vertex_handles[3];
    auto f0 = optional_face_handle(face_handles[0]);
    auto f2 = optional_face_handle(face_handles[2]);
    auto edge_handle = mesh.get_edge_between(v2, v3);
    auto faces = mesh.get_faces_of_edge(edge_handle.unwrap());
    ASSERT_THAT(faces, ::testing::UnorderedElementsAre(f0, f2));
}

TEST_F(TmeshTestWithCubeData, GetFacesOfVertex) {
    auto v2 = vertex_handles[2];
    auto f0 = face_handles[0];
    auto f1 = face_handles[1];
    auto f2 = face_handles[2];
    auto f3 = face_handles[3];
    auto faces = mesh.get_faces_of_vertex(v2);
    ASSERT_THAT(faces, ::testing::UnorderedElementsAre(f0, f1, f2, f3));
}

TEST_F(TmeshTestWithCubeData, GetValenceOfVertexThreeEdges) {
    auto v2 = vertex_handles[0];
    EXPECT_EQ(3, mesh.get_valence(v2));
}

TEST_F(TmeshTestWithCubeData, GetValenceOfVertexFourEdges) {
    auto v2 = vertex_handles[2];
    EXPECT_EQ(4, mesh.get_valence(v2));
}

TEST_F(TmeshTestWithCubeData, GetEdgesOfVertexFourEdges) {
    auto v2 = vertex_handles[2];
    auto edges = mesh.get_edges_of_vertex(v2);
    EXPECT_EQ(4, edges.size());

    auto v1 = vertex_handles[1];
    auto v5 = vertex_handles[5];
    auto v7 = vertex_handles[7];
    auto v3 = vertex_handles[3];

    auto edge_21 = mesh.get_edge_between(v2, v1).unwrap();
    auto edge_25 = mesh.get_edge_between(v2, v5).unwrap();
    auto edge_27 = mesh.get_edge_between(v2, v7).unwrap();
    auto edge_23 = mesh.get_edge_between(v2, v3).unwrap();

    ASSERT_THAT(edges, ::testing::UnorderedElementsAre(edge_21, edge_23, edge_25, edge_27));
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgesOfVertexFourEdges) {
    auto v2 = vertex_handles[2];
    auto edges = mesh.get_half_edges_of_vertex(v2);
    EXPECT_EQ(4, edges.size());

    auto v1 = vertex_handles[1];
    auto v5 = vertex_handles[5];
    auto v7 = vertex_handles[7];
    auto v3 = vertex_handles[3];

    auto edge_21 = mesh.get_half_edge_between(v1, v2).unwrap();
    auto edge_25 = mesh.get_half_edge_between(v5, v2).unwrap();
    auto edge_27 = mesh.get_half_edge_between(v7, v2).unwrap();
    auto edge_23 = mesh.get_half_edge_between(v3, v2).unwrap();

    ASSERT_THAT(edges, ::testing::UnorderedElementsAre(edge_21, edge_23, edge_25, edge_27));
}

TEST_F(TmeshTestWithCubeData, GetEdgesOfVertexThreeEdges) {
    auto v0 = vertex_handles[0];
    auto edges = mesh.get_edges_of_vertex(v0);
    EXPECT_EQ(3, edges.size());

    auto v1 = vertex_handles[1];
    auto v3 = vertex_handles[3];
    auto v21 = vertex_handles[21];

    auto edge_01 = mesh.get_edge_between(v0, v1).unwrap();
    auto edge_03 = mesh.get_edge_between(v0, v3).unwrap();
    auto edge_021 = mesh.get_edge_between(v0, v21).unwrap();

    ASSERT_THAT(edges, ::testing::UnorderedElementsAre(edge_01, edge_03, edge_021));
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgesOfVertexThreeEdges) {
    auto v0 = vertex_handles[0];
    auto edges = mesh.get_half_edges_of_vertex(v0);
    EXPECT_EQ(3, edges.size());

    auto v1 = vertex_handles[1];
    auto v3 = vertex_handles[3];
    auto v21 = vertex_handles[21];

    auto edge_01 = mesh.get_half_edge_between(v1, v0).unwrap();
    auto edge_03 = mesh.get_half_edge_between(v3, v0).unwrap();
    auto edge_021 = mesh.get_half_edge_between(v21, v0).unwrap();

    ASSERT_THAT(edges, ::testing::UnorderedElementsAre(edge_01, edge_03, edge_021));
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgesOfVertexThreeEdgesOutgoing) {
    auto v0 = vertex_handles[0];
    auto edges = mesh.get_half_edges_of_vertex(v0, edge_direction::outgoing);
    EXPECT_EQ(3, edges.size());

    auto v1 = vertex_handles[1];
    auto v3 = vertex_handles[3];
    auto v21 = vertex_handles[21];

    auto edge_01 = mesh.get_half_edge_between(v0, v1).unwrap();
    auto edge_03 = mesh.get_half_edge_between(v0, v3).unwrap();
    auto edge_021 = mesh.get_half_edge_between(v0, v21).unwrap();

    ASSERT_THAT(edges, ::testing::UnorderedElementsAre(edge_01, edge_03, edge_021));
}

TEST_F(TmeshTestWithCubeData, GetFaceBetween) {
    auto v1 = vertex_handles[1];
    auto v4 = vertex_handles[4];
    auto v5 = vertex_handles[5];
    auto v2 = vertex_handles[2];
    auto f1 = face_handles[1];
    EXPECT_EQ(f1, mesh.get_face_between({v1, v4, v5, v2}).unwrap());
    EXPECT_EQ(f1, mesh.get_face_between({v2, v5, v4, v1}).unwrap());
    EXPECT_EQ(f1, mesh.get_face_between({v2, v1, v4, v5}).unwrap());

#ifndef NDEBUG
    // This should throw an exception, because the order of the vertices is messed up
    ASSERT_THROW(mesh.get_face_between({v1, v5, v4, v2}).unwrap(), panic_exception);
#endif
}

TEST_F(TmeshTestWithCubeData, NumAdjacentFaces) {
    auto v2 = vertex_handles[2];
    auto v3 = vertex_handles[3];
    auto edge_handle = mesh.get_edge_between(v2, v3);
    EXPECT_EQ(2, mesh.num_adjacent_faces(edge_handle.unwrap()));
}

TEST_F(TmeshTestWithCubeData, GetEdgesOfFace) {
    auto f0 = face_handles[0];
    auto half_edges = mesh.get_half_edges_of_face(f0);
    EXPECT_EQ(4, half_edges.size());

    vector<half_edge_handle> expected_handles;
    expected_handles.reserve(4);
    expected_handles.push_back(mesh.get_half_edge_between(vertex_handles[0], vertex_handles[1]).unwrap());
    expected_handles.push_back(mesh.get_half_edge_between(vertex_handles[1], vertex_handles[2]).unwrap());
    expected_handles.push_back(mesh.get_half_edge_between(vertex_handles[2], vertex_handles[3]).unwrap());
    expected_handles.push_back(mesh.get_half_edge_between(vertex_handles[3], vertex_handles[0]).unwrap());

    ASSERT_THAT(half_edges, ::testing::UnorderedElementsAreArray(expected_handles));
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgeBetweenFaces) {
    auto f0 = face_handles[0];
    auto f1 = face_handles[1];
    auto v1 = vertex_handles[1];
    auto v2 = vertex_handles[2];
    auto eh = mesh.get_half_edge_between(f0, f1);
    auto eeh = mesh.get_half_edge_between(v1, v2);
    EXPECT_EQ(eeh, eh);
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgesOfFace) {
    auto f0 = face_handles[0];
    auto half_edges = mesh.get_half_edges_of_face(f0);
    EXPECT_EQ(4, half_edges.size());

    auto v0 = vertex_handles[0];
    auto v1 = vertex_handles[1];
    auto v2 = vertex_handles[2];
    auto v3 = vertex_handles[3];

    auto edge_01 = mesh.get_half_edge_between(v0, v1).unwrap();
    auto edge_12 = mesh.get_half_edge_between(v1, v2).unwrap();
    auto edge_23 = mesh.get_half_edge_between(v2, v3).unwrap();
    auto edge_30 = mesh.get_half_edge_between(v3, v0).unwrap();

    ASSERT_THAT(half_edges, ::testing::UnorderedElementsAre(edge_01, edge_12, edge_23, edge_30));
}

TEST_F(TmeshTestWithCubeData, GetNeighboursOfFace) {
    auto f0 = face_handles[0];
    auto neighbours = mesh.get_neighbours_of_face(f0);
    EXPECT_EQ(4, neighbours.size());

    auto f1 = face_handles[1];
    auto f2 = face_handles[2];
    auto f13 = face_handles[13];
    auto f18 = face_handles[18];
    ASSERT_THAT(neighbours, ::testing::UnorderedElementsAre(f1, f2, f13, f18));
}

TEST_F(TmeshTestWithCubeData, GetHalfEdgesOfEdge) {
    auto v0 = vertex_handles[0];
    auto v1 = vertex_handles[1];
    auto edge_handle = mesh.get_edge_between(v0, v1);
    auto half_edges = mesh.get_half_edges_of_edge(edge_handle.unwrap());

    auto expected_half_edge01 = mesh.get_half_edge_between(v0, v1).unwrap();
    auto expected_half_edge10 = mesh.get_half_edge_between(v1, v0).unwrap();

    ASSERT_THAT(half_edges, ::testing::UnorderedElementsAre(expected_half_edge01, expected_half_edge10));
}

TEST_F(TmeshTestAsGrid, RemoveEdge) {
    // v4 ===== v1 ===== v0
    // ||       ||       ||
    // ||  f??  ||  f??  ||
    // ||       ||       ||
    // v5 ===== v2 ===== v3
    auto v0 = vertex_handles[41];
    auto v1 = vertex_handles[40];
    auto v2 = vertex_handles[49];
    auto v3 = vertex_handles[50];

    auto v4 = vertex_handles[39];
    auto v5 = vertex_handles[48];

    auto num_faces = mesh.num_faces();
    auto num_vertices = mesh.num_vertices();
    auto num_edges = mesh.num_edges();
    auto num_half_edges = mesh.num_half_edges();

    auto edge12 = mesh.get_edge_between(v1, v2);
    ASSERT_TRUE(mesh.remove_edge(edge12.unwrap()));

    // test valence
    EXPECT_EQ(4, mesh.get_extended_valence(v2));
    EXPECT_EQ(3, mesh.get_valence(v2));
    EXPECT_EQ(4, mesh.get_extended_valence(v1));
    EXPECT_EQ(3, mesh.get_valence(v1));

    // try to get deleted edge
    edge12 = mesh.get_edge_between(v1, v2);
    ASSERT_FALSE(edge12);

    // check edge and face count decresed
    EXPECT_EQ(num_faces - 1, mesh.num_faces());
    EXPECT_EQ(num_vertices, mesh.num_vertices());
    EXPECT_EQ(num_edges - 1, mesh.num_edges());
    EXPECT_EQ(num_half_edges - 2, mesh.num_half_edges());

    // try to get deleted face
    auto face0 = mesh.get_face_between({v0, v1, v2, v3});
    auto face1 = mesh.get_face_between({v1, v4, v5, v2});
    EXPECT_EQ(face1, face0);
}

TEST_F(TmeshTestAsGrid, RemoveEdgeWithOneTVertex) {
    // v30 ==== v31 ==== v32
    // ||       ||       ||
    // ||  f27  ||  f28  ||
    // ||       ||       ||
    // v39 ==== v40 ==== v41
    // ||       ||       ||
    // ||  f35  ||  f36  ||
    // ||       ||       ||
    // v48 ==== v49 ==== v50
    auto v30 = vertex_handles[30];
    auto v31 = vertex_handles[31];
    auto v32 = vertex_handles[32];
    auto v39 = vertex_handles[39];
    auto v40 = vertex_handles[40];
    auto v41 = vertex_handles[41];
    auto v48 = vertex_handles[48];
    auto v49 = vertex_handles[49];
    auto v50 = vertex_handles[50];

    auto f35 = face_handles[35];
    auto f36 = face_handles[36];

    // v30 ==== v31 ==== v32
    // ||                ||
    // ||      f??       ||
    // ||                ||
    // v39 ==== v40 ==== v41
    // ||       ||       ||
    // ||  f35  ||  f36  ||
    // ||       ||       ||
    // v48 ==== v49 ==== v50
    auto edge1 = mesh.get_edge_between(v31, v40);
    mesh.remove_edge(edge1.unwrap());

    auto num_faces = mesh.num_faces();
    auto num_vertices = mesh.num_vertices();
    auto num_edges = mesh.num_edges();
    auto num_half_edges = mesh.num_half_edges();

    auto edge2 = mesh.get_edge_between(v40, v49);
    ASSERT_FALSE(mesh.remove_edge(edge2.unwrap(), true));
    ASSERT_TRUE(mesh.remove_edge(edge2.unwrap(), false));

    // test valence
    EXPECT_EQ(4, mesh.get_extended_valence(v49));
    EXPECT_EQ(3, mesh.get_valence(v49));
    EXPECT_EQ(4, mesh.get_extended_valence(v31));
    EXPECT_EQ(3, mesh.get_valence(v31));

    // check edge and face count decresed
    EXPECT_EQ(num_faces - 1, mesh.num_faces());
    EXPECT_EQ(num_vertices - 1, mesh.num_vertices());
    EXPECT_EQ(num_edges - 2, mesh.num_edges());
    EXPECT_EQ(num_half_edges - 4, mesh.num_half_edges());

    EXPECT_EQ(3, mesh.get_faces_of_vertex(v49).size());
    auto face = mesh.get_face_between({v39, v48, v49, v50, v41}).unwrap();
    vector<face_handle> faces = {f35, f36};
    EXPECT_TRUE(std::find(faces.begin(), faces.end(), face) != faces.end());
}

TEST_F(TmeshTestWithTfaceTest, GetExtendedValence) {
    EXPECT_EQ(4, mesh.get_extended_valence(vertex_handles[4]));
}

TEST_F(TmeshTestSplittingOperations, TestSetup) {
    EXPECT_EQ(2, mesh.num_faces());
    EXPECT_EQ(6, mesh.num_vertices());
    EXPECT_EQ(14, mesh.num_half_edges());

    // find the half-edge between vertex handles 3 and 2
    auto heh = mesh.get_half_edge_between(vertex_handles[3], vertex_handles[2]).unwrap();
    EXPECT_TRUE(mesh.get_face_of_half_edge(heh));
    EXPECT_EQ(face_handles[1], mesh.get_face_of_half_edge(heh).unwrap());

    // ensure that all knot intervals are as presented in the test
    EXPECT_EQ(2.0, expect(mesh.get_knot_interval(heh), "knot interval should be valid for this halfedge in this test"));
    EXPECT_EQ(1.0, expect(mesh.get_knot_interval(mesh.get_next(heh)), "knot interval should be valid for this halfedge in this test"));
    EXPECT_EQ(2.0, expect(mesh.get_knot_interval(mesh.get_next(mesh.get_next(heh))), "knot interval should be valid for this halfedge in this test"));
    EXPECT_EQ(1.0, expect(mesh.get_knot_interval(mesh.get_prev(heh)), "knot interval should be valid for this halfedge in this test"));

    heh = mesh.get_twin(heh);
    auto iter = heh;
    do {
        EXPECT_EQ(1.0, expect(mesh.get_knot_interval(iter), "knot interval should be valid for this halfedge in this test"));
        iter = mesh.get_next(iter);
    } while (iter != heh);
}

// test splitting of interior edge with different scales
TEST_F(TmeshTestSplittingOperations, SplitInteriorEdge) {
    // Split to the following configuration
    // v4-----------------------------v5
    // |              2                |
    // |                               |
    // |                               |
    // |                               |
    // | 1                           1 |
    // |                               |
    // |                               |
    // |        4/3            2/3     |
    // v3-----------------v6----------v2
    // |        2/3            1/3     |
    // |                               |
    // |                               |
    // |                               |
    // | 1                           1 |
    // |                               |
    // |                               |
    // |             1                 |
    // v0-----------------------------v1
    auto heh = mesh.get_half_edge_between(vertex_handles[3], vertex_handles[2]).unwrap();
    bool is_control_mesh_vertex = false;
//    auto split_vert = mesh.split_edge_at_interval(heh, 0.66666666666666667, is_control_mesh_vertex);
//
//    // Check correct number of vertices
//    EXPECT_EQ(7, mesh.num_vertices());
//    EXPECT_EQ(6, mesh.num_control_vertices());
//    EXPECT_EQ(1, mesh.num_virtual_vertices());
//
//    // Check correct number of half-edges and edges
//    EXPECT_EQ(16, mesh.num_half_edges());
//    EXPECT_EQ(16, mesh.num_control_half_edges());
//    EXPECT_EQ(0, mesh.num_virtual_half_edges());
//    EXPECT_EQ(8, mesh.num_edges());
//    EXPECT_EQ(8, mesh.num_control_edges());
//    EXPECT_EQ(0, mesh.num_virtual_edges());
//
//    // Check knot intervals on the halfedges
//    EXPECT_DOUBLE_EQ(4.0/3.0, expect(mesh.get_knot_interval(heh), "knot interval error"));
//    EXPECT_DOUBLE_EQ(2.0/3.0, expect(mesh.get_knot_interval(mesh.get_next(heh)), "knot interval error"));
//    EXPECT_DOUBLE_EQ(2.0/3.0, expect(mesh.get_knot_interval(mesh.get_twin(heh)), "knot interval error"));
//    EXPECT_DOUBLE_EQ(1.0/3.0, expect(mesh.get_knot_interval(mesh.get_next(mesh.get_twin(heh))), "knot interval error"));
}

TEST_F(TmeshTestBezierMeshExtraction, TestSetup) {
    // ensure proper setup of mesh
    int n_border = 0;
    int n_one_interval = 0;
    int n_two_interval = 0;
    int n_t_junction = 0;
    for(const auto& handle : mesh.get_half_edges())
    {
        if(mesh.is_border(handle))
            ++n_border;
        else {
            if(expect(mesh.get_knot_interval(handle), "all interior half-edges should have a well-defined knot interval") == 1.0)
                ++n_one_interval;
            else
                ++n_two_interval;

            if(!expect(mesh.corner(handle), "All interior half-edges should have a well-defined set of corner vertices"))
                ++n_t_junction;
        }
    }

    EXPECT_EQ(13, n_border);
    EXPECT_EQ(32, n_one_interval);
    EXPECT_EQ(5, n_two_interval);
    EXPECT_EQ(1, n_t_junction);
}

TEST_F(TmeshTestBezierMeshExtraction, ExtractBezierMesh) {
    EXPECT_EQ(17, mesh.num_control_vertices());
    EXPECT_EQ(0, mesh.num_virtual_vertices());
    EXPECT_EQ(9, mesh.num_faces());
    EXPECT_EQ(50, mesh.num_control_half_edges());
    EXPECT_EQ(0, mesh.num_virtual_half_edges());

    // perform Bezier mesh extraction
    mesh.extend_to_bezier_mesh();

    EXPECT_EQ(17, mesh.num_control_vertices());
    EXPECT_EQ(2, mesh.num_virtual_vertices());
    EXPECT_EQ(11, mesh.num_faces());
    EXPECT_EQ(54, mesh.num_control_half_edges());
    EXPECT_EQ(4, mesh.num_virtual_half_edges());
}

}
