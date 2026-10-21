#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "tests/harness/test.h"

using namespace roam;

struct TestGrid {
    Array<float> positions;
    Array<uint32_t> indices;
};

static TestGrid create_test_grid(int size) {
    TestGrid grid = {};
    for (int z = 0; z < size; z++) {
        for (int x = 0; x < size; x++) {
            uint32_t base = grid.positions.length / 3;
            const float corners[] = {(float)x, 0, (float)z, (float)x + 1, 0, (float)z, (float)x + 1, 0, (float)z + 1, (float)x, 0, (float)z + 1};
            for (float value : corners) grid.positions.push(value);
            const uint32_t quad[] = {base, base + 1, base + 2, base, base + 2, base + 3};
            for (uint32_t index : quad) grid.indices.push(index);
        }
    }
    return grid;
}

static void release_test_grid(TestGrid &grid) {
    grid.positions.release();
    grid.indices.release();
}

TEST_CASE("chunky tri mesh create should create a chunky tri mesh from vertices and indices") {
    TestGrid grid = create_test_grid(4);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices}, 8);

    CHECK(mesh.nodes.length > 0);
    CHECK(mesh.triangles.length == grid.indices.length);
    CHECK(mesh.max_tris_per_chunk <= 8);

    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh create should respect the tris per chunk parameter") {
    TestGrid grid = create_test_grid(10);
    ChunkyTriMesh mesh_4 = chunky_tri_mesh::create({grid.positions, grid.indices}, 4);
    ChunkyTriMesh mesh_16 = chunky_tri_mesh::create({grid.positions, grid.indices}, 16);

    CHECK(mesh_4.nodes.length > mesh_16.nodes.length);
    CHECK(mesh_4.max_tris_per_chunk <= 4);
    CHECK(mesh_16.max_tris_per_chunk <= 16);

    chunky_tri_mesh::release(mesh_16);
    chunky_tri_mesh::release(mesh_4);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh create should handle a single triangle") {
    const float positions[] = {0, 0, 0, 1, 0, 0, 0, 0, 1};
    const uint32_t indices[] = {0, 1, 2};
    ChunkyTriMesh mesh = chunky_tri_mesh::create({positions, indices});

    CHECK(mesh.nodes.length == 1);
    REQUIRE(mesh.triangles.length == 3);
    CHECK(mesh.triangles[0] == 0);
    CHECK(mesh.triangles[1] == 1);
    CHECK(mesh.triangles[2] == 2);
    CHECK(mesh.max_tris_per_chunk == 1);

    chunky_tri_mesh::release(mesh);
}

TEST_CASE("chunky tri mesh get chunks overlapping rect should find chunks overlapping a rectangular region") {
    TestGrid grid = create_test_grid(10);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices}, 16);

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_rect(mesh, {4, 4}, {6, 6});
    CHECK(chunks.length > 0);
    for (uint32_t chunk_index : chunks) CHECK(mesh.nodes[chunk_index].index >= 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get chunks overlapping rect should return empty array for region with no triangles") {
    TestGrid grid = create_test_grid(5);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_rect(mesh, {100, 100}, {200, 200});
    CHECK(chunks.length == 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get chunks overlapping rect should find all chunks when querying the entire bounds") {
    TestGrid grid = create_test_grid(5);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices}, 8);

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_rect(mesh, {-1, -1}, {10, 10});
    uint32_t leaf_count = 0;
    for (const ChunkyTriMeshNode &node : mesh.nodes) {
        if (node.index >= 0) leaf_count++;
    }
    CHECK(chunks.length == leaf_count);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get triangles in rect should return all triangles in a region") {
    TestGrid grid = create_test_grid(4);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> triangles = chunky_tri_mesh::get_triangles_in_rect(mesh, {0, 0}, {2, 2});
    CHECK(triangles.length > 0);
    CHECK(triangles.length % 3 == 0);

    triangles.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get triangles in rect should return valid triangle indices") {
    TestGrid grid = create_test_grid(3);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> triangles = chunky_tri_mesh::get_triangles_in_rect(mesh, {0, 0}, {3, 3});
    for (uint32_t index : triangles) CHECK(index < grid.positions.length / 3);

    triangles.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get triangles in rect should return empty array for region with no triangles") {
    TestGrid grid = create_test_grid(5);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> triangles = chunky_tri_mesh::get_triangles_in_rect(mesh, {100, 100}, {200, 200});
    CHECK(triangles.length == 0);

    triangles.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get chunks overlapping segment should find chunks overlapping a line segment") {
    TestGrid grid = create_test_grid(10);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices}, 16);

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_segment(mesh, {0, 0}, {5, 5});
    CHECK(chunks.length > 0);
    for (uint32_t chunk_index : chunks) CHECK(mesh.nodes[chunk_index].index >= 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get chunks overlapping segment should handle horizontal line segment") {
    TestGrid grid = create_test_grid(10);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_segment(mesh, {0, 5}, {10, 5});
    CHECK(chunks.length > 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get chunks overlapping segment should handle vertical line segment") {
    TestGrid grid = create_test_grid(10);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_segment(mesh, {5, 0}, {5, 10});
    CHECK(chunks.length > 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh get chunks overlapping segment should return empty array for segment outside bounds") {
    TestGrid grid = create_test_grid(5);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices});

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_segment(mesh, {100, 100}, {200, 200});
    CHECK(chunks.length == 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}

TEST_CASE("chunky tri mesh queries every region of a large mesh") {
    TestGrid grid = create_test_grid(50);
    ChunkyTriMesh mesh = chunky_tri_mesh::create({grid.positions, grid.indices}, 64);

    for (int i = 0; i < 100; i++) {
        float x = (float)((i % 10) * 5);
        float z = (float)((i / 10) * 5);
        Array<uint32_t> triangles = chunky_tri_mesh::get_triangles_in_rect(mesh, {x, z}, {x + 5, z + 5});
        CHECK(triangles.length > 0);
        triangles.release();
    }

    chunky_tri_mesh::release(mesh);
    release_test_grid(grid);
}
