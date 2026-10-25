#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"
#include "roam/blocks/search/flood_fill_nav_mesh.h"
#include "roam/query/find_path.h"
#include "roam/query/node.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

using namespace roam;

TEST_CASE("merge positions and indices shares the first vertex") {
    const float positions[] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    const uint32_t first[] = {0, 1, 2};
    const uint32_t second[] = {0, 2, 3};
    const InputTriangleMesh meshes[] = {{positions, first}, {positions, second}};

    MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices(meshes);

    CHECK(merged.positions.length == 12);
    REQUIRE(merged.indices.length == 6);
    const uint32_t expected[] = {0, 1, 2, 0, 2, 3};
    for (uint32_t i = 0; i < 6; i++) CHECK(merged.indices[i] == expected[i]);

    merge_positions_and_indices::release(merged);
}

TEST_CASE("merge positions and indices treats negative zero as zero") {
    const float positions[] = {0, 0, 0, -0.0f, 0, -0.0f};
    const uint32_t indices[] = {0, 1};
    const InputTriangleMesh mesh = {positions, indices};

    MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices({&mesh, 1});

    CHECK(merged.positions.length == 3);
    REQUIRE(merged.indices.length == 2);
    CHECK(merged.indices[1] == 0);

    merge_positions_and_indices::release(merged);
}

TEST_CASE("chunky tri mesh of nothing is empty") {
    ChunkyTriMesh mesh = chunky_tri_mesh::create({});
    CHECK(mesh.nodes.length == 0);
    CHECK(mesh.max_tris_per_chunk == 0);

    Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_rect(mesh, {0, 0}, {1, 1});
    CHECK(chunks.length == 0);

    chunks.release();
    chunky_tri_mesh::release(mesh);
}

TEST_CASE("chunky tri mesh finds every triangle in a covering rect") {
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    for (int z = 0; z < 10; z++) {
        for (int x = 0; x < 10; x++) {
            uint32_t base = positions.length / 3;
            const float corners[] = {(float)x, 0, (float)z, (float)x + 1, 0, (float)z, (float)x + 1, 0, (float)z + 1, (float)x, 0, (float)z + 1};
            for (float value : corners) positions.push(value);
            const uint32_t quad[] = {base, base + 1, base + 2, base, base + 2, base + 3};
            for (uint32_t index : quad) indices.push(index);
        }
    }

    ChunkyTriMesh mesh = chunky_tri_mesh::create({positions, indices}, 16);
    CHECK(mesh.max_tris_per_chunk <= 16);
    CHECK(mesh.triangles.length == indices.length);

    Array<uint32_t> all = chunky_tri_mesh::get_triangles_in_rect(mesh, {-1, -1}, {11, 11});
    CHECK(all.length == indices.length);

    Array<uint32_t> corner = chunky_tri_mesh::get_triangles_in_rect(mesh, {0.1f, 0.1f}, {0.2f, 0.2f});
    CHECK(corner.length > 0);
    CHECK(corner.length < indices.length);

    Array<uint32_t> missed = chunky_tri_mesh::get_chunks_overlapping_segment(mesh, {20, 20}, {30, 30});
    CHECK(missed.length == 0);

    missed.release();
    corner.release();
    all.release();
    chunky_tri_mesh::release(mesh);
    positions.release();
    indices.release();
}

TEST_CASE("flood fill nav mesh splits disconnected islands") {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 7, 0, 0, 9, 0, 0, 9, 0, 2, 7, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions, 8, indices, 12);

    NodeRef seed = INVALID_NODE_REF;
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (node.allocated) {
            seed = node.ref;
            break;
        }
    }

    NodeRef reachable[8];
    NodeRef unreachable[8];
    FloodFillNavMeshResult result;
    flood_fill_nav_mesh::flood_fill_nav_mesh(result, nav_mesh, {&seed, 1}, reachable, unreachable);
    CHECK(result.reachable_count == 2);
    CHECK(result.unreachable_count == 2);
    CHECK(reachable[0] == seed);

    nav_mesh_api::release(nav_mesh);
}

static const float PLANE_POSITIONS[] = {0, 0, 0, 10, 0, 0, 10, 0, 10, 0, 0, 10};
static const uint32_t PLANE_INDICES[] = {0, 2, 1, 0, 3, 2};

TEST_CASE("generate solo nav mesh covers a plane") {
    SoloNavMeshOptions options = {
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .walkable_radius_voxels = 2,
        .walkable_radius_world = 0.4f,
        .walkable_climb_voxels = 2,
        .walkable_climb_world = 0.4f,
        .walkable_height_voxels = 10,
        .walkable_height_world = 2,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 1.2f,
        .detail_sample_max_error = 0.2f,
    };

    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh({PLANE_POSITIONS, PLANE_INDICES}, options);
    CHECK(result.intermediates.poly_mesh.poly_count > 0);

    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    StraightPathPoint points[64];
    FindPathResult path;
    find_path::find_path(path, workspace, result.nav_mesh, {1, 0, 1}, {9, 0, 9}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);
    CHECK(path.success);

    nav_mesh_search::release(workspace);
    generate_solo_nav_mesh::release(result.intermediates);
    nav_mesh_api::release(result.nav_mesh);
}

TEST_CASE("generate tiled nav mesh connects tiles across a plane") {
    TiledNavMeshOptions options = {
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .tile_size_voxels = 25,
        .tile_size_world = 25 * 0.2f,
        .walkable_radius_voxels = 2,
        .walkable_radius_world = 0.4f,
        .walkable_climb_voxels = 2,
        .walkable_climb_world = 0.4f,
        .walkable_height_voxels = 10,
        .walkable_height_world = 2,
        .walkable_slope_angle_degrees = 45,
        .border_size = 5,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 1.2f,
        .detail_sample_max_error = 0.2f,
    };

    TiledNavMeshResult result = generate_tiled_nav_mesh::generate_tiled_nav_mesh({PLANE_POSITIONS, PLANE_INDICES}, options);
    CHECK(result.intermediates.poly_mesh.length == 4);

    uint32_t tile_count = 0;
    for (const NavMeshTile &tile : result.nav_mesh.tiles) tile_count += tile.allocated ? 1 : 0;
    CHECK(tile_count == 4);

    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    StraightPathPoint points[64];
    NodeRef node_path[256];
    FindPathResult path;
    find_path::find_path(path, workspace, result.nav_mesh, {1, 0, 1}, {9, 0, 9}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points,
                         {.node_path = node_path});
    CHECK(path.success);
    REQUIRE(path.node_path_count > 0);
    const NavMeshNode &first = nav_mesh_api::get_node_by_ref(result.nav_mesh, node_path[0]);
    const NavMeshNode &last = nav_mesh_api::get_node_by_ref(result.nav_mesh, node_path[path.node_path_count - 1]);
    CHECK(first.tile_id != last.tile_id);
    CHECK((path.flags & FIND_PATH_RESULT_COMPLETE_PATH) != 0);

    nav_mesh_search::release(workspace);
    generate_tiled_nav_mesh::release(result.intermediates);
    nav_mesh_api::release(result.nav_mesh);
}
