#include <math.h>

#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_search.h"
#include "tests/fuzz/fuzz.h"
#include "tests/harness/test.h"

static const float CELL_SIZE = 0.2f;
static const float CELL_HEIGHT = 0.2f;

static SoloNavMeshOptions solo_options() {
    return {
        .cell_size = CELL_SIZE,
        .cell_height = CELL_HEIGHT,
        .walkable_radius_voxels = 1,
        .walkable_radius_world = 0.2f,
        .walkable_climb_voxels = 3,
        .walkable_climb_world = 0.6f,
        .walkable_height_voxels = 5,
        .walkable_height_world = 1.0f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 2,
        .merge_region_area = 10,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = CELL_SIZE * 6,
        .detail_sample_max_error = CELL_HEIGHT,
    };
}

static TiledNavMeshOptions tiled_options() {
    SoloNavMeshOptions solo = solo_options();
    return {
        .cell_size = solo.cell_size,
        .cell_height = solo.cell_height,
        .tile_size_voxels = 16,
        .tile_size_world = 16 * solo.cell_size,
        .walkable_radius_voxels = solo.walkable_radius_voxels,
        .walkable_radius_world = solo.walkable_radius_world,
        .walkable_climb_voxels = solo.walkable_climb_voxels,
        .walkable_climb_world = solo.walkable_climb_world,
        .walkable_height_voxels = solo.walkable_height_voxels,
        .walkable_height_world = solo.walkable_height_world,
        .walkable_slope_angle_degrees = solo.walkable_slope_angle_degrees,
        .border_size = solo.walkable_radius_voxels + 3,
        .min_region_area = solo.min_region_area,
        .merge_region_area = solo.merge_region_area,
        .max_simplification_error = solo.max_simplification_error,
        .max_edge_length = solo.max_edge_length,
        .max_vertices_per_poly = solo.max_vertices_per_poly,
        .detail_sample_distance = solo.detail_sample_distance,
        .detail_sample_max_error = solo.detail_sample_max_error,
    };
}

static uint32_t count_polys(const NavMesh &nav_mesh) {
    uint32_t count = 0;
    for (const NavMeshTile &tile : nav_mesh.tiles) count += tile.allocated ? tile.polys.length : 0;
    return count;
}

static void query_everywhere(const NavMesh &nav_mesh, Box3 bounds) {
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    StraightPathPoint points[32];
    test::Random random = {99};
    for (int i = 0; i < 8; i++) {
        Vec3 a = {test::next_float(random, bounds.min.x, bounds.max.x), bounds.max.y, test::next_float(random, bounds.min.z, bounds.max.z)};
        Vec3 b = {test::next_float(random, bounds.min.x, bounds.max.x), bounds.min.y, test::next_float(random, bounds.min.z, bounds.max.z)};
        FindPathResult result = {};
        find_path::find_path(result, workspace, nav_mesh, a, b, {2, 4, 2}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);
    }
    nav_mesh_search::release(workspace);
}

static uint32_t generate_both(Span<const float> positions, Span<const uint32_t> indices) {
    Box3 bounds = {{-1, -1, -1}, {1, 1, 1}};
    for (uint32_t i = 0; i + 2 < positions.capacity; i += 3) {
        Vec3 position = {positions[i], positions[i + 1], positions[i + 2]};
        if (isfinite(position.x) && isfinite(position.y) && isfinite(position.z)) bounds = box3::expand_by_point(bounds, position);
    }

    SoloNavMeshResult solo = generate_solo_nav_mesh::generate_solo_nav_mesh({positions, indices}, solo_options());
    CHECK(snapshot_is_valid(solo.nav_mesh));
    query_everywhere(solo.nav_mesh, bounds);
    uint32_t solo_polys = count_polys(solo.nav_mesh);
    generate_solo_nav_mesh::release(solo.intermediates);
    nav_mesh_api::release(solo.nav_mesh);

    TiledNavMeshResult tiled = generate_tiled_nav_mesh::generate_tiled_nav_mesh({positions, indices}, tiled_options());
    CHECK(snapshot_is_valid(tiled.nav_mesh));
    query_everywhere(tiled.nav_mesh, bounds);
    generate_tiled_nav_mesh::release(tiled.intermediates);
    nav_mesh_api::release(tiled.nav_mesh);
    return solo_polys;
}

TEST_CASE("generation handles an empty mesh") { CHECK(generate_both({}, {}) == 0); }

TEST_CASE("generation handles a single triangle") {
    const float positions[] = {0, 0, 0, 0, 0, 6, 6, 0, 0};
    const uint32_t indices[] = {0, 1, 2};
    CHECK(generate_both(positions, indices) > 0);
}

TEST_CASE("generation handles a triangle smaller than a cell") {
    const float positions[] = {1, 0, 1, 1, 0, 1.05f, 1.05f, 0, 1};
    const uint32_t indices[] = {0, 1, 2};
    generate_both(positions, indices);
}

TEST_CASE("generation handles zero area, repeated and duplicate triangles") {
    const float positions[] = {0, 0, 0, 0, 0, 6, 6, 0, 0, 3, 0, 0};
    const uint32_t indices[] = {0, 1, 2, 0, 1, 2, 0, 2, 3, 1, 1, 2, 0, 0, 0};
    CHECK(generate_both(positions, indices) > 0);
}

TEST_CASE("generation handles walls with nothing walkable") {
    const float positions[] = {0, 0, 0, 0, 4, 0, 6, 0, 0, 6, 4, 0};
    const uint32_t indices[] = {0, 1, 2, 2, 1, 3};
    CHECK(generate_both(positions, indices) == 0);
}

TEST_CASE("generation ignores triangles with non finite vertices") {
    const float positions[] = {0, 0, 0, 0, 0, 6, 6, 0, 0, NAN, 0, 3, 3, INFINITY, 3};
    const uint32_t indices[] = {0, 1, 2, 3, 1, 2, 4, 1, 0};
    CHECK(generate_both(positions, indices) > 0);
}

TEST_CASE("generation works far from the origin") {
    const float offset = 100000.0f;
    const float positions[] = {offset, 0, offset, offset, 0, offset + 6, offset + 6, 0, offset};
    const uint32_t indices[] = {0, 1, 2};
    CHECK(generate_both(positions, indices) > 0);
}

TEST_CASE("generation handles random triangle soup") {
    test::Random random = {0x5011ull};
    for (int trial = 0; trial < 6; trial++) {
        INFO("trial ", trial);
        Array<float> positions = {};
        Array<uint32_t> indices = {};
        uint32_t vertex_count = 60 + test::next_below(random, 60);
        for (uint32_t i = 0; i < vertex_count; i++) {
            positions.push(test::next_float(random, 0, 10));
            positions.push(test::one_in(random, 3) ? test::next_float(random, 0, 3) : 0.0f);
            positions.push(test::next_float(random, 0, 10));
        }
        uint32_t triangle_count = 40 + test::next_below(random, 160);
        for (uint32_t i = 0; i < triangle_count * 3; i++) indices.push(test::next_below(random, vertex_count));
        generate_both(positions, indices);
        positions.release();
        indices.release();
    }
}
