#include <stdio.h>

#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"
#include "roam/blocks/search/flood_fill_nav_mesh.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/detail_ties.h"
#include "tests/parity/nav_test_input.h"
#include "tests/parity/nav_test_nav_mesh.h"
#include "tests/parity/nav_test_pipeline.h"

struct BlocksTest {
    fixture::Fixture solo_fixture;
    fixture::Fixture expected;
    NavTestInput input;
    NavTestOptions options;
};

static BlocksTest load_blocks_test() {
    BlocksTest test = {};
    test.solo_fixture = fixture::load("nav_test_solo");
    test.expected = fixture::load("nav_test_blocks");
    if (!test.solo_fixture.loaded || !test.expected.loaded) return test;
    test.input = load_nav_test_input(test.solo_fixture);
    test.options = load_nav_test_options(test.solo_fixture);
    return test;
}

static void release(BlocksTest &test) {
    release(test.input);
    fixture::release(test.expected);
    fixture::release(test.solo_fixture);
}

static Array<int32_t> to_i32(const uint32_t *values, uint32_t count) {
    Array<int32_t> wide = {};
    for (uint32_t i = 0; i < count; i++) wide.push((int32_t)values[i]);
    return wide;
}

static void check_refs(const char *label, const Array<NodeRef> &actual, const fixture::F64 &expected) {
    bool same = actual.length == expected.count;
    for (uint32_t i = 0; same && i < actual.length; i++) same = actual[i] == ref_from_f64(expected.data[i]);
    if (same) return;
    char message[256];
    snprintf(message, sizeof(message), "%s: %u refs, expected %u", label, actual.length, expected.count);
    test::fail(__FILE__, __LINE__, message);
}

static void check_tiles(const char *prefix, const NavMesh &nav_mesh, const fixture::Fixture &expected) {
    Array<int32_t> positions = {};
    Array<float> bounds = {};
    Array<int32_t> vertex_counts = {};
    Array<int32_t> poly_counts = {};
    Array<float> vertices = {};
    Array<int32_t> poly_vertex_counts = {};
    Array<int32_t> poly_vertices = {};
    Array<int32_t> poly_neis = {};
    Array<int32_t> poly_flags = {};
    Array<int32_t> poly_areas = {};

    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        positions.push(tile.tile_x);
        positions.push(tile.tile_y);
        positions.push(tile.tile_layer);
        bounds.push(tile.bounds.min.x);
        bounds.push(tile.bounds.min.y);
        bounds.push(tile.bounds.min.z);
        bounds.push(tile.bounds.max.x);
        bounds.push(tile.bounds.max.y);
        bounds.push(tile.bounds.max.z);
        vertex_counts.push((int32_t)tile.vertices.length);
        poly_counts.push((int32_t)tile.polys.length);
        for (float value : tile.vertices) vertices.push(value);
        for (const NavMeshPoly &poly : tile.polys) {
            poly_vertex_counts.push((int32_t)poly.vertex_count);
            for (uint32_t i = 0; i < poly.vertex_count; i++) {
                poly_vertices.push((int32_t)tile.poly_vertices[poly.vertex_base + i]);
                poly_neis.push(tile.poly_neis[poly.vertex_base + i]);
            }
            poly_flags.push((int32_t)poly.flags);
            poly_areas.push(poly.area);
        }
    }

    char name[256];
    auto entry = [&](const char *field) {
        snprintf(name, sizeof(name), "%s.%s", prefix, field);
        return name;
    };

    CHECK_I32_ARRAY(entry("tile_positions"), positions.data, positions.length, fixture::i32(expected, name));
    CHECK_F64_ARRAY(entry("tile_bounds"), bounds.data, bounds.length, fixture::f64(expected, name), 1e-4);
    CHECK_I32_ARRAY(entry("tile_vertex_counts"), vertex_counts.data, vertex_counts.length, fixture::i32(expected, name));
    CHECK_I32_ARRAY(entry("tile_poly_counts"), poly_counts.data, poly_counts.length, fixture::i32(expected, name));
    CHECK_F64_ARRAY(entry("vertices"), vertices.data, vertices.length, fixture::f64(expected, name), 1e-4);
    CHECK_I32_ARRAY(entry("poly_vertex_counts"), poly_vertex_counts.data, poly_vertex_counts.length, fixture::i32(expected, name));
    CHECK_I32_ARRAY(entry("poly_vertices"), poly_vertices.data, poly_vertices.length, fixture::i32(expected, name));
    CHECK_I32_ARRAY(entry("poly_neis"), poly_neis.data, poly_neis.length, fixture::i32(expected, name));
    CHECK_I32_ARRAY(entry("poly_flags"), poly_flags.data, poly_flags.length, fixture::i32(expected, name));
    CHECK_I32_ARRAY(entry("poly_areas"), poly_areas.data, poly_areas.length, fixture::i32(expected, name));

    fixture::I32 detail_meshes = fixture::i32(expected, entry("detail_meshes"));
    fixture::F64 detail_vertices = fixture::f64(expected, entry("detail_vertices"));
    fixture::I32 detail_triangles = fixture::i32(expected, entry("detail_triangles"));
    fixture::I32 tile_detail_vertex_counts = fixture::i32(expected, entry("tile_detail_vertex_counts"));
    fixture::I32 tile_detail_triangle_counts = fixture::i32(expected, entry("tile_detail_triangle_counts"));

    uint32_t tile_index = 0;
    uint32_t poly_cursor = 0;
    uint32_t vertex_offset = 0;
    uint32_t triangle_offset = 0;
    uint32_t total_polys = 0;
    uint32_t tied_meshes = 0;
    uint32_t different_meshes = 0;
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        if (tile_index >= tile_detail_vertex_counts.count || poly_cursor + tile.polys.length > detail_meshes.count / 4) break;
        for (uint32_t p = 0; p < tile.polys.length; p++) {
            const NavMeshPolyDetail &mesh = tile.detail_meshes[p];
            DetailMeshRange actual = {(int)mesh.vertices_base, (int)mesh.vertices_count, (int)mesh.triangles_base, (int)mesh.triangles_count};
            const int32_t *range = &detail_meshes.data[(poly_cursor + p) * 4];
            DetailMeshMatch match = compare_detail_mesh(actual, tile.detail_vertices.data, tile.detail_triangles.data, {range[0], range[1], range[2], range[3]},
                                                        detail_vertices.data + vertex_offset, detail_triangles.data + triangle_offset);
            if (match == DETAIL_MESH_TIED) tied_meshes++;
            if (match == DETAIL_MESH_DIFFERENT) different_meshes++;
        }
        poly_cursor += tile.polys.length;
        vertex_offset += (uint32_t)tile_detail_vertex_counts.data[tile_index];
        triangle_offset += (uint32_t)tile_detail_triangle_counts.data[tile_index];
        total_polys += tile.polys.length;
        tile_index++;
    }
    CHECK(poly_cursor * 4 == detail_meshes.count);
    CHECK(different_meshes == 0);
    CHECK(tied_meshes <= total_polys / 25);

    check_nav_mesh(prefix, nav_mesh, expected);

    positions.release();
    bounds.release();
    vertex_counts.release();
    poly_counts.release();
    vertices.release();
    poly_vertex_counts.release();
    poly_vertices.release();
    poly_neis.release();
    poly_flags.release();
    poly_areas.release();
}

static SoloNavMeshOptions solo_options(const BlocksTest &test) {
    const NavTestOptions &options = test.options;
    fixture::F64 walkable = fixture::f64(test.solo_fixture, "nav_mesh.walkable");
    return {
        .cell_size = options.cell_size,
        .cell_height = options.cell_height,
        .walkable_radius_voxels = options.walkable_radius,
        .walkable_radius_world = (float)walkable.data[1],
        .walkable_climb_voxels = options.walkable_climb,
        .walkable_climb_world = (float)walkable.data[2],
        .walkable_height_voxels = options.walkable_height,
        .walkable_height_world = (float)walkable.data[0],
        .walkable_slope_angle_degrees = options.walkable_slope_angle,
        .border_size = options.border_size,
        .min_region_area = options.min_region_area,
        .merge_region_area = options.merge_region_area,
        .max_simplification_error = options.max_simplification_error,
        .max_edge_length = options.max_edge_length,
        .max_vertices_per_poly = options.max_vertices_per_poly,
        .detail_sample_distance = options.detail_sample_distance,
        .detail_sample_max_error = options.detail_sample_max_error,
    };
}

TEST(merge_positions_and_indices_matches_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);

    uint32_t split = (uint32_t)fixture::i32(test.expected, "merge.split").data[0];
    const MeshView meshes[] = {
        {test.input.positions.data, test.input.indices.data, split},
        {test.input.positions.data, test.input.indices.data + split, test.input.indices.length - split},
        {test.input.positions.data, test.input.indices.data, 30},
    };
    MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices(meshes, 3);

    CHECK_F64_ARRAY("merge.positions", merged.positions.data, merged.positions.length, fixture::f64(test.expected, "merge.positions"), 0);
    Array<int32_t> indices = to_i32(merged.indices.data, merged.indices.length);
    CHECK_I32_ARRAY("merge.indices", indices.data, indices.length, fixture::i32(test.expected, "merge.indices"));

    indices.release();
    merge_positions_and_indices::release(merged);
    release(test);
}

TEST(chunky_tri_mesh_matches_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);
    const fixture::Fixture &expected = test.expected;

    ChunkyTriMesh mesh = chunky_tri_mesh::create(test.input.positions.data, test.input.indices.data, test.input.indices.length, 64);

    Array<float> bounds = {};
    Array<int32_t> index = {};
    Array<int32_t> count = {};
    for (const ChunkyTriMeshNode &node : mesh.nodes) {
        bounds.push(node.min.x);
        bounds.push(node.min.y);
        bounds.push(node.max.x);
        bounds.push(node.max.y);
        index.push(node.index);
        count.push((int32_t)node.count);
    }
    CHECK_F64_ARRAY("chunky.bounds", bounds.data, bounds.length, fixture::f64(expected, "chunky.bounds"), 0);
    CHECK_I32_ARRAY("chunky.index", index.data, index.length, fixture::i32(expected, "chunky.index"));
    CHECK_I32_ARRAY("chunky.count", count.data, count.length, fixture::i32(expected, "chunky.count"));
    Array<int32_t> triangles = to_i32(mesh.triangles.data, mesh.triangles.length);
    CHECK_I32_ARRAY("chunky.triangles", triangles.data, triangles.length, fixture::i32(expected, "chunky.triangles"));
    CHECK((int32_t)mesh.max_tris_per_chunk == fixture::i32(expected, "chunky.max_tris_per_chunk").data[0]);

    fixture::F64 rects = fixture::f64(expected, "chunky.rects");
    fixture::I32 rect_counts = fixture::i32(expected, "chunky.rect_counts");
    fixture::I32 rect_chunks = fixture::i32(expected, "chunky.rect_chunks");
    fixture::I32 rect_triangle_counts = fixture::i32(expected, "chunky.rect_triangle_counts");
    fixture::F64 segments = fixture::f64(expected, "chunky.segments");
    fixture::I32 segment_counts = fixture::i32(expected, "chunky.segment_counts");
    fixture::I32 segment_chunks = fixture::i32(expected, "chunky.segment_chunks");

    uint32_t rect_cursor = 0;
    uint32_t segment_cursor = 0;
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < rect_counts.count; i++) {
        const double *rect = &rects.data[i * 4];
        Vec2 min = {(float)rect[0], (float)rect[1]};
        Vec2 max = {(float)rect[2], (float)rect[3]};

        Array<uint32_t> chunks = chunky_tri_mesh::get_chunks_overlapping_rect(mesh, min, max);
        bool same = chunks.length == (uint32_t)rect_counts.data[i];
        for (uint32_t k = 0; same && k < chunks.length; k++) same = (int32_t)chunks[k] == rect_chunks.data[rect_cursor + k];
        rect_cursor += (uint32_t)rect_counts.data[i];
        chunks.release();

        Array<uint32_t> rect_triangles = chunky_tri_mesh::get_triangles_in_rect(mesh, min, max);
        same = same && (int32_t)rect_triangles.length == rect_triangle_counts.data[i];
        rect_triangles.release();

        const double *segment = &segments.data[i * 4];
        Array<uint32_t> hits = chunky_tri_mesh::get_chunks_overlapping_segment(mesh, {(float)segment[0], (float)segment[1]}, {(float)segment[2], (float)segment[3]});
        same = same && hits.length == (uint32_t)segment_counts.data[i];
        for (uint32_t k = 0; same && k < hits.length; k++) same = (int32_t)hits[k] == segment_chunks.data[segment_cursor + k];
        segment_cursor += (uint32_t)segment_counts.data[i];
        hits.release();

        if (!same) mismatches++;
    }
    CHECK(mismatches == 0);

    bounds.release();
    index.release();
    count.release();
    triangles.release();
    chunky_tri_mesh::release(mesh);
    release(test);
}

TEST(generate_solo_nav_mesh_matches_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);

    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(
        {test.input.positions.data, test.input.indices.data, test.input.indices.length}, solo_options(test));
    check_tiles("solo", result.nav_mesh, test.expected);

    generate_solo_nav_mesh::release(result.intermediates);
    nav_mesh_api::release_nav_mesh(result.nav_mesh);
    release(test);
}

TEST(generate_tiled_nav_mesh_and_flood_fill_match_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);
    const fixture::Fixture &expected = test.expected;

    SoloNavMeshOptions solo = solo_options(test);
    TiledNavMeshOptions options = {
        .cell_size = solo.cell_size,
        .cell_height = solo.cell_height,
        .tile_size_voxels = fixture::i32(expected, "tiled.tile_size_voxels").data[0],
        .tile_size_world = (float)fixture::number(expected, "tiled.tile_size_world"),
        .walkable_radius_voxels = solo.walkable_radius_voxels,
        .walkable_radius_world = solo.walkable_radius_world,
        .walkable_climb_voxels = solo.walkable_climb_voxels,
        .walkable_climb_world = solo.walkable_climb_world,
        .walkable_height_voxels = solo.walkable_height_voxels,
        .walkable_height_world = solo.walkable_height_world,
        .walkable_slope_angle_degrees = solo.walkable_slope_angle_degrees,
        .border_size = fixture::i32(expected, "tiled.border_size").data[0],
        .min_region_area = solo.min_region_area,
        .merge_region_area = solo.merge_region_area,
        .max_simplification_error = solo.max_simplification_error,
        .max_edge_length = solo.max_edge_length,
        .max_vertices_per_poly = solo.max_vertices_per_poly,
        .detail_sample_distance = solo.detail_sample_distance,
        .detail_sample_max_error = solo.detail_sample_max_error,
    };

    TiledNavMeshResult result =
        generate_tiled_nav_mesh::generate_tiled_nav_mesh({test.input.positions.data, test.input.indices.data, test.input.indices.length}, options);
    CHECK((int32_t)result.intermediates.poly_mesh.length == fixture::i32(expected, "tiled.tile_count").data[0]);
    check_tiles("tiled", result.nav_mesh, expected);

    fixture::F64 seeds = fixture::f64(expected, "flood.seeds");
    Array<NodeRef> seed_refs = {};
    for (uint32_t i = 0; i < seeds.count; i++) seed_refs.push(ref_from_f64(seeds.data[i]));
    FloodFillNavMeshResult flood = flood_fill_nav_mesh::flood_fill_nav_mesh(result.nav_mesh, seed_refs.data, seed_refs.length);
    check_refs("flood.reachable", flood.reachable, fixture::f64(expected, "flood.reachable"));
    check_refs("flood.unreachable", flood.unreachable, fixture::f64(expected, "flood.unreachable"));

    flood_fill_nav_mesh::release(flood);
    seed_refs.release();
    generate_tiled_nav_mesh::release(result.intermediates);
    nav_mesh_api::release_nav_mesh(result.nav_mesh);
    release(test);
}
