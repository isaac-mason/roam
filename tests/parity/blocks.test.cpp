#include <stdio.h>

#include <math.h>
#include <stdlib.h>

#include "roam/blocks/agents/crowd.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"
#include "roam/blocks/search/flood_fill_nav_mesh.h"
#include "roam/util/sort.h"
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

static void check_refs(const char *label, const NodeRef *actual, uint32_t actual_count, const fixture::F64 &expected) {
    bool same = actual_count == expected.count;
    for (uint32_t i = 0; same && i < actual_count; i++) same = actual[i] == ref_from_f64(expected.data[i]);
    if (same) return;
    char message[256];
    snprintf(message, sizeof(message), "%s: %u refs, expected %u", label, actual_count, expected.count);
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
    nav_mesh_api::release(result.nav_mesh);
    release(test);
}

static TiledNavMeshResult generate_tiled(const BlocksTest &test) {
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

    return generate_tiled_nav_mesh::generate_tiled_nav_mesh({test.input.positions.data, test.input.indices.data, test.input.indices.length}, options);
}

struct LinkKey {
    int64_t from;
    int64_t to;
    int edge;
    int side;
    int bmin;
    int bmax;
};

static int64_t node_key(const NavMesh &nav_mesh, NodeRef ref) {
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, ref);
    if (node.type != NODE_TYPE_POLY) return -1 - node.off_mesh_connection_id;
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
    return ((int64_t)tile.tile_x << 40) | ((int64_t)tile.tile_y << 20) | node.poly_index;
}

static bool link_key_less(const LinkKey &a, const LinkKey &b) {
    if (a.from != b.from) return a.from < b.from;
    if (a.to != b.to) return a.to < b.to;
    if (a.edge != b.edge) return a.edge < b.edge;
    if (a.side != b.side) return a.side < b.side;
    if (a.bmin != b.bmin) return a.bmin < b.bmin;
    return a.bmax < b.bmax;
}

static Array<LinkKey> link_keys(const NavMesh &nav_mesh) {
    Array<LinkKey> keys = {};
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (!node.allocated) continue;
        for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
            const NavMeshLink &l = nav_mesh.links[(uint32_t)link];
            keys.push({node_key(nav_mesh, node.ref), node_key(nav_mesh, l.to_node_ref), l.edge, l.side, l.bmin, l.bmax});
        }
    }
    sort::stable(keys.data, keys.length, link_key_less);
    return keys;
}

template <typename T>
static Array<T> copy_of(const Array<T> &source) {
    Array<T> copy = {};
    for (const T &value : source) copy.push(value);
    return copy;
}

static NavMeshTile copy_tile(const NavMeshTile &tile) {
    NavMeshTile copy = tile;
    copy.vertices = copy_of(tile.vertices);
    copy.polys = copy_of(tile.polys);
    copy.poly_vertices = copy_of(tile.poly_vertices);
    copy.poly_neis = copy_of(tile.poly_neis);
    copy.detail_meshes = copy_of(tile.detail_meshes);
    copy.detail_vertices = copy_of(tile.detail_vertices);
    copy.detail_triangles = copy_of(tile.detail_triangles);
    copy.poly_nodes = {};
    copy.bv_tree.nodes = copy_of(tile.bv_tree.nodes);
    return copy;
}

TEST(generate_tiled_nav_mesh_and_flood_fill_match_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);
    const fixture::Fixture &expected = test.expected;

    TiledNavMeshResult result = generate_tiled(test);
    CHECK((int32_t)result.intermediates.poly_mesh.length == fixture::i32(expected, "tiled.tile_count").data[0]);
    check_tiles("tiled", result.nav_mesh, expected);

    fixture::F64 seeds = fixture::f64(expected, "flood.seeds");
    Array<NodeRef> seed_refs = {};
    for (uint32_t i = 0; i < seeds.count; i++) seed_refs.push(ref_from_f64(seeds.data[i]));
    Array<NodeRef> reachable = {};
    Array<NodeRef> unreachable = {};
    reachable.resize(result.nav_mesh.nodes.length);
    unreachable.resize(result.nav_mesh.nodes.length);
    FloodFillNavMeshResult flood;
    flood_fill_nav_mesh::flood_fill_nav_mesh(flood, result.nav_mesh, seed_refs, reachable,
                                             unreachable);
    check_refs("flood.reachable", reachable.data, flood.reachable_count, fixture::f64(expected, "flood.reachable"));
    check_refs("flood.unreachable", unreachable.data, flood.unreachable_count, fixture::f64(expected, "flood.unreachable"));

    reachable.release();
    unreachable.release();
    seed_refs.release();
    generate_tiled_nav_mesh::release(result.intermediates);
    nav_mesh_api::release(result.nav_mesh);
    release(test);
}

TEST(removing_and_re_adding_a_tile_restores_every_link) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);

    TiledNavMeshResult result = generate_tiled(test);
    NavMesh &nav_mesh = result.nav_mesh;
    Array<LinkKey> before = link_keys(nav_mesh);

    uint32_t removed = 0;
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        uint32_t neighbours = 0;
        for (const NavMeshTile &other : nav_mesh.tiles) {
            if (other.allocated && abs(other.tile_x - tile.tile_x) + abs(other.tile_y - tile.tile_y) == 1) neighbours++;
        }
        if (neighbours >= 3) {
            removed = tile.id;
            break;
        }
    }
    NavMeshTile saved = copy_tile(nav_mesh.tiles[removed]);
    REQUIRE(nav_mesh_api::remove_tile(nav_mesh, saved.tile_x, saved.tile_y, saved.tile_layer));
    Array<LinkKey> without = link_keys(nav_mesh);
    CHECK(without.length < before.length);
    nav_mesh_api::add_tile(nav_mesh, saved);

    Array<LinkKey> after = link_keys(nav_mesh);
    bool same = after.length == before.length;
    for (uint32_t i = 0; same && i < after.length; i++) {
        same = after[i].from == before[i].from && after[i].to == before[i].to && after[i].edge == before[i].edge && after[i].side == before[i].side &&
               after[i].bmin == before[i].bmin && after[i].bmax == before[i].bmax;
    }
    CHECK(same);
    CHECK(nav_mesh.link_index_pool.counter - nav_mesh.link_index_pool.free.length == after.length);

    before.release();
    without.release();
    after.release();
    generate_tiled_nav_mesh::release(result.intermediates);
    nav_mesh_api::release(nav_mesh);
    release(test);
}

struct AgentPoint {
    NodeRef ref;
    Vec3 position;
};

static Array<AgentPoint> load_agent_points(const fixture::Fixture &expected) {
    fixture::F64 points = fixture::f64(expected, "agents.points");
    Array<AgentPoint> loaded = {};
    for (uint32_t i = 0; i + 3 < points.count; i += 4) {
        loaded.push({ref_from_f64(points.data[i]), {(float)points.data[i + 1], (float)points.data[i + 2], (float)points.data[i + 3]}});
    }
    return loaded;
}

static Vec3 vec3_at(const double *values) { return {(float)values[0], (float)values[1], (float)values[2]}; }

TEST(local_boundary_matches_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);
    const fixture::Fixture &expected = test.expected;

    TiledNavMeshResult tiled = generate_tiled(test);
    Array<AgentPoint> points = load_agent_points(expected);

    fixture::I32 segment_counts = fixture::i32(expected, "boundary.segment_counts");
    fixture::F64 segments = fixture::f64(expected, "boundary.segments");
    fixture::I32 poly_counts = fixture::i32(expected, "boundary.poly_counts");
    fixture::F64 polys = fixture::f64(expected, "boundary.polys");
    REQUIRE(segment_counts.count == points.length);

    SearchWorkspace workspace = nav_mesh_search::create_search_workspace();
    uint32_t segment_cursor = 0;
    uint32_t poly_cursor = 0;
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < points.length; i++) {
        LocalBoundary boundary = local_boundary::create();
        local_boundary::update_local_boundary(boundary, points[i].ref, points[i].position, 1.5f, tiled.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER,
                                              workspace);

        bool same = boundary.segment_count == (uint32_t)segment_counts.data[i] && boundary.poly_count == (uint32_t)poly_counts.data[i];
        for (uint32_t k = 0; same && k < boundary.segment_count; k++) {
            const double *segment = &segments.data[(segment_cursor + k) * 7];
            const LocalBoundarySegment &actual = boundary.segments[k];
            same = vec3::distance(actual.start, vec3_at(segment)) < 1e-4f && vec3::distance(actual.end, vec3_at(segment + 3)) < 1e-4f &&
                   fabs((double)actual.distance_sqr - segment[6]) < 1e-4;
        }
        for (uint32_t k = 0; same && k < boundary.poly_count; k++) same = boundary.polys[k] == ref_from_f64(polys.data[poly_cursor + k]);
        segment_cursor += (uint32_t)segment_counts.data[i];
        poly_cursor += (uint32_t)poly_counts.data[i];
        if (!same) mismatches++;
    }
    CHECK(mismatches == 0);

    nav_mesh_search::release(workspace);
    points.release();
    generate_tiled_nav_mesh::release(tiled.intermediates);
    nav_mesh_api::release(tiled.nav_mesh);
    release(test);
}

TEST(obstacle_avoidance_matches_navcat) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);

    fixture::F64 inputs = fixture::f64(test.expected, "avoidance.inputs");
    fixture::F64 outputs = fixture::f64(test.expected, "avoidance.outputs");

    const uint32_t input_stride = 9 + 3 * 10 + 2 * 6;
    const uint32_t output_stride = 8;
    REQUIRE(inputs.count % input_stride == 0);

    ObstacleAvoidanceParams grid_params = DEFAULT_OBSTACLE_AVOIDANCE_PARAMS;
    grid_params.grid_size = 9;

    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < inputs.count / input_stride; i++) {
        const double *input = &inputs.data[i * input_stride];
        const double *output = &outputs.data[i * output_stride];

        ObstacleAvoidanceQuery query = obstacle_avoidance::create_obstacle_avoidance_query(8, 8);
        for (uint32_t c = 0; c < 3; c++) {
            const double *circle = input + 9 + c * 10;
            obstacle_avoidance::add_circle_obstacle(query, vec3_at(circle), (float)circle[3], vec3_at(circle + 4), vec3_at(circle + 7));
        }
        for (uint32_t c = 0; c < 2; c++) {
            const double *segment = input + 39 + c * 6;
            obstacle_avoidance::add_segment_obstacle(query, vec3_at(segment), vec3_at(segment + 3));
        }

        Vec3 position = vec3_at(input);
        Vec3 velocity = vec3_at(input + 3);
        Vec3 desired = vec3_at(input + 6);

        SampleVelocityResult adaptive = {};
        obstacle_avoidance::sample_velocity_adaptive(adaptive, query, position, 0.4f, 2, velocity, desired, DEFAULT_OBSTACLE_AVOIDANCE_PARAMS);
        SampleVelocityResult grid = {};
        obstacle_avoidance::sample_velocity_grid(grid, query, position, 0.4f, 2, velocity, desired, grid_params);

        bool same = vec3::distance(adaptive.velocity, vec3_at(output)) < 1e-3f && adaptive.samples == (int)output[3] &&
                    vec3::distance(grid.velocity, vec3_at(output + 4)) < 1e-3f && grid.samples == (int)output[7];
        if (!same) mismatches++;

        obstacle_avoidance::release(query);
    }
    CHECK(mismatches == 0);

    release(test);
}

struct CrowdComparison {
    uint32_t first_divergent_frame;
    float max_xz_error;
    float max_height_error;
    uint32_t final_state_mismatches;
};

static CrowdComparison run_crowd(Crowd &crowd, const NavMesh &nav_mesh, const Array<uint32_t> &agent_ids, uint32_t frame_count,
                                 const fixture::F64 &frames, const fixture::I32 &states) {
    CrowdComparison comparison = {};
    uint32_t sample = 0;
    for (uint32_t frame = 1; frame <= frame_count; frame++) {
        crowd::update(crowd, nav_mesh, 1.0f / 60.0f);

        for (uint32_t id : agent_ids) {
            const Agent &agent = crowd.agents[id];
            Vec3 wanted = vec3_at(&frames.data[sample * 6]);
            const int32_t *wanted_state = &states.data[sample * 3];
            sample++;

            float xz_error = vec2::distance({agent.position.x, agent.position.z}, {wanted.x, wanted.z});
            comparison.max_xz_error = fmaxf(comparison.max_xz_error, xz_error);
            comparison.max_height_error = fmaxf(comparison.max_height_error, fabsf(agent.position.y - wanted.y));

            bool same_state = agent.state == wanted_state[0] && agent.target_state == wanted_state[1] &&
                              (int32_t)agent.corridor.path.length == wanted_state[2];
            if (comparison.first_divergent_frame == 0 && (xz_error > 1e-3f || !same_state)) comparison.first_divergent_frame = frame;
            if (frame == frame_count && (agent.state != wanted_state[0] || agent.target_state != wanted_state[1])) comparison.final_state_mismatches++;
        }
    }
    return comparison;
}

static AgentParams crowd_agent_params(float radius, float max_speed, float collision_query_range, float separation_weight) {
    return {
        .radius = radius,
        .height = 1,
        .max_acceleration = 8,
        .max_speed = max_speed,
        .collision_query_range = collision_query_range,
        .separation_weight = separation_weight,
        .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OPTIMIZE_VIS |
                        CROWD_UPDATE_OPTIMIZE_TOPO,
    };
}

TEST(crowd_matches_navcat_on_a_flat_scene) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);
    const fixture::Fixture &expected = test.expected;

    fixture::F64 positions_values = fixture::f64(expected, "flat.positions");
    fixture::I32 indices_values = fixture::i32(expected, "flat.indices");
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    for (uint32_t i = 0; i < positions_values.count; i++) positions.push((float)positions_values.data[i]);
    for (uint32_t i = 0; i < indices_values.count; i++) indices.push((uint32_t)indices_values.data[i]);

    TiledNavMeshOptions options = {
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .tile_size_voxels = 32,
        .tile_size_world = 32 * 0.2f,
        .walkable_radius_voxels = 2,
        .walkable_radius_world = 0.4f,
        .walkable_climb_voxels = 2,
        .walkable_climb_world = (float)fixture::f64(test.solo_fixture, "nav_mesh.walkable").data[2],
        .walkable_height_voxels = 10,
        .walkable_height_world = 2,
        .walkable_slope_angle_degrees = test.options.walkable_slope_angle,
        .border_size = 5,
        .min_region_area = test.options.min_region_area,
        .merge_region_area = test.options.merge_region_area,
        .max_simplification_error = test.options.max_simplification_error,
        .max_edge_length = test.options.max_edge_length,
        .max_vertices_per_poly = test.options.max_vertices_per_poly,
        .detail_sample_distance = 1.2f,
        .detail_sample_max_error = 0.2f,
    };
    TiledNavMeshResult tiled = generate_tiled_nav_mesh::generate_tiled_nav_mesh({positions.data, indices.data, indices.length}, options);

    uint32_t agent_count = (uint32_t)fixture::i32(expected, "flat.agent_count").data[0];
    Crowd crowd = crowd::create(0.6f);
    Array<uint32_t> agent_ids = {};
    for (uint32_t i = 0; i < agent_count; i++) {
        double angle = ((double)i / agent_count) * 3.14159265358979323846 * 2;
        Vec3 start = {(float)(cos(angle) * 6.5), 0, (float)(sin(angle) * 6.5)};
        Vec3 end = {-start.x, 0, -start.z};
        AgentParams params = crowd_agent_params(0.4f, 3, 2.4f, 1);
        params.height = 2;
        uint32_t id = crowd::add_agent(crowd, tiled.nav_mesh, start, params);
        agent_ids.push(id);
        FindNearestPolyResult target = {};
        nav_mesh_api::find_nearest_poly(target, tiled.nav_mesh, end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        crowd::request_move_target(crowd, id, target.node_ref, target.position);
    }

    CrowdComparison comparison =
        run_crowd(crowd, tiled.nav_mesh, agent_ids, 240, fixture::f64(expected, "flat.frames"), fixture::i32(expected, "flat.states"));
    printf("flat crowd: first divergent frame %u, max xz error %f, max height error %f, final state mismatches %u\n", comparison.first_divergent_frame,
           comparison.max_xz_error, comparison.max_height_error, comparison.final_state_mismatches);
    CHECK(comparison.first_divergent_frame == 0);
    CHECK(comparison.final_state_mismatches == 0);

    agent_ids.release();
    crowd::release(crowd);
    generate_tiled_nav_mesh::release(tiled.intermediates);
    nav_mesh_api::release(tiled.nav_mesh);
    positions.release();
    indices.release();
    release(test);
}

TEST(crowd_matches_navcat_on_nav_test) {
    BlocksTest test = load_blocks_test();
    REQUIRE(test.expected.loaded);
    const fixture::Fixture &expected = test.expected;

    TiledNavMeshResult tiled = generate_tiled(test);
    Array<AgentPoint> points = load_agent_points(expected);

    uint32_t agent_count = (uint32_t)fixture::i32(expected, "crowd.agent_count").data[0];
    Crowd crowd = crowd::create(0.6f);
    Array<uint32_t> agent_ids = {};
    for (uint32_t i = 0; i < agent_count; i++) {
        uint32_t id = crowd::add_agent(crowd, tiled.nav_mesh, points[i].position, crowd_agent_params(0.3f, 2, 1.8f, 0.5f));
        agent_ids.push(id);
        const AgentPoint &target = points[points.length - 1 - i];
        crowd::request_move_target(crowd, id, target.ref, target.position);
    }

    CrowdComparison comparison =
        run_crowd(crowd, tiled.nav_mesh, agent_ids, 120, fixture::f64(expected, "crowd.frames"), fixture::i32(expected, "crowd.states"));
    printf("nav-test crowd: first divergent frame %u, max xz error %f, max height error %f, final state mismatches %u\n",
           comparison.first_divergent_frame, comparison.max_xz_error, comparison.max_height_error, comparison.final_state_mismatches);
    CHECK(comparison.first_divergent_frame == 0 || comparison.first_divergent_frame > 30);
    CHECK(comparison.max_xz_error < 0.5f);
    CHECK(comparison.final_state_mismatches == 0);

    agent_ids.release();
    crowd::release(crowd);
    points.release();
    generate_tiled_nav_mesh::release(tiled.intermediates);
    nav_mesh_api::release(tiled.nav_mesh);
    release(test);
}
