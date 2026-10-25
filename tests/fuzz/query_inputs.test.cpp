#include <math.h>

#include "roam/query/find_path.h"
#include "roam/query/find_smooth_path.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/local_neighbourhood.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"
#include "tests/fuzz/fuzz.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

static const uint32_t CAPACITIES[] = {0, 1, 2, 7, 64};
static const uint32_t MAX_CAPACITY = 64;

struct QueryFuzz {
    test::Random random;
    uint32_t successes;
    NavMesh nav_mesh;
    QueryWorkspace workspace;
    Array<NodeRef> live_refs;
    Array<NodeRef> stale_refs;
};

static NodeRef pick_ref(QueryFuzz &fuzz) {
    test::Random &random = fuzz.random;
    NodeRef live = fuzz.live_refs[test::next_below(random, fuzz.live_refs.length)];
    if (!test::one_in(random, 4)) return live;
    switch (test::next_below(random, 5)) {
        case 0: return INVALID_NODE_REF;
        case 1: return test::next_u64(random);
        case 2: return fuzz.stale_refs[test::next_below(random, fuzz.stale_refs.length)];
        case 3: return node::ser_node_ref(node::get_node_ref_type(live), node::get_node_ref_index(live), node::get_node_ref_sequence(live) + 1);
        default: return node::ser_node_ref(NODE_TYPE_POLY, node::MAX_NODE_INDEX, 0);
    }
}

static float bad_coordinate(test::Random &random) {
    switch (test::next_below(random, 5)) {
        case 0: return NAN;
        case 1: return INFINITY;
        case 2: return -INFINITY;
        case 3: return 1e30f;
        default: return test::next_float(random, -80, 100);
    }
}

static float pick_coordinate(test::Random &random, float min, float max) {
    return test::one_in(random, 4) ? bad_coordinate(random) : test::next_float(random, min, max);
}

static Vec3 pick_position(test::Random &random) {
    Vec3 position = {test::next_float(random, 0, 24), test::next_float(random, -0.5f, 0.5f), test::next_float(random, 0, 24)};
    if (test::one_in(random, 4)) {
        float bad = bad_coordinate(random);
        switch (test::next_below(random, 3)) {
            case 0: position.x = bad; break;
            case 1: position.y = bad; break;
            default: position.z = bad; break;
        }
    }
    return position;
}

static Vec3 pick_extents(test::Random &random) {
    if (test::one_in(random, 16)) return {0, 0, 0};
    if (test::one_in(random, 16)) return {NAN, 1, 1};
    return {test::next_float(random, 0.1f, 4), test::next_float(random, 0.5f, 4), test::next_float(random, 0.1f, 4)};
}

static uint32_t pick_capacity(test::Random &random) { return CAPACITIES[test::next_below(random, sizeof(CAPACITIES) / sizeof(CAPACITIES[0]))]; }

static const QueryFilter &pick_filter(test::Random &random) {
    return test::one_in(random, 4) ? nav_mesh_api::ANY_QUERY_FILTER : nav_mesh_api::DEFAULT_QUERY_FILTER;
}

static bool counts_consistent(uint32_t count, uint32_t total, bool truncated, uint32_t capacity) {
    return count <= capacity && count <= total && truncated == (total > count);
}

static bool refs_valid(const NavMesh &nav_mesh, const NodeRef *refs, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        if (!nav_mesh_api::is_valid_node_ref(nav_mesh, refs[i])) return false;
    }
    return true;
}

static bool finite(Vec3 v) { return isfinite(v.x) && isfinite(v.y) && isfinite(v.z); }

static void fuzz_find_nearest_poly(QueryFuzz &fuzz) {
    FindNearestPolyResult out = {};
    nav_mesh_api::find_nearest_poly(out, fuzz.nav_mesh, pick_position(fuzz.random), pick_extents(fuzz.random), pick_filter(fuzz.random));
    if (out.success) {
        fuzz.successes++;
        CHECK(nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, out.node_ref));
        CHECK(finite(out.position));
    }
}

static void fuzz_query_polygons(QueryFuzz &fuzz) {
    NodeRef polys[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    Vec3 center = pick_position(fuzz.random);
    Vec3 extents = pick_extents(fuzz.random);
    QueryPolygonsResult out = {};
    nav_mesh_api::query_polygons(out, fuzz.nav_mesh, {vec3::subtract(center, extents), vec3::add(center, extents)}, pick_filter(fuzz.random),
                                 {polys, capacity});
    CHECK(counts_consistent(out.poly_count, out.poly_total, out.truncated, capacity));
    CHECK(refs_valid(fuzz.nav_mesh, polys, out.poly_count));
    if (out.poly_count > 0) fuzz.successes++;
}

static void fuzz_find_node_path(QueryFuzz &fuzz) {
    NodeRef path[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    NodeRef start = pick_ref(fuzz);
    NodeRef end = pick_ref(fuzz);
    INFO("start ", start, " end ", end);
    FindNodePathResult out = {};
    nav_mesh_search::find_node_path(out, fuzz.workspace, fuzz.nav_mesh, start, end, pick_position(fuzz.random), pick_position(fuzz.random),
                                    pick_filter(fuzz.random), {path, capacity}, {test::one_in(fuzz.random, 3) ? 5.0f : 0.0f});
    if (!nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, start) || !nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, end)) CHECK_FALSE(out.success);
    CHECK(counts_consistent(out.path_count, out.path_total, out.truncated, capacity));
    CHECK(refs_valid(fuzz.nav_mesh, path, out.path_count));
    if (out.success) fuzz.successes++;
    if (out.success && out.path_count > 0) CHECK(path[0] == start);
}

static void fuzz_find_path(QueryFuzz &fuzz) {
    StraightPathPoint points[MAX_CAPACITY];
    NodeRef node_path[MAX_CAPACITY];
    uint32_t point_capacity = pick_capacity(fuzz.random);
    uint32_t node_capacity = pick_capacity(fuzz.random);
    FindPathResult out = {};
    find_path::find_path(out, fuzz.workspace, fuzz.nav_mesh, pick_position(fuzz.random), pick_position(fuzz.random), pick_extents(fuzz.random),
                         pick_filter(fuzz.random), {points, point_capacity}, {.raycast_distance = 0, .node_path = {node_path, node_capacity}});
    if (out.success) fuzz.successes++;
    CHECK(out.point_count <= point_capacity);
    CHECK(out.node_path_count <= node_capacity);
    CHECK(out.node_path_count <= out.node_path_total);
    CHECK(refs_valid(fuzz.nav_mesh, node_path, out.node_path_count));
    for (uint32_t i = 0; i < out.point_count; i++) CHECK(finite(points[i].position));
}

static void fuzz_find_straight_path(QueryFuzz &fuzz) {
    NodeRef node_path[16];
    uint32_t node_count = test::next_below(fuzz.random, 17);
    for (uint32_t i = 0; i < node_count; i++) node_path[i] = pick_ref(fuzz);
    StraightPathPoint points[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    FindStraightPathResult out = {};
    find_straight_path::find_straight_path(out, fuzz.nav_mesh, pick_position(fuzz.random), pick_position(fuzz.random), {node_path, node_count},
                                           {points, capacity}, test::next_below(fuzz.random, 4));
    CHECK(out.point_count <= capacity);
    if (out.success) {
        fuzz.successes++;
        for (uint32_t i = 0; i < out.point_count; i++) CHECK(finite(points[i].position));
    }
}

static void fuzz_find_smooth_path(QueryFuzz &fuzz) {
    SmoothPathPoint points[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    float step = test::one_in(fuzz.random, 8) ? 0.0f : test::next_float(fuzz.random, 0.1f, 2);
    FindSmoothPathResult out = {};
    find_smooth_path::find_smooth_path(out, fuzz.workspace, fuzz.nav_mesh, pick_position(fuzz.random), pick_position(fuzz.random),
                                       pick_extents(fuzz.random), pick_filter(fuzz.random), {points, capacity},
                                       {.step_size = step, .slop = 0.01f, .raycast_distance = 0, .node_path = {}});
    CHECK(out.point_count <= capacity);
    if (out.success) fuzz.successes++;
}

static Vec3 pick_start_position(QueryFuzz &fuzz, NodeRef start) {
    Vec3 position = pick_position(fuzz.random);
    if (test::one_in(fuzz.random, 4)) return position;
    GetClosestPointOnPolyResult closest = {};
    nav_mesh_api::get_closest_point_on_poly(closest, fuzz.nav_mesh, start, position);
    return closest.success ? closest.position : position;
}

static void fuzz_raycast(QueryFuzz &fuzz) {
    NodeRef path[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    NodeRef start = pick_ref(fuzz);
    Vec3 from = pick_start_position(fuzz, start);
    INFO("start ", start);
    RaycastResult out = {};
    if (test::one_in(fuzz.random, 2)) {
        nav_mesh_search::raycast(out, fuzz.nav_mesh, start, from, pick_position(fuzz.random), pick_filter(fuzz.random), {path, capacity});
    } else {
        nav_mesh_search::raycast_with_costs(out, fuzz.nav_mesh, start, from, pick_position(fuzz.random), pick_filter(fuzz.random), pick_ref(fuzz),
                                            {path, capacity});
    }
    CHECK(counts_consistent(out.path_count, out.path_total, out.truncated, capacity));
    CHECK(refs_valid(fuzz.nav_mesh, path, out.path_count));
    if (out.path_total > 1) fuzz.successes++;
}

static void fuzz_move_along_surface(QueryFuzz &fuzz) {
    NodeRef visited[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    NodeRef start = pick_ref(fuzz);
    Vec3 from = pick_start_position(fuzz, start);
    Vec3 to = pick_position(fuzz.random);
    INFO("start ", start);
    MoveAlongSurfaceResult out = {};
    nav_mesh_search::move_along_surface(out, fuzz.workspace, fuzz.nav_mesh, start, from, to, pick_filter(fuzz.random), {visited, capacity});
    if (!nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, start) || !finite(from) || !finite(to)) CHECK_FALSE(out.success);
    CHECK(counts_consistent(out.visited_count, out.visited_total, out.truncated, capacity));
    CHECK(refs_valid(fuzz.nav_mesh, visited, out.visited_count));
    if (out.success) {
        fuzz.successes++;
        CHECK(nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, out.node_ref));
    }
}

static void fuzz_local_neighbourhood(QueryFuzz &fuzz) {
    NodeRef polys[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    NodeRef start = pick_ref(fuzz);
    INFO("start ", start);
    FindLocalNeighbourhoodResult out = {};
    local_neighbourhood::find_local_neighbourhood(out, fuzz.workspace, fuzz.nav_mesh, start, pick_position(fuzz.random),
                                                  pick_coordinate(fuzz.random, 0, 5), pick_filter(fuzz.random), {polys, capacity});
    if (!nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, start)) CHECK_FALSE(out.success);
    CHECK(counts_consistent(out.poly_count, out.poly_total, out.truncated, capacity));
    CHECK(refs_valid(fuzz.nav_mesh, polys, out.poly_count));
    if (out.success) fuzz.successes++;
}

static void fuzz_wall_segments(QueryFuzz &fuzz) {
    WallSegment segments[MAX_CAPACITY];
    NodeRef segment_refs[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    uint32_t ref_capacity = test::one_in(fuzz.random, 2) ? capacity : 0;
    NodeRef ref = pick_ref(fuzz);
    INFO("ref ", ref);
    PolyWallSegmentsResult out = {};
    local_neighbourhood::get_poly_wall_segments(out, fuzz.nav_mesh, ref, pick_filter(fuzz.random), test::one_in(fuzz.random, 2), {segments, capacity},
                                                {segment_refs, ref_capacity});
    if (!nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, ref)) CHECK_FALSE(out.success);
    CHECK(counts_consistent(out.segment_count, out.segment_total, out.truncated, capacity));
    if (out.success) fuzz.successes++;
}

static float random_unit(void *user_data) { return test::next_float(*(test::Random *)user_data, 0, 1); }

static void fuzz_random_points(QueryFuzz &fuzz) {
    FindRandomPointResult out = {};
    if (test::one_in(fuzz.random, 3)) {
        nav_mesh_search::find_random_point(out, fuzz.nav_mesh, pick_filter(fuzz.random), random_unit, &fuzz.random);
    } else {
        NodeRef start = pick_ref(fuzz);
        INFO("start ", start);
        nav_mesh_search::find_random_point_around_circle(out, fuzz.workspace, fuzz.nav_mesh, start, pick_position(fuzz.random),
                                                         pick_coordinate(fuzz.random, 0, 6), pick_filter(fuzz.random), random_unit, &fuzz.random);
        if (!nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, start)) CHECK_FALSE(out.success);
    }
    if (out.success) {
        fuzz.successes++;
        CHECK(nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, out.node_ref));
    }
}

static void fuzz_closest_points(QueryFuzz &fuzz) {
    NodeRef ref = pick_ref(fuzz);
    INFO("ref ", ref);
    bool valid = nav_mesh_api::is_valid_node_ref(fuzz.nav_mesh, ref);
    GetClosestPointOnPolyResult closest = {};
    nav_mesh_api::get_closest_point_on_poly(closest, fuzz.nav_mesh, ref, pick_position(fuzz.random));
    if (!valid) CHECK_FALSE(closest.success);
    if (closest.success) fuzz.successes++;
    GetClosestPointOnPolyBoundaryResult boundary = {};
    nav_mesh_api::get_closest_point_on_poly_boundary(boundary, fuzz.nav_mesh, ref, pick_position(fuzz.random));
    if (!valid) CHECK_FALSE(boundary.success);
    GetPortalPointsResult portal = {};
    nav_mesh_search::get_portal_points(portal, fuzz.nav_mesh, ref, pick_ref(fuzz));
    GetEdgeMidPointResult mid = {};
    nav_mesh_search::get_edge_mid_point(mid, fuzz.nav_mesh, pick_ref(fuzz), ref);
}

static void fuzz_sliced_path(QueryFuzz &fuzz) {
    SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();
    NodeRef start = pick_ref(fuzz);
    NodeRef end = pick_ref(fuzz);
    INFO("start ", start, " end ", end);
    nav_mesh_search::init_sliced_find_node_path(query, fuzz.nav_mesh, start, end, pick_position(fuzz.random), pick_position(fuzz.random),
                                                pick_filter(fuzz.random), test::next_below(fuzz.random, 2));
    for (int step = 0; step < 4; step++) nav_mesh_search::update_sliced_find_node_path(query, fuzz.nav_mesh, 1 + (int)test::next_below(fuzz.random, 8));

    NodeRef path[MAX_CAPACITY];
    uint32_t capacity = pick_capacity(fuzz.random);
    SlicedFindNodePathResult out = {};
    if (test::one_in(fuzz.random, 2)) {
        nav_mesh_search::finalize_sliced_find_node_path(out, fuzz.nav_mesh, query, {path, capacity});
    } else {
        NodeRef existing[4] = {pick_ref(fuzz), pick_ref(fuzz), pick_ref(fuzz), pick_ref(fuzz)};
        nav_mesh_search::finalize_sliced_find_node_path_partial(out, fuzz.nav_mesh, query, {existing, test::next_below(fuzz.random, 5)},
                                                                {path, capacity});
    }
    CHECK(counts_consistent(out.path_count, out.path_total, out.truncated, capacity));
    CHECK(refs_valid(fuzz.nav_mesh, path, out.path_count));
    if (out.status & SLICED_FIND_NODE_PATH_SUCCESS) fuzz.successes++;
    nav_mesh_search::release(query);
}

using QueryFuzzer = void (*)(QueryFuzz &fuzz);

struct NamedFuzzer {
    const char *name;
    QueryFuzzer run;
};

static const NamedFuzzer FUZZERS[] = {
    {"find nearest poly", fuzz_find_nearest_poly},
    {"query polygons", fuzz_query_polygons},
    {"find node path", fuzz_find_node_path},
    {"find path", fuzz_find_path},
    {"find straight path", fuzz_find_straight_path},
    {"find smooth path", fuzz_find_smooth_path},
    {"raycast", fuzz_raycast},
    {"move along surface", fuzz_move_along_surface},
    {"local neighbourhood", fuzz_local_neighbourhood},
    {"wall segments", fuzz_wall_segments},
    {"random points", fuzz_random_points},
    {"closest points and portals", fuzz_closest_points},
    {"sliced node path", fuzz_sliced_path},
};

TEST_CASE("queries handle stale refs, bad positions and small buffers") {
    QueryFuzz fuzz = {};
    fuzz.random = {0x5eedull};
    fuzz.nav_mesh = create_tiled_terrain();
    fuzz.workspace = nav_mesh_search::create_query_workspace();

    const NavMeshTile *removed = nullptr;
    for (const NavMeshTile &tile : fuzz.nav_mesh.tiles) {
        if (tile.allocated) removed = &tile;
    }
    REQUIRE(removed != nullptr);
    for (uint32_t node_index : removed->poly_nodes) fuzz.stale_refs.push(fuzz.nav_mesh.nodes[node_index].ref);
    nav_mesh_api::remove_tile(fuzz.nav_mesh, removed->tile_x, removed->tile_y, removed->tile_layer);
    for (const NavMeshNode &node : fuzz.nav_mesh.nodes) {
        if (node.allocated) fuzz.live_refs.push(node.ref);
    }
    REQUIRE(fuzz.stale_refs.length > 0);
    REQUIRE(fuzz.live_refs.length > 0);

    const uint32_t fuzzer_count = sizeof(FUZZERS) / sizeof(FUZZERS[0]);
    uint32_t successes_by_fuzzer[fuzzer_count] = {};
    for (uint32_t iteration = 0; iteration < 3000; iteration++) {
        const NamedFuzzer &fuzzer = FUZZERS[iteration % fuzzer_count];
        INFO("iteration ", iteration, ": ", doctest::String(fuzzer.name));
        uint32_t before = fuzz.successes;
        fuzzer.run(fuzz);
        successes_by_fuzzer[iteration % fuzzer_count] += fuzz.successes - before;
    }
    for (uint32_t i = 0; i < fuzzer_count; i++) {
        INFO(doctest::String(FUZZERS[i].name), " succeeded ", successes_by_fuzzer[i], " times");
        CHECK(successes_by_fuzzer[i] >= 40);
    }

    fuzz.live_refs.release();
    fuzz.stale_refs.release();
    nav_mesh_search::release(fuzz.workspace);
    nav_mesh_api::release(fuzz.nav_mesh);
}
