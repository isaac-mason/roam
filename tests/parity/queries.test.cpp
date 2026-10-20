#include <math.h>
#include <stdio.h>

#include "roam/query/find_path.h"
#include "roam/query/find_smooth_path.h"
#include "roam/query/local_neighbourhood.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_nav_mesh.h"

struct ExpectedPoints {
    fixture::I32 count;
    fixture::F64 positions;
    fixture::I32 types;
    fixture::F64 refs;
    fixture::I32 flags;
    uint32_t cursor;
};

static ExpectedPoints load_points(const fixture::Fixture &expected, const char *prefix) {
    char name[256];
    ExpectedPoints points = {};
    snprintf(name, sizeof(name), "%s.counts", prefix);
    points.count = fixture::i32(expected, name);
    snprintf(name, sizeof(name), "%s.positions", prefix);
    points.positions = fixture::f64(expected, name);
    snprintf(name, sizeof(name), "%s.types", prefix);
    points.types = fixture::i32(expected, name);
    snprintf(name, sizeof(name), "%s.refs", prefix);
    points.refs = fixture::f64(expected, name);
    snprintf(name, sizeof(name), "%s.flags", prefix);
    points.flags = fixture::i32(expected, name);
    return points;
}

static bool near(float actual, double expected, double tolerance) { return fabs((double)actual - expected) <= tolerance; }

static bool near3(Vec3 actual, const double *expected, double tolerance) {
    return near(actual.x, expected[0], tolerance) && near(actual.y, expected[1], tolerance) && near(actual.z, expected[2], tolerance);
}

static bool same_points(const Array<StraightPathPoint> &actual, ExpectedPoints &expected, uint32_t expected_count) {
    bool same = actual.length == expected_count;
    for (uint32_t i = 0; same && i < actual.length; i++) {
        uint32_t at = expected.cursor + i;
        same = near3(actual[i].position, &expected.positions.data[at * 3], 1e-3) && (int32_t)actual[i].type == expected.types.data[at] &&
               actual[i].node_ref == ref_from_f64(expected.refs.data[at]) && (int32_t)actual[i].flags == expected.flags.data[at];
    }
    expected.cursor += expected_count;
    return same;
}

static bool same_refs(const NodeRef *actual, uint32_t actual_count, const fixture::F64 &expected, uint32_t &cursor, uint32_t expected_count) {
    bool same = actual_count == expected_count;
    for (uint32_t i = 0; same && i < actual_count; i++) same = actual[i] == ref_from_f64(expected.data[cursor + i]);
    cursor += expected_count;
    return same;
}

static void report(const char *name, uint32_t mismatches, uint32_t first, uint32_t total) {
    if (mismatches == 0) return;
    char message[256];
    snprintf(message, sizeof(message), "%s: %u of %u differ, first at pair %u", name, mismatches, total, first);
    test::fail(__FILE__, __LINE__, message);
}

struct Tally {
    uint32_t mismatches;
    uint32_t first;
    void record(bool same, uint32_t pair) {
        if (same) return;
        if (mismatches == 0) first = pair;
        mismatches++;
    }
};

struct Lcg {
    uint32_t state;
};

static float next_random(void *user_data) {
    Lcg &lcg = *(Lcg *)user_data;
    lcg.state = lcg.state * 1664525u + 1013904223u;
    return (float)((double)lcg.state / 4294967296.0);
}

TEST(queries_match_navcat_on_nav_test) {
    fixture::Fixture expected = fixture::load("nav_test_solo");
    REQUIRE(expected.loaded);

    NavMesh nav_mesh = load_nav_test_nav_mesh(expected, true);
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;

    fixture::F64 starts = fixture::f64(expected, "paths.starts");
    fixture::F64 ends = fixture::f64(expected, "paths.ends");
    fixture::F64 half_extents_values = fixture::f64(expected, "queries.half_extents");
    Vec3 half_extents = {(float)half_extents_values.data[0], (float)half_extents_values.data[1], (float)half_extents_values.data[2]};

    fixture::I32 path_success = fixture::i32(expected, "paths.success");
    fixture::I32 path_flags = fixture::i32(expected, "paths.flags");
    fixture::I32 node_path_counts = fixture::i32(expected, "paths.node_path_counts");
    fixture::F64 node_path_refs = fixture::f64(expected, "paths.node_path_refs");
    fixture::I32 any_angle_counts = fixture::i32(expected, "paths.any_angle_node_path_counts");
    fixture::F64 any_angle_refs = fixture::f64(expected, "paths.any_angle_node_path_refs");
    fixture::F64 ray_t = fixture::f64(expected, "paths.ray_t");
    fixture::I32 ray_edge = fixture::i32(expected, "paths.ray_edge");
    fixture::F64 ray_normal = fixture::f64(expected, "paths.ray_normal");
    fixture::I32 ray_counts = fixture::i32(expected, "paths.ray_counts");
    fixture::F64 ray_refs = fixture::f64(expected, "paths.ray_refs");
    fixture::I32 move_success = fixture::i32(expected, "paths.move_success");
    fixture::F64 move_positions = fixture::f64(expected, "paths.move_positions");
    fixture::F64 move_refs = fixture::f64(expected, "paths.move_refs");
    fixture::I32 move_counts = fixture::i32(expected, "paths.move_counts");
    fixture::F64 move_visited = fixture::f64(expected, "paths.move_visited");
    fixture::I32 neighbourhood_counts = fixture::i32(expected, "paths.neighbourhood_counts");
    fixture::F64 neighbourhood_refs = fixture::f64(expected, "paths.neighbourhood_refs");
    fixture::I32 wall_counts = fixture::i32(expected, "paths.wall_counts");
    fixture::F64 wall_vertices = fixture::f64(expected, "paths.wall_vertices");
    fixture::F64 wall_refs = fixture::f64(expected, "paths.wall_refs");
    fixture::I32 sliced_status = fixture::i32(expected, "paths.sliced_status");
    fixture::I32 sliced_counts = fixture::i32(expected, "paths.sliced_counts");
    fixture::F64 sliced_refs = fixture::f64(expected, "paths.sliced_refs");

    ExpectedPoints straight = load_points(expected, "paths.straight");
    ExpectedPoints crossings = load_points(expected, "paths.crossings");
    ExpectedPoints smooth = load_points(expected, "paths.smooth");

    uint32_t node_path_cursor = 0;
    uint32_t any_angle_cursor = 0;
    uint32_t ray_cursor = 0;
    uint32_t move_cursor = 0;
    uint32_t neighbourhood_cursor = 0;
    uint32_t wall_cursor = 0;
    uint32_t sliced_cursor = 0;

    Tally paths = {};
    Tally node_paths = {};
    Tally straight_paths = {};
    Tally any_angle_paths = {};
    Tally crossing_paths = {};
    Tally smooth_paths = {};
    Tally rays = {};
    Tally moves = {};
    Tally neighbourhoods = {};
    Tally walls = {};
    Tally sliced_paths = {};

    uint32_t pair_count = path_success.count;
    for (uint32_t pair = 0; pair < pair_count; pair++) {
        Vec3 start = {(float)starts.data[pair * 3], (float)starts.data[pair * 3 + 1], (float)starts.data[pair * 3 + 2]};
        Vec3 end = {(float)ends.data[pair * 3], (float)ends.data[pair * 3 + 1], (float)ends.data[pair * 3 + 2]};

        FindPathResult path = find_path::find_path(nav_mesh, start, end, half_extents, filter);
        paths.record(path.success == (path_success.data[pair] != 0) && (int32_t)path.flags == path_flags.data[pair], pair);
        node_paths.record(same_refs(path.node_path.path.data, path.node_path.path.length, node_path_refs, node_path_cursor,
                                    (uint32_t)node_path_counts.data[pair]),
                          pair);
        straight_paths.record(same_points(path.path, straight, (uint32_t)straight.count.data[pair]), pair);

        FindPathResult any_angle = find_path::find_path(nav_mesh, start, end, half_extents, filter, {10});
        any_angle_paths.record(same_refs(any_angle.node_path.path.data, any_angle.node_path.path.length, any_angle_refs, any_angle_cursor,
                                         (uint32_t)any_angle_counts.data[pair]),
                               pair);
        find_path::release(any_angle);

        if (path.success) {
            FindStraightPathResult crossing = find_straight_path::find_straight_path(nav_mesh, path.start_position, path.end_position,
                                                                                     path.node_path.path.data, path.node_path.path.length,
                                                                                     UNLIMITED_POINTS, FIND_STRAIGHT_PATH_ALL_CROSSINGS);
            crossing_paths.record(same_points(crossing.path, crossings, (uint32_t)crossings.count.data[pair]), pair);
            find_straight_path::release(crossing);
        }

        FindSmoothPathResult smooth_path = find_smooth_path::find_smooth_path(nav_mesh, start, end, half_extents, filter, {0.5f, 0.01f, 256, 0});
        uint32_t smooth_count = (uint32_t)smooth.count.data[pair];
        smooth_paths.record(same_points(smooth_path.path, smooth, smooth_count), pair);
        find_smooth_path::release(smooth_path);

        NodeRef start_ref = path.start_node_ref;

        RaycastResult ray = nav_mesh_search::raycast(nav_mesh, start_ref, path.start_position, end, filter);
        bool same_ray = (ray_t.data[pair] > 1e30 ? ray.t > 1e30f : near(ray.t, ray_t.data[pair], 1e-4)) && ray.hit_edge_index == ray_edge.data[pair] &&
                        near3(ray.hit_normal, &ray_normal.data[pair * 3], 1e-4);
        same_ray = same_refs(ray.path.data, ray.path.length, ray_refs, ray_cursor, (uint32_t)ray_counts.data[pair]) && same_ray;
        rays.record(same_ray, pair);
        nav_mesh_search::release(ray);

        MoveAlongSurfaceResult move = nav_mesh_search::move_along_surface(nav_mesh, start_ref, path.start_position, end, filter);
        bool same_move = move.success == (move_success.data[pair] != 0) && near3(move.position, &move_positions.data[pair * 3], 1e-3) &&
                         move.node_ref == ref_from_f64(move_refs.data[pair]);
        same_move = same_refs(move.visited.data, move.visited.length, move_visited, move_cursor, (uint32_t)move_counts.data[pair]) && same_move;
        moves.record(same_move, pair);
        nav_mesh_search::release(move);

        FindLocalNeighbourhoodResult neighbourhood = local_neighbourhood::find_local_neighbourhood(nav_mesh, start_ref, path.start_position, 2, filter);
        neighbourhoods.record(same_refs(neighbourhood.node_refs.data, neighbourhood.node_refs.length, neighbourhood_refs, neighbourhood_cursor,
                                        (uint32_t)neighbourhood_counts.data[pair]),
                              pair);
        local_neighbourhood::release(neighbourhood);

        PolyWallSegmentsResult wall = local_neighbourhood::get_poly_wall_segments(nav_mesh, start_ref, filter, true);
        uint32_t wall_count = (uint32_t)wall_counts.data[pair];
        bool same_wall = wall.segment_refs.length == wall_count;
        for (uint32_t k = 0; same_wall && k < wall_count; k++) {
            same_wall = wall.segment_refs[k] == ref_from_f64(wall_refs.data[wall_cursor + k]);
            for (uint32_t c = 0; same_wall && c < 6; c++) same_wall = near(wall.segment_vertices[k * 6 + c], wall_vertices.data[(wall_cursor + k) * 6 + c], 1e-4);
        }
        wall_cursor += wall_count;
        walls.record(same_wall, pair);
        local_neighbourhood::release(wall);

        SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();
        nav_mesh_search::init_sliced_find_node_path(nav_mesh, query, start_ref, path.end_node_ref, path.start_position, path.end_position, filter,
                                                    SLICED_FIND_NODE_PATH_ANY_ANGLE);
        while (query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS) nav_mesh_search::update_sliced_find_node_path(nav_mesh, query, 7);
        SlicedFindNodePathResult sliced = nav_mesh_search::finalize_sliced_find_node_path(nav_mesh, query);
        bool same_sliced = (int32_t)sliced.status == sliced_status.data[pair];
        same_sliced = same_refs(sliced.path.data, sliced.path.length, sliced_refs, sliced_cursor, (uint32_t)sliced_counts.data[pair]) && same_sliced;
        sliced_paths.record(same_sliced, pair);
        nav_mesh_search::release(sliced);
        nav_mesh_search::release(query);

        find_path::release(path);
    }

    report("find_path result", paths.mismatches, paths.first, pair_count);
    report("node path", node_paths.mismatches, node_paths.first, pair_count);
    report("straight path", straight_paths.mismatches, straight_paths.first, pair_count);
    report("any angle node path", any_angle_paths.mismatches, any_angle_paths.first, pair_count);
    report("straight path with crossings", crossing_paths.mismatches, crossing_paths.first, pair_count);
    report("smooth path", smooth_paths.mismatches, smooth_paths.first, pair_count);
    report("raycast", rays.mismatches, rays.first, pair_count);
    report("move along surface", moves.mismatches, moves.first, pair_count);
    report("local neighbourhood", neighbourhoods.mismatches, neighbourhoods.first, pair_count);
    report("poly wall segments", walls.mismatches, walls.first, pair_count);
    report("sliced path", sliced_paths.mismatches, sliced_paths.first, pair_count);

    fixture::F64 random_points = fixture::f64(expected, "random.points");
    Lcg random = {99};
    Tally randoms = {};
    for (uint32_t i = 0; i < random_points.count / 4; i++) {
        FindRandomPointResult point = nav_mesh_search::find_random_point(nav_mesh, filter, next_random, &random);
        randoms.record(point.node_ref == ref_from_f64(random_points.data[i * 4]) && near3(point.position, &random_points.data[i * 4 + 1], 1e-3), i);
    }
    report("random point", randoms.mismatches, randoms.first, random_points.count / 4);

    nav_mesh_api::release_nav_mesh(nav_mesh);
    fixture::release(expected);
}
