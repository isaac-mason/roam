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

static bool same_points(const StraightPathPoint *actual, uint32_t actual_count, ExpectedPoints &expected, uint32_t expected_count) {
    bool same = actual_count == expected_count;
    for (uint32_t i = 0; same && i < actual_count; i++) {
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

static bool same_end_point(const StraightPathPoint *actual, uint32_t actual_count, ExpectedPoints &expected, uint32_t expected_count) {
    bool same = (actual_count == 0) == (expected_count == 0);
    if (same && actual_count > 0) {
        const StraightPathPoint &point = actual[actual_count - 1];
        uint32_t at = expected.cursor + expected_count - 1;
        same = near3(point.position, &expected.positions.data[at * 3], 1e-3) && point.node_ref == ref_from_f64(expected.refs.data[at]) &&
               (int32_t)point.flags == expected.flags.data[at];
    }
    expected.cursor += expected_count;
    return same;
}

static void report(const char *name, uint32_t mismatches, uint32_t first, uint32_t total, uint32_t allowed_ties = 0) {
    if (mismatches <= allowed_ties) return;
    char message[256];
    snprintf(message, sizeof(message), "%s: %u of %u differ (%u ties allowed), first at pair %u", name, mismatches, total, allowed_ties, first);
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

    const uint32_t MAX_REFS = 4096;
    const uint32_t MAX_POINTS = 4096;
    Array<StraightPathPoint> points = {};
    Array<StraightPathPoint> other_points = {};
    Array<NodeRef> node_path = {};
    Array<NodeRef> other_refs = {};
    Array<WallSegment> segments = {};
    points.resize(MAX_POINTS);
    other_points.resize(MAX_POINTS);
    node_path.resize(MAX_REFS);
    other_refs.resize(MAX_REFS);
    segments.resize(MAX_REFS);
    SearchWorkspace workspace = nav_mesh_search::create_search_workspace();
    SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();

    uint32_t pair_count = path_success.count;
    for (uint32_t pair = 0; pair < pair_count; pair++) {
        Vec3 start = {(float)starts.data[pair * 3], (float)starts.data[pair * 3 + 1], (float)starts.data[pair * 3 + 2]};
        Vec3 end = {(float)ends.data[pair * 3], (float)ends.data[pair * 3 + 1], (float)ends.data[pair * 3 + 2]};

        FindPathResult path;
        find_path::find_path(path, workspace, nav_mesh, start, end, half_extents, filter, points,
                             {.node_path = node_path});
        paths.record(path.success == (path_success.data[pair] != 0) && (int32_t)path.flags == path_flags.data[pair], pair);
        node_paths.record(same_refs(node_path.data, path.node_path_count, node_path_refs, node_path_cursor, (uint32_t)node_path_counts.data[pair]), pair);
        straight_paths.record(same_points(points.data, path.point_count, straight, (uint32_t)straight.count.data[pair]), pair);

        FindPathResult any_angle;
        find_path::find_path(any_angle, workspace, nav_mesh, start, end, half_extents, filter, other_points,
                             {.raycast_distance = 10, .node_path = other_refs});
        any_angle_paths.record(
            same_refs(other_refs.data, any_angle.node_path_count, any_angle_refs, any_angle_cursor, (uint32_t)any_angle_counts.data[pair]), pair);

        if (path.success) {
            FindStraightPathResult crossing;
            find_straight_path::find_straight_path(crossing, nav_mesh, path.start_position, path.end_position, {node_path.data, path.node_path_count},
                                                   other_points, FIND_STRAIGHT_PATH_ALL_CROSSINGS);
            crossing_paths.record(same_points(other_points.data, crossing.point_count, crossings, (uint32_t)crossings.count.data[pair]), pair);
        }

        FindSmoothPathResult smooth_path;
        find_smooth_path::find_smooth_path(smooth_path, workspace, nav_mesh, start, end, half_extents, filter, other_points,
                                           {.step_size = 0.5f, .slop = 0.01f});
        smooth_paths.record(
            smooth_path.success && same_end_point(other_points.data, smooth_path.point_count, smooth, (uint32_t)smooth.count.data[pair]), pair);

        NodeRef start_ref = path.start_node_ref;

        RaycastResult ray;
        nav_mesh_search::raycast(ray, nav_mesh, start_ref, path.start_position, end, filter, other_refs);
        bool same_ray = (ray_t.data[pair] > 1e30 ? ray.t > 1e30f : near(ray.t, ray_t.data[pair], 1e-4)) && ray.hit_edge_index == ray_edge.data[pair] &&
                        near3(ray.hit_normal, &ray_normal.data[pair * 3], 1e-4);
        same_ray = same_refs(other_refs.data, ray.path_count, ray_refs, ray_cursor, (uint32_t)ray_counts.data[pair]) && same_ray;
        rays.record(same_ray, pair);

        MoveAlongSurfaceResult move;
        nav_mesh_search::move_along_surface(move, workspace, nav_mesh, start_ref, path.start_position, end, filter, other_refs);
        bool same_move = move.success == (move_success.data[pair] != 0) && near3(move.position, &move_positions.data[pair * 3], 1e-3) &&
                         move.node_ref == ref_from_f64(move_refs.data[pair]);
        same_move = same_refs(other_refs.data, move.visited_count, move_visited, move_cursor, (uint32_t)move_counts.data[pair]) && same_move;
        moves.record(same_move, pair);

        FindLocalNeighbourhoodResult neighbourhood;
        local_neighbourhood::find_local_neighbourhood(neighbourhood, workspace, nav_mesh, start_ref, path.start_position, 2, filter, other_refs);
        neighbourhoods.record(same_refs(other_refs.data, neighbourhood.poly_count, neighbourhood_refs, neighbourhood_cursor,
                                        (uint32_t)neighbourhood_counts.data[pair]),
                              pair);

        PolyWallSegmentsResult wall;
        local_neighbourhood::get_poly_wall_segments(wall, nav_mesh, start_ref, filter, true, segments, other_refs);
        uint32_t wall_count = (uint32_t)wall_counts.data[pair];
        bool same_wall = wall.segment_count == wall_count;
        for (uint32_t k = 0; same_wall && k < wall_count; k++) {
            same_wall = other_refs[k] == ref_from_f64(wall_refs.data[wall_cursor + k]);
            same_wall = same_wall && near3(segments[k].start, &wall_vertices.data[(wall_cursor + k) * 6], 1e-4) &&
                        near3(segments[k].end, &wall_vertices.data[(wall_cursor + k) * 6 + 3], 1e-4);
        }
        wall_cursor += wall_count;
        walls.record(same_wall, pair);

        nav_mesh_search::init_sliced_find_node_path(query, nav_mesh, start_ref, path.end_node_ref, path.start_position, path.end_position, filter,
                                                    SLICED_FIND_NODE_PATH_ANY_ANGLE);
        while (query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS) nav_mesh_search::update_sliced_find_node_path(query, nav_mesh, 7);
        SlicedFindNodePathResult sliced;
        nav_mesh_search::finalize_sliced_find_node_path(sliced, nav_mesh, query, other_refs);
        bool same_sliced = (int32_t)sliced.status == sliced_status.data[pair];
        same_sliced = same_refs(other_refs.data, sliced.path_count, sliced_refs, sliced_cursor, (uint32_t)sliced_counts.data[pair]) && same_sliced;
        sliced_paths.record(same_sliced, pair);
    }

    nav_mesh_search::release(query);
    nav_mesh_search::release(workspace);
    points.release();
    other_points.release();
    node_path.release();
    other_refs.release();
    segments.release();

    uint32_t allowed_ties = pair_count / 20;
    report("find_path result", paths.mismatches, paths.first, pair_count);
    report("node path", node_paths.mismatches, node_paths.first, pair_count);
    report("straight path", straight_paths.mismatches, straight_paths.first, pair_count, allowed_ties);
    report("any angle node path", any_angle_paths.mismatches, any_angle_paths.first, pair_count, allowed_ties);
    report("straight path with crossings", crossing_paths.mismatches, crossing_paths.first, pair_count, allowed_ties);
    report("smooth path", smooth_paths.mismatches, smooth_paths.first, pair_count, allowed_ties);
    report("raycast", rays.mismatches, rays.first, pair_count);
    report("move along surface", moves.mismatches, moves.first, pair_count);
    report("local neighbourhood", neighbourhoods.mismatches, neighbourhoods.first, pair_count);
    report("poly wall segments", walls.mismatches, walls.first, pair_count);
    report("sliced path", sliced_paths.mismatches, sliced_paths.first, pair_count, allowed_ties);

    fixture::F64 random_points = fixture::f64(expected, "random.points");
    Lcg random = {99};
    Tally randoms = {};
    for (uint32_t i = 0; i < random_points.count / 4; i++) {
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, nav_mesh, filter, next_random, &random);
        randoms.record(point.node_ref == ref_from_f64(random_points.data[i * 4]) && near3(point.position, &random_points.data[i * 4 + 1], 1e-3), i);
    }
    report("random point", randoms.mismatches, randoms.first, random_points.count / 4);

    nav_mesh_api::release(nav_mesh);
    fixture::release(expected);
}
