#include <float.h>
#include <stdio.h>

#include "snippets.h"
#include "roam/query/find_path.h"
#include "roam/query/find_smooth_path.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/local_neighbourhood.h"

namespace roam {

/* SNIPPET_START: next_random */
// any generator that returns a float in [0, 1): here mulberry32, seeded
float next_random(void *user_data) {
    uint32_t &state = *static_cast<uint32_t *>(user_data);
    uint32_t t = state += 0x6d2b79f5u;
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return (float)((t ^ (t >> 14)) >> 8) / 16777216.0f;
}
/* SNIPPET_END: next_random */

static void find_path_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: find_path */
    const Vec3 start = {-8, 0, 0};
    const Vec3 end = {8, 0, 0};
    const Vec3 half_extents = {1, 1, 1};

    // nearest polys for both ends, a node path between them, and its corners, in one call
    StraightPathPoint points[64];
    NodeRef node_path[256];
    FindPathResult path = {};
    find_path::find_path(path, workspace, nav_mesh, start, end, half_extents, filter, points,
                         {.node_path = node_path});

    if (path.flags & FIND_PATH_RESULT_PARTIAL_PATH) printf("out of reach: as near as it can\n");
    if (path.flags & FIND_PATH_RESULT_MAX_POINTS_REACHED) printf("more than 64 corners\n");
    for (uint32_t i = 0; i < path.point_count; i++) {
        const Vec3 point = points[i].position;
        const bool off_mesh = points[i].type == NODE_TYPE_OFFMESH;
        printf("%.2f %.2f %.2f%s\n", point.x, point.y, point.z, off_mesh ? ", off-mesh" : "");
    }
    /* SNIPPET_END: find_path */

    /* SNIPPET_START: find_path_raycast */
    // raycast_distance: the search shortcuts through open space within this distance, where it
    // can see
    find_path::find_path(path, workspace, nav_mesh, start, end, half_extents, filter, points,
                         {.raycast_distance = 10});
    /* SNIPPET_END: find_path_raycast */
}

static void find_smooth_path_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;
    const Vec3 start = {-8, 0, 0};
    const Vec3 end = {8, 0, 0};
    const Vec3 half_extents = {1, 1, 1};

    /* SNIPPET_START: find_smooth_path */
    // the path walked in steps of 0.5 m along the surface, each point on the detail mesh. slop
    // is how near a corner counts as reached
    SmoothPathPoint points[512];
    FindSmoothPathResult path = {};
    find_smooth_path::find_smooth_path(path, workspace, nav_mesh, start, end, half_extents, filter,
                                       points, {.step_size = 0.5f, .slop = 0.01f});
    /* SNIPPET_END: find_smooth_path */
}

static void node_path_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: find_node_path */
    const Vec3 half_extents = {1, 1, 1};
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, half_extents, filter);
    FindNearestPolyResult end = {};
    nav_mesh_api::find_nearest_poly(end, nav_mesh, {8, 0, 0}, half_extents, filter);

    // the polys and off-mesh connections from start to end, A* over the nav mesh's graph
    Array<NodeRef> node_path = {};
    node_path.reserve(4);
    FindNodePathResult path = {};
    nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref,
                                    start.position, end.position, filter, node_path);

    // path_total is its true length: grow and copy it out of the workspace, without searching
    // again
    if (path.truncated) {
        node_path.reserve(path.path_total);
        nav_mesh_search::copy_last_node_path(path, workspace, node_path);
    }
    node_path.resize(path.path_count);
    /* SNIPPET_END: find_node_path */

    /* SNIPPET_START: find_straight_path */
    // the node path string-pulled into corners. AREA_CROSSINGS adds a point where the path
    // changes area, and ALL_CROSSINGS at every poly edge
    StraightPathPoint points[64];
    FindStraightPathResult straight = {};
    find_straight_path::find_straight_path(straight, nav_mesh, start.position, end.position,
                                           node_path, points, FIND_STRAIGHT_PATH_AREA_CROSSINGS);
    /* SNIPPET_END: find_straight_path */

    node_path.release();
}

static void sliced_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    const QueryFilter &filter = scene.filter;
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);
    FindNearestPolyResult end = {};
    nav_mesh_api::find_nearest_poly(end, nav_mesh, {8, 0, 0}, {1, 1, 1}, filter);

    /* SNIPPET_START: sliced */
    // a search spread over frames: a few iterations at a time until it is no longer in progress
    SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();
    nav_mesh_search::init_sliced_find_node_path(query, nav_mesh, start.node_ref, end.node_ref,
                                                start.position, end.position, filter);
    while (query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS) {
        nav_mesh_search::update_sliced_find_node_path(query, nav_mesh, 10);
    }

    NodeRef node_path[256];
    SlicedFindNodePathResult path = {};
    nav_mesh_search::finalize_sliced_find_node_path(path, nav_mesh, query, node_path);
    nav_mesh_search::release(query);
    /* SNIPPET_END: sliced */
}

static void surface_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: move_along_surface */
    // a character controller's step: from where it is towards where its input would take it,
    // sliding along walls and staying on the nav mesh. keep its poly ref between frames
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-4, 0, 0}, {1, 1, 1}, filter);
    const Vec3 target = {0, 0, 0.5f};

    NodeRef visited[16];
    MoveAlongSurfaceResult move = {};
    nav_mesh_search::move_along_surface(move, workspace, nav_mesh, start.node_ref, start.position,
                                        target, filter, visited);
    if (move.success) {
        // the move is in 2D: the height is the detail mesh's under the new position
        GetPolyHeightResult height;
        nav_mesh_api::get_poly_height(height, nav_mesh, move.node_ref, move.position);
        Vec3 position = move.position;
        if (height.success) position.y = height.height;
        printf("now at %.2f %.2f %.2f\n", position.x, position.y, position.z);
    }
    /* SNIPPET_END: move_along_surface */
}

static void raycast_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    const QueryFilter &filter = scene.filter;
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);

    /* SNIPPET_START: raycast */
    // along the surface from a point on a poly, towards another: can an agent walk straight there?
    const Vec3 end = {8, 0, 0};
    NodeRef visited[64];
    RaycastResult ray = {};
    nav_mesh_search::raycast(ray, nav_mesh, start.node_ref, start.position, end, filter, visited);
    if (ray.t == FLT_MAX) {
        printf("clear all the way\n");
    } else {
        const Vec3 hit = vec3::lerp(start.position, end, ray.t);
        const Vec3 normal = ray.hit_normal;
        printf("blocked at %.2f %.2f, by a wall facing %.2f %.2f\n", hit.x, hit.z, normal.x, normal.z);
    }
    /* SNIPPET_END: raycast */

    /* SNIPPET_START: raycast_with_costs */
    // the same, adding up the filter's cost of the polys crossed. pass the poly you came from for
    // accurate costs
    const NodeRef previous = INVALID_NODE_REF;
    nav_mesh_search::raycast_with_costs(ray, nav_mesh, start.node_ref, start.position, {-8, 0, 8},
                                        filter, previous, visited);
    printf("costs %.2f\n", ray.path_cost);
    /* SNIPPET_END: raycast_with_costs */
}

static void nearest_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: find_nearest_poly */
    // the poly nearest a point, within a box of these half extents around it, and the nearest
    // point on it
    const Vec3 position = {-6, 1, 3};
    const Vec3 half_extents = {1, 2, 1};
    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, position, half_extents, filter);
    if (nearest.success) {
        const Vec3 at = nearest.position;
        printf("poly %llu at %.2f %.2f %.2f\n", (unsigned long long)nearest.node_ref, at.x, at.y, at.z);
    }
    /* SNIPPET_END: find_nearest_poly */
}

static void random_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: random_points */
    uint32_t seed = 42;

    // anywhere on the nav mesh, polys weighted by area
    FindRandomPointResult anywhere = {};
    nav_mesh_search::find_random_point(anywhere, nav_mesh, filter, next_random, &seed);

    // within 3 m of a point, through polys connected to its own
    FindNearestPolyResult centre = {};
    nav_mesh_api::find_nearest_poly(centre, nav_mesh, {-6, 0, -6}, {1, 1, 1}, filter);
    FindRandomPointResult nearby = {};
    nav_mesh_search::find_random_point_around_circle(nearby, workspace, nav_mesh, centre.node_ref,
                                                     centre.position, 3, filter, next_random, &seed);
    /* SNIPPET_END: random_points */
}

static void reachability_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: is_reachable */
    // the floor and the top of the platform are islands of their own until a connection joins
    // them
    FindNearestPolyResult floor = {};
    nav_mesh_api::find_nearest_poly(floor, nav_mesh, {6, 0, -2}, {1, 1, 1}, filter);
    FindNearestPolyResult platform = {};
    nav_mesh_api::find_nearest_poly(platform, nav_mesh, {6.5f, 1, -7}, {1, 1, 1}, filter);
    if (!nav_mesh_search::is_reachable(workspace, nav_mesh, floor.node_ref, platform.node_ref)) {
        printf("no way up\n");
    }
    /* SNIPPET_END: is_reachable */
}

static void neighbourhood_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-4, 0, 0}, {1, 1, 1}, filter);

    /* SNIPPET_START: find_local_neighbourhood */
    // the polys within 2 m of a point that connect to its own without overlapping each other,
    // and their walls
    NodeRef polys[32];
    FindLocalNeighbourhoodResult neighbourhood = {};
    local_neighbourhood::find_local_neighbourhood(neighbourhood, workspace, nav_mesh, start.node_ref,
                                                  start.position, 2, filter, polys);
    for (uint32_t i = 0; i < neighbourhood.poly_count; i++) {
        WallSegment walls[16];
        PolyWallSegmentsResult segments = {};
        local_neighbourhood::get_poly_wall_segments(segments, nav_mesh, polys[i], filter, false, walls);
        printf("poly %llu: %u walls\n", (unsigned long long)polys[i], segments.segment_count);
    }
    /* SNIPPET_END: find_local_neighbourhood */
}

void run_query_snippets(Scene &scene) {
    find_path_snippets(scene);
    find_smooth_path_snippets(scene);
    node_path_snippets(scene);
    sliced_snippets(scene);
    surface_snippets(scene);
    raycast_snippets(scene);
    nearest_snippets(scene);
    random_snippets(scene);
    reachability_snippets(scene);
    neighbourhood_snippets(scene);
}

}
