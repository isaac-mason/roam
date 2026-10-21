#include "roam/query/find_path.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/local_neighbourhood.h"
#include "roam/query/nav_mesh_search.h"
#include "roam/util/memory.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

using namespace roam;

static SearchNode node_for(NodeRef ref, int state) {
    SearchNode node = {};
    node.node_ref = ref;
    node.state = state;
    node.parent_node_ref = INVALID_NODE_REF;
    node.parent_state = NO_PARENT_STATE;
    return node;
}

static NodeRef poly_ref(uint32_t index) { return node::ser_node_ref(NODE_TYPE_POLY, index, 1); }

TEST(search_node_pool_finds_every_node_after_growing) {
    SearchNodePool pool = {};
    for (uint32_t i = 0; i < 1000; i++) nav_mesh_search::add_search_node(pool, node_for(poly_ref(i * 37), (int)(i % 3)));

    bool all_found = true;
    for (uint32_t i = 0; i < 1000; i++) all_found &= nav_mesh_search::get_search_node(pool, poly_ref(i * 37), (int)(i % 3)) == (int32_t)i;
    CHECK(all_found);
    CHECK(nav_mesh_search::get_search_node(pool, poly_ref(3), 0) == NULL_SEARCH_NODE);
    CHECK(nav_mesh_search::get_search_node(pool, poly_ref(0), 1) == NULL_SEARCH_NODE);

    nav_mesh_search::release(pool);
}

TEST(search_node_pool_clear_keeps_memory_and_forgets_nodes) {
    SearchNodePool pool = {};
    for (uint32_t i = 0; i < 300; i++) nav_mesh_search::add_search_node(pool, node_for(poly_ref(i), 0));
    uint32_t node_capacity = pool.nodes.capacity;
    uint32_t table_length = pool.first_by_index.length;

    nav_mesh_search::clear(pool);
    CHECK(pool.nodes.length == 0);
    CHECK(pool.nodes.capacity == node_capacity);
    CHECK(pool.first_by_index.length == table_length);
    CHECK(nav_mesh_search::get_search_node(pool, poly_ref(5), 0) == NULL_SEARCH_NODE);

    CHECK(nav_mesh_search::add_search_node(pool, node_for(poly_ref(5), 0)) == 0);
    CHECK(nav_mesh_search::get_search_node(pool, poly_ref(5), 0) == 0);

    nav_mesh_search::release(pool);
}

TEST(search_node_pool_first_node_is_the_earliest_added_for_a_ref) {
    SearchNodePool pool = {};
    nav_mesh_search::add_search_node(pool, node_for(poly_ref(42), 2));
    for (uint32_t i = 0; i < 200; i++) nav_mesh_search::add_search_node(pool, node_for(poly_ref(i + 100), 0));
    nav_mesh_search::add_search_node(pool, node_for(poly_ref(42), 0));
    nav_mesh_search::add_search_node(pool, node_for(poly_ref(42), 1));

    CHECK(nav_mesh_search::get_first_search_node(pool, poly_ref(42)) == 0);
    CHECK(nav_mesh_search::get_first_search_node(pool, poly_ref(7)) == NULL_SEARCH_NODE);

    nav_mesh_search::release(pool);
}

static NavMesh create_strip_nav_mesh() {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 4, 0, 0, 4, 0, 2, 6, 0, 0, 6, 0, 2};
    const uint32_t indices[] = {0, 2, 1, 0, 3, 2, 1, 5, 4, 1, 2, 5, 4, 7, 6, 4, 5, 7};
    return create_nav_mesh_from_triangles(positions, 8, indices, 18);
}

static const uint32_t MAX_REFS = 64;
static const uint32_t MAX_POINTS = 64;

TEST(search_workspace_reuse_matches_fresh_queries) {
    NavMesh nav_mesh = create_strip_nav_mesh();
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;

    SearchWorkspace reused = nav_mesh_search::create_search_workspace();
    for (int round = 0; round < 3; round++) {
        SearchWorkspace fresh = nav_mesh_search::create_search_workspace();

        StraightPathPoint fresh_points[MAX_POINTS];
        StraightPathPoint reused_points[MAX_POINTS];
        NodeRef fresh_refs[MAX_REFS];
        NodeRef reused_refs[MAX_REFS];
        FindPathResult fresh_path;
        find_path::find_path(fresh_path, fresh, nav_mesh, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, {1, 1, 1}, filter, fresh_points,
                             {.node_path = fresh_refs});
        FindPathResult reused_path;
        find_path::find_path(reused_path, reused, nav_mesh, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, {1, 1, 1}, filter, reused_points,
                             {.node_path = reused_refs});
        REQUIRE(fresh_path.success && reused_path.success);
        CHECK(fresh_path.node_path_count == reused_path.node_path_count);
        CHECK(fresh_path.point_count == reused_path.point_count);
        for (uint32_t i = 0; i < fresh_path.node_path_count; i++) CHECK(fresh_refs[i] == reused_refs[i]);

        FindNearestPolyResult nearest;
        nav_mesh_api::find_nearest_poly(nearest, nav_mesh, {0.5f, 0, 0.5f}, {1, 1, 1}, filter);
        NodeRef start = nearest.node_ref;
        MoveAlongSurfaceResult fresh_move;
        nav_mesh_search::move_along_surface(fresh_move, fresh, nav_mesh, start, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, filter, fresh_refs);
        MoveAlongSurfaceResult reused_move;
        nav_mesh_search::move_along_surface(reused_move, reused, nav_mesh, start, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, filter, reused_refs);
        CHECK(vec3::exactly_equals(fresh_move.position, reused_move.position));
        CHECK(fresh_move.visited_count == reused_move.visited_count);

        FindLocalNeighbourhoodResult fresh_neighbourhood;
        local_neighbourhood::find_local_neighbourhood(fresh_neighbourhood, fresh, nav_mesh, start, {0.5f, 0, 0.5f}, 3, filter, fresh_refs);
        FindLocalNeighbourhoodResult reused_neighbourhood;
        local_neighbourhood::find_local_neighbourhood(reused_neighbourhood, reused, nav_mesh, start, {0.5f, 0, 0.5f}, 3, filter, reused_refs);
        CHECK(fresh_neighbourhood.poly_count == reused_neighbourhood.poly_count);

        nav_mesh_search::release(fresh);
    }

    nav_mesh_search::release(reused);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}

TEST(small_buffers_keep_the_start_of_the_path_and_report_its_full_length) {
    NavMesh nav_mesh = create_strip_nav_mesh();
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    SearchWorkspace workspace = nav_mesh_search::create_search_workspace();

    FindNearestPolyResult start;
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {1.5f, 0, 0.5f}, {1, 1, 1}, filter);
    FindNearestPolyResult end;
    nav_mesh_api::find_nearest_poly(end, nav_mesh, {5.5f, 0, 1.5f}, {1, 1, 1}, filter);

    NodeRef full[MAX_REFS];
    FindNodePathResult full_path;
    nav_mesh_search::find_node_path(full_path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, full);
    REQUIRE(full_path.success);
    REQUIRE(full_path.path_count > 2);
    CHECK(full_path.path_total == full_path.path_count);
    CHECK(!full_path.truncated);

    NodeRef truncated[2];
    FindNodePathResult truncated_path;
    nav_mesh_search::find_node_path(truncated_path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, truncated);
    CHECK(truncated_path.success);
    CHECK(truncated_path.path_count == 2);
    CHECK(truncated_path.path_total == full_path.path_count);
    CHECK(truncated_path.truncated);
    CHECK(truncated[0] == full[0]);
    CHECK(truncated[1] == full[1]);

    NodeRef copied[MAX_REFS];
    FindNodePathResult copied_path = truncated_path;
    nav_mesh_search::copy_last_node_path(copied_path, workspace, copied);
    CHECK(!copied_path.truncated);
    REQUIRE(copied_path.path_count == full_path.path_count);
    bool same_path = true;
    for (uint32_t i = 0; i < full_path.path_count; i++) same_path = same_path && copied[i] == full[i];
    CHECK(same_path);

    FindNodePathResult counted;
    nav_mesh_search::find_node_path(counted, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, {});
    CHECK(counted.path_count == 0);
    CHECK(counted.path_total == full_path.path_count);

    NodeRef ray_refs[1];
    RaycastResult ray;
    nav_mesh_search::raycast(ray, nav_mesh, start.node_ref, start.position, end.position, filter, ray_refs);
    CHECK(ray.path_count == 1);
    CHECK(ray.path_total > 1);
    CHECK(ray.truncated);
    CHECK(ray_refs[0] == start.node_ref);

    StraightPathPoint all_points[MAX_POINTS];
    FindStraightPathResult all;
    find_straight_path::find_straight_path(all, nav_mesh, start.position, end.position, {full, full_path.path_count}, all_points);
    REQUIRE(all.success);
    CHECK(!all.truncated);

    Array<StraightPathPoint> exact = {};
    exact.resize(all.point_count);
    FindStraightPathResult exact_fit;
    find_straight_path::find_straight_path(exact_fit, nav_mesh, start.position, end.position, {full, full_path.path_count}, exact);
    CHECK(exact_fit.point_count == all.point_count);
    CHECK(!exact_fit.truncated);

    exact.resize(all.point_count - 1);
    Span<StraightPathPoint> short_buffer = {exact.data, exact.length};
    FindStraightPathResult cut;
    find_straight_path::find_straight_path(cut, nav_mesh, start.position, end.position, {full, full_path.path_count}, short_buffer);
    CHECK(cut.point_count == all.point_count - 1);
    CHECK(cut.truncated);
    exact.release();

    uint32_t seed = 7;
    FindRandomPointResult random_point;
    nav_mesh_search::find_random_point_around_circle(random_point, workspace, nav_mesh, start.node_ref, start.position, 2, filter,
                                                     [](void *user_data) {
                                                         uint32_t &state = *(uint32_t *)user_data;
                                                         state = state * 1664525u + 1013904223u;
                                                         return (float)(state >> 8) / (float)(1u << 24);
                                                     },
                                                     &seed);
    FindNodePathResult after_other_search;
    nav_mesh_search::copy_last_node_path(after_other_search, workspace, copied);
    CHECK(after_other_search.path_total == 0);

    nav_mesh_search::release(workspace);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}

TEST(sliced_finalize_can_be_repeated_to_grow_the_buffer) {
    NavMesh nav_mesh = create_strip_nav_mesh();
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;

    FindNearestPolyResult start;
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {1.5f, 0, 0.5f}, {1, 1, 1}, filter);
    FindNearestPolyResult end;
    nav_mesh_api::find_nearest_poly(end, nav_mesh, {5.5f, 0, 1.5f}, {1, 1, 1}, filter);

    SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();
    nav_mesh_search::init_sliced_find_node_path(nav_mesh, query, start.node_ref, end.node_ref, start.position, end.position, filter);
    while (query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS) nav_mesh_search::update_sliced_find_node_path(nav_mesh, query, 2);

    NodeRef first[1];
    SlicedFindNodePathResult cut;
    nav_mesh_search::finalize_sliced_find_node_path(cut, nav_mesh, query, first);
    CHECK(cut.truncated);
    REQUIRE(cut.path_total > 1);

    NodeRef all[MAX_REFS];
    SlicedFindNodePathResult again;
    nav_mesh_search::finalize_sliced_find_node_path(again, nav_mesh, query, all);
    CHECK(again.status == cut.status);
    CHECK(!again.truncated);
    CHECK(again.path_count == cut.path_total);
    CHECK(all[0] == first[0]);
    CHECK(all[again.path_count - 1] == end.node_ref);

    nav_mesh_search::release(query);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}

TEST(warm_queries_do_not_allocate) {
    if (!memory::tracking_enabled()) return;

    NavMesh nav_mesh = create_strip_nav_mesh();
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    SearchWorkspace workspace = nav_mesh_search::create_search_workspace();
    StraightPathPoint points[MAX_POINTS];
    NodeRef refs[MAX_REFS];
    int64_t allocations_after_warm_up = 0;

    for (int round = 0; round < 2; round++) {
        if (round == 1) allocations_after_warm_up = memory::outstanding().total_allocations;

        FindPathResult path;
        find_path::find_path(path, workspace, nav_mesh, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, {1, 1, 1}, filter, points);
        CHECK(path.success);

        FindNearestPolyResult nearest;
        nav_mesh_api::find_nearest_poly(nearest, nav_mesh, {0.5f, 0, 0.5f}, {1, 1, 1}, filter);

        RaycastResult ray;
        nav_mesh_search::raycast(ray, nav_mesh, nearest.node_ref, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, filter, refs);

        MoveAlongSurfaceResult moved;
        nav_mesh_search::move_along_surface(moved, workspace, nav_mesh, nearest.node_ref, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, filter, refs);

        FindLocalNeighbourhoodResult neighbourhood;
        local_neighbourhood::find_local_neighbourhood(neighbourhood, workspace, nav_mesh, nearest.node_ref, {0.5f, 0, 0.5f}, 3, filter, refs);

        WallSegment segments[MAX_REFS];
        PolyWallSegmentsResult walls;
        local_neighbourhood::get_poly_wall_segments(walls, nav_mesh, nearest.node_ref, filter, true, segments, refs);
    }

    CHECK(memory::outstanding().total_allocations == allocations_after_warm_up);

    nav_mesh_search::release(workspace);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}
