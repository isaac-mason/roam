#include "roam/query/find_path.h"
#include "roam/query/local_neighbourhood.h"
#include "roam/query/nav_mesh_search.h"
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

TEST(search_node_pool_finds_every_node_after_growing) {
    SearchNodePool pool = {};
    for (uint32_t i = 0; i < 1000; i++) nav_mesh_search::add_search_node(pool, node_for(i * 7919ull, (int)(i % 3)));

    bool all_found = true;
    for (uint32_t i = 0; i < 1000; i++) all_found &= nav_mesh_search::get_search_node(pool, i * 7919ull, (int)(i % 3)) == (int32_t)i;
    CHECK(all_found);
    CHECK(nav_mesh_search::get_search_node(pool, 3, 0) == NULL_SEARCH_NODE);
    CHECK(nav_mesh_search::get_search_node(pool, 0, 1) == NULL_SEARCH_NODE);

    nav_mesh_search::release(pool);
}

TEST(search_node_pool_clear_keeps_memory_and_forgets_nodes) {
    SearchNodePool pool = {};
    for (uint32_t i = 0; i < 300; i++) nav_mesh_search::add_search_node(pool, node_for(i, 0));
    uint32_t node_capacity = pool.nodes.capacity;
    uint32_t bucket_count = pool.buckets.length;

    nav_mesh_search::clear(pool);
    CHECK(pool.nodes.length == 0);
    CHECK(pool.nodes.capacity == node_capacity);
    CHECK(pool.buckets.length == bucket_count);
    CHECK(nav_mesh_search::get_search_node(pool, 5, 0) == NULL_SEARCH_NODE);

    CHECK(nav_mesh_search::add_search_node(pool, node_for(5, 0)) == 0);
    CHECK(nav_mesh_search::get_search_node(pool, 5, 0) == 0);

    nav_mesh_search::release(pool);
}

TEST(search_node_pool_first_node_is_the_earliest_added_for_a_ref) {
    SearchNodePool pool = {};
    nav_mesh_search::add_search_node(pool, node_for(42, 2));
    for (uint32_t i = 0; i < 200; i++) nav_mesh_search::add_search_node(pool, node_for(i + 100, 0));
    nav_mesh_search::add_search_node(pool, node_for(42, 0));
    nav_mesh_search::add_search_node(pool, node_for(42, 1));

    CHECK(nav_mesh_search::get_first_search_node(pool, 42) == 0);
    CHECK(nav_mesh_search::get_first_search_node(pool, 7) == NULL_SEARCH_NODE);

    nav_mesh_search::release(pool);
}

TEST(search_workspace_reuse_matches_fresh_queries) {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 4, 0, 0, 4, 0, 2, 6, 0, 0, 6, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 1, 4, 5, 1, 5, 2, 4, 6, 7, 4, 7, 5};
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions, 8, indices, 18);
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;

    SearchWorkspace workspace = nav_mesh_search::create_search_workspace();
    for (int round = 0; round < 3; round++) {
        FindPathResult fresh = find_path::find_path(nav_mesh, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, {1, 1, 1}, filter);
        FindPathResult reused = find_path::find_path(nav_mesh, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, {1, 1, 1}, filter, {}, &workspace);
        REQUIRE(fresh.success && reused.success);
        CHECK(fresh.node_path.path.length == reused.node_path.path.length);
        CHECK(fresh.path.length == reused.path.length);
        CHECK(workspace.nodes.nodes.length > 0);
        find_path::release(fresh);
        find_path::release(reused);

        NodeRef start = nav_mesh_api::find_nearest_poly(nav_mesh, {0.5f, 0, 0.5f}, {1, 1, 1}, filter).node_ref;
        MoveAlongSurfaceResult fresh_move = nav_mesh_search::move_along_surface(nav_mesh, start, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, filter);
        MoveAlongSurfaceResult reused_move =
            nav_mesh_search::move_along_surface(nav_mesh, start, {0.5f, 0, 0.5f}, {5.5f, 0, 1.5f}, filter, &workspace);
        CHECK(vec3::exactly_equals(fresh_move.position, reused_move.position));
        CHECK(fresh_move.visited.length == reused_move.visited.length);
        nav_mesh_search::release(fresh_move);
        nav_mesh_search::release(reused_move);

        FindLocalNeighbourhoodResult fresh_neighbourhood = local_neighbourhood::find_local_neighbourhood(nav_mesh, start, {0.5f, 0, 0.5f}, 3, filter);
        FindLocalNeighbourhoodResult reused_neighbourhood =
            local_neighbourhood::find_local_neighbourhood(nav_mesh, start, {0.5f, 0, 0.5f}, 3, filter, &workspace);
        CHECK(fresh_neighbourhood.node_refs.length == reused_neighbourhood.node_refs.length);
        local_neighbourhood::release(fresh_neighbourhood);
        local_neighbourhood::release(reused_neighbourhood);
    }

    nav_mesh_search::release(workspace);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}
