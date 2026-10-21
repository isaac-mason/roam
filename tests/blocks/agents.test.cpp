#include "roam/blocks/agents/crowd.h"
#include "roam/blocks/agents/path_corridor.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

using namespace roam;

#define COUNT_OF(values) (uint32_t)(sizeof(values) / sizeof(values[0]))

#define REFS(name, ...)                         \
    const NodeRef name##_values[] = {__VA_ARGS__}; \
    Array<NodeRef> name = refs(name##_values, COUNT_OF(name##_values))

#define CHECK_REFS(actual, ...)                                    \
    do {                                                           \
        const NodeRef expected_refs[] = {__VA_ARGS__};             \
        CHECK(equals(actual, expected_refs, COUNT_OF(expected_refs))); \
    } while (0)

static Array<NodeRef> refs(const NodeRef *values, uint32_t count) {
    Array<NodeRef> array = {};
    for (uint32_t i = 0; i < count; i++) array.push(values[i]);
    return array;
}

static bool equals(const Array<NodeRef> &actual, const NodeRef *expected, uint32_t count) {
    if (actual.length != count) return false;
    for (uint32_t i = 0; i < count; i++) {
        if (actual[i] != expected[i]) return false;
    }
    return true;
}

TEST(path_corridor_merge_start_moved_prepends_visited_in_reverse) {
    REFS(path, 1, 2, 3, 4);
    REFS(visited, 1, 7, 2);

    path_corridor::merge_start_moved(path, visited);
    CHECK_REFS(path, 2, 3, 4);

    REFS(wandered, 2, 8);
    path_corridor::merge_start_moved(path, wandered);
    CHECK_REFS(path, 8, 2, 3, 4);

    wandered.release();
    visited.release();
    path.release();
}

TEST(path_corridor_merge_start_shortcut_replaces_the_start) {
    REFS(path, 1, 2, 3, 4, 5);
    REFS(shortcut, 1, 9, 4);

    path_corridor::merge_start_shortcut(path, shortcut);
    CHECK_REFS(path, 1, 9, 4, 5);

    REFS(unrelated, 6, 7);
    path_corridor::merge_start_shortcut(path, unrelated);
    CHECK_REFS(path, 1, 9, 4, 5);

    unrelated.release();
    shortcut.release();
    path.release();
}

TEST(path_corridor_fix_path_start_keeps_the_end) {
    PathCorridor corridor = path_corridor::create();
    path_corridor::reset(corridor, 5, {0, 0, 0});

    path_corridor::fix_path_start(corridor, 9, {1, 0, 1});
    CHECK_REFS(corridor.path, 9, INVALID_NODE_REF, 5);
    CHECK(corridor.position.x == 1);

    NodeRef long_path[] = {1, 2, 3, 4};
    path_corridor::set_path(corridor, {0, 0, 0}, long_path, 4);
    path_corridor::fix_path_start(corridor, 9, {1, 0, 1});
    CHECK_REFS(corridor.path, 9, INVALID_NODE_REF, 3, 4);

    path_corridor::release(corridor);
}

static NavMesh create_two_islands() {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 7, 0, 0, 9, 0, 0, 9, 0, 2, 7, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions, 8, indices, 12);
    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1.5f, 0, 1}, {7.5f, 0, 1}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});
    return nav_mesh;
}

TEST(path_corridor_move_over_off_mesh_connection_lands_on_the_far_side) {
    NavMesh nav_mesh = create_two_islands();
    const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[0];

    PathCorridor corridor = path_corridor::create();
    NodeRef path[] = {attachment.start_poly_node, attachment.off_mesh_node, attachment.end_poly_node};
    path_corridor::reset(corridor, path[0], {1.5f, 0, 1});
    path_corridor::set_path(corridor, {8, 0, 1}, path, 3);

    MoveOverOffMeshConnectionResult result = {};
    path_corridor::move_over_off_mesh_connection(result, corridor, attachment.off_mesh_node, nav_mesh);
    REQUIRE(result.success);
    CHECK(result.end_node_ref == attachment.end_poly_node);
    CHECK(result.end_position.x == 7.5f);
    CHECK_REFS(corridor.path, attachment.end_poly_node);
    CHECK(corridor.position.x == 7.5f);

    path_corridor::release(corridor);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}

static AgentParams test_agent_params() {
    return {
        .radius = 0.2f,
        .height = 1,
        .max_acceleration = 20,
        .max_speed = 3,
        .collision_query_range = 1,
        .separation_weight = 0,
        .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS,
    };
}

TEST(crowd_reuses_removed_agent_ids) {
    NavMesh nav_mesh = create_two_islands();
    Crowd crowd = crowd::create(0.5f);

    uint32_t first = crowd::add_agent(crowd, nav_mesh, {1, 0, 1}, test_agent_params());
    uint32_t second = crowd::add_agent(crowd, nav_mesh, {1.5f, 0, 1.5f}, test_agent_params());
    CHECK(first != second);
    CHECK(crowd.agents[first].state == AGENT_STATE_WALKING);

    CHECK(crowd::remove_agent(crowd, first));
    CHECK(!crowd::is_agent(crowd, first));
    CHECK(!crowd::remove_agent(crowd, first));
    CHECK(!crowd::request_move_velocity(crowd, first, {1, 0, 0}));

    uint32_t third = crowd::add_agent(crowd, nav_mesh, {1, 0, 1}, test_agent_params());
    CHECK(third == first);

    uint32_t lost = crowd::add_agent(crowd, nav_mesh, {50, 0, 50}, test_agent_params());
    CHECK(crowd.agents[lost].state == AGENT_STATE_INVALID);

    crowd::release(crowd);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}

TEST(crowd_agent_crosses_an_off_mesh_connection_to_its_target) {
    NavMesh nav_mesh = create_two_islands();
    Crowd crowd = crowd::create(0.5f);

    uint32_t agent_id = crowd::add_agent(crowd, nav_mesh, {0.5f, 0, 1}, test_agent_params());
    FindNearestPolyResult target = {};
    nav_mesh_api::find_nearest_poly(target, nav_mesh, {8.5f, 0, 1}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    REQUIRE(target.success);
    CHECK(crowd::request_move_target(crowd, agent_id, target.node_ref, target.position));

    bool went_off_mesh = false;
    bool arrived = false;
    for (int frame = 0; frame < 600 && !arrived; frame++) {
        crowd::update(crowd, nav_mesh, 1.0f / 60.0f);
        went_off_mesh |= crowd.agents[agent_id].state == AGENT_STATE_OFFMESH;
        arrived = crowd::is_agent_at_target(crowd, agent_id, 0.1f);
    }

    CHECK(went_off_mesh);
    CHECK(arrived);
    CHECK(crowd.agents[agent_id].position.x > 7);

    crowd::release(crowd);
    nav_mesh_api::release_nav_mesh(nav_mesh);
}

static NavMesh create_open_floor() {
    const float positions[] = {0, 0, 0, 10, 0, 0, 10, 0, 10, 0, 0, 10};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3};
    return create_nav_mesh_from_triangles(positions, 4, indices, 6);
}

TEST(crowd_max_neighbours_keeps_the_nearest) {
    NavMesh nav_mesh = create_open_floor();

    for (uint32_t cap = 0; cap <= 2; cap += 2) {
        Crowd crowd = crowd::create(0.5f);
        crowd.max_neighbours = cap;
        AgentParams params = test_agent_params();
        params.collision_query_range = 5;

        uint32_t center = crowd::add_agent(crowd, nav_mesh, {5, 0, 5}, params);
        const float offsets[] = {2.0f, 0.5f, 1.5f, 1.0f};
        for (float offset : offsets) crowd::add_agent(crowd, nav_mesh, {5 + offset, 0, 5}, params);

        crowd::update(crowd, nav_mesh, 0.0f);

        const Array<AgentNeighbour> &neis = crowd.agents[center].neis;
        if (cap == 0) {
            CHECK(neis.length == 4);
        } else {
            REQUIRE(neis.length == 2);
            CHECK_NEAR(neis[0].distance_sqr, 0.25f, 1e-4f);
            CHECK_NEAR(neis[1].distance_sqr, 1.0f, 1e-4f);
        }

        crowd::release(crowd);
    }

    nav_mesh_api::release_nav_mesh(nav_mesh);
}
