#include "app/examples.h"
#include "imgui.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/blocks/search/flood_fill_nav_mesh.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

static const Color AGENT_COLORS[] = {
    {0, 0, 1, 1}, {0, 1, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {1, 0, 1, 1},
    {0, 1, 1, 1}, {1, 0.65f, 0, 1}, {0.5f, 0, 0.5f, 1}, {1, 0.75f, 0.8f, 1}, {0.56f, 0.93f, 0.56f, 1},
};

static const int AGENT_COUNT = 300;
static const float AGENT_RADIUS = 0.35f;
static const float AGENT_HEIGHT = 0.4f;

struct CrowdStressTestExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    Crowd crowd;
    uint32_t random_state;
    Array<double> next_target_times;
    bool show_velocity_vectors;
    bool periodic_scatter;
    double update_milliseconds;
};

static CrowdStressTestExample crowd_stress_test_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static double next_target_time(double now) {
    CrowdStressTestExample &state = crowd_stress_test_example;
    return now + 5000 + next_random(&state.random_state) * 5000;
}

static void pick_new_target(uint32_t agent_id, double now) {
    CrowdStressTestExample &state = crowd_stress_test_example;
    FindRandomPointResult point = {};
    nav_mesh_search::find_random_point(point, state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
    if (!point.success) return;
    crowd::request_move_target(state.crowd, agent_id, point.node_ref, point.position);
    state.next_target_times[agent_id] = next_target_time(now);
}

static void init(App &app) {
    CrowdStressTestExample &state = crowd_stress_test_example;
    state = {};
    state.random_state = 42;
    state.show_velocity_vectors = true;
    state.periodic_scatter = true;

    app::load_level(app, "crowd-simulation-stress-test.glb");
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});

    const float cell_size = 0.15f;
    const float cell_height = 0.15f;
    const int tile_size_voxels = 32;
    const float walkable_radius_world = 0.3f;
    const float walkable_climb_world = 0.5f;
    const float walkable_height_world = 1;
    TiledNavMeshOptions options = {
        .cell_size = cell_size,
        .cell_height = cell_height,
        .tile_size_voxels = tile_size_voxels,
        .tile_size_world = tile_size_voxels * cell_size,
        .walkable_radius_voxels = (int)ceilf(walkable_radius_world / cell_size),
        .walkable_radius_world = walkable_radius_world,
        .walkable_climb_voxels = (int)ceilf(walkable_climb_world / cell_height),
        .walkable_climb_world = walkable_climb_world,
        .walkable_height_voxels = (int)ceilf(walkable_height_world / cell_height),
        .walkable_height_world = walkable_height_world,
        .walkable_slope_angle_degrees = 45,
        .border_size = 4,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 5,
        .detail_sample_distance = cell_size * 6,
        .detail_sample_max_error = cell_height * 1,
    };

    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions, level.walkable_indices}, options);
    state.generated = true;
    NavMesh &nav_mesh = state.result.nav_mesh;

    FindNearestPolyResult seed = {};
    nav_mesh_api::find_nearest_poly(seed, nav_mesh, {0, 0, 0}, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (seed.success) {
        Array<NodeRef> unreachable = {};
        unreachable.resize(nav_mesh_api::get_max_node_count(nav_mesh));
        FloodFillNavMeshResult flood = {};
        flood_fill_nav_mesh::flood_fill_nav_mesh(flood, nav_mesh, {&seed.node_ref, 1}, {}, unreachable);
        unreachable.resize(flood.unreachable_count);
        for (NodeRef ref : unreachable) nav_mesh.nodes[node::get_node_ref_index(ref)].flags = 0;
        unreachable.release();
    }

    DebugPrimitives nav_mesh_helper = debug::create_nav_mesh_helper(nav_mesh);
    app::lift(nav_mesh_helper, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh_helper);
    debug::release(nav_mesh_helper);

    state.crowd = crowd::create(1);
    AgentParams params = {
        .radius = AGENT_RADIUS,
        .height = AGENT_HEIGHT,
        .max_acceleration = 15.0f,
        .max_speed = 3.5f,
        .collision_query_range = 2,
        .separation_weight = 0.5f,
        .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_OPTIMIZE_TOPO |
                        CROWD_UPDATE_OPTIMIZE_VIS,
    };

    double now = app::now_milliseconds();
    for (int i = 0; i < AGENT_COUNT; i++) {
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        uint32_t agent_id = crowd::add_agent(state.crowd, nav_mesh, point.position, params);
        if (state.next_target_times.length <= agent_id) state.next_target_times.resize(agent_id + 1, 0);
        state.next_target_times[agent_id] = next_target_time(now);
    }

    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (crowd::is_agent(state.crowd, id)) pick_new_target(id, now);
    }
}

static void velocity_arrow(DrawList &list, Vec3 origin, Vec3 velocity, Color color) {
    float length = vec3::length(velocity);
    if (length <= 0.01f) return;
    renderer::arrow(list, origin, vec3::scale_and_add(origin, velocity, 0.5f), color);
}

static void update(App &app, float delta_time) {
    CrowdStressTestExample &state = crowd_stress_test_example;
    const NavMesh &nav_mesh = state.result.nav_mesh;
    double now = app::now_milliseconds();

    if (state.periodic_scatter) {
        for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
            if (crowd::is_agent(state.crowd, id) && now >= state.next_target_times[id]) pick_new_target(id, now);
        }
    }

    double start = app::now_milliseconds();
    crowd::update(state.crowd, nav_mesh, fminf(delta_time, 0.1f));
    state.update_milliseconds = app::now_milliseconds() - start;

    app::draw_static(app, state.nav_mesh_drawing);

    DrawList &list = app::frame(app);
    uint32_t color_count = sizeof(AGENT_COLORS) / sizeof(AGENT_COLORS[0]);
    uint32_t instance = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        const Agent &agent = state.crowd.agents[id];
        Color color = AGENT_COLORS[instance++ % color_count];
        renderer::cylinder(list, agent.position, AGENT_RADIUS, AGENT_HEIGHT + AGENT_RADIUS * 2, color);
        renderer::point(list, vec3::add(agent.target_position, {0, 0.1f, 0}), 0.1f, color);
        if (state.show_velocity_vectors) {
            velocity_arrow(list, vec3::add(agent.position, {0, 0.5f, 0}), agent.velocity, {0, 1, 0, 1});
            velocity_arrow(list, vec3::add(agent.position, {0, 0.6f, 0}), agent.desired_velocity, {1, 0, 0, 1});
        }
    }
}

static void ui(App &) {
    CrowdStressTestExample &state = crowd_stress_test_example;
    ImGui::TextWrapped("%d agents wander the level, each picking a new random target every 5 to 10 seconds", AGENT_COUNT);
    ImGui::Text("crowd update %.3f ms", state.update_milliseconds);
    ImGui::Checkbox("show velocity vectors", &state.show_velocity_vectors);
    ImGui::Checkbox("periodic scatter", &state.periodic_scatter);
}

static void shutdown(App &) {
    CrowdStressTestExample &state = crowd_stress_test_example;
    crowd::release(state.crowd);
    state.next_target_times.release();
    renderer::release(state.nav_mesh_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_CROWD_SIMULATION_STRESS_TEST = {"crowd simulation stress test", init, update, ui, shutdown};
