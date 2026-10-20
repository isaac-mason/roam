#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

static const Color AGENT_COLORS[] = {
    {0, 0, 1, 1}, {0, 1, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {1, 0, 1, 1},
    {0, 1, 1, 1}, {1, 0.65f, 0, 1}, {0.5f, 0, 0.5f, 1}, {1, 0.75f, 0.8f, 1}, {0.56f, 0.93f, 0.56f, 1},
};

struct CrowdExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    Crowd crowd;
    uint32_t random_state;
    int agent_count;
    uint32_t update_flags;
    bool paused;
    bool show_corridors;
    bool show_corners;
    bool show_boundaries;
    bool show_neighbours;
    bool show_velocities;
    bool has_target;
    Vec3 target;
    double update_milliseconds;
};

static CrowdExample crowd_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static AgentParams agent_params() {
    return {
        .radius = 0.3f,
        .height = 0.6f,
        .max_acceleration = 15.0f,
        .max_speed = 3.5f,
        .collision_query_range = 2,
        .separation_weight = 0.5f,
        .update_flags = crowd_example.update_flags,
    };
}

static void spawn_agents() {
    CrowdExample &state = crowd_example;
    crowd::release(state.crowd);
    state.crowd = crowd::create(1);
    state.has_target = false;
    for (int i = 0; i < state.agent_count; i++) {
        FindRandomPointResult point =
            nav_mesh_search::find_random_point(state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::add_agent(state.crowd, state.result.nav_mesh, point.position, agent_params());
    }
}

static void scatter() {
    CrowdExample &state = crowd_example;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        FindRandomPointResult point =
            nav_mesh_search::find_random_point(state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::request_move_target(state.crowd, id, point.node_ref, point.position);
    }
    state.has_target = false;
}

static void init(App &app) {
    CrowdExample &state = crowd_example;
    state = {};
    state.random_state = 42;
    state.agent_count = 10;
    state.update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_OPTIMIZE_TOPO |
                         CROWD_UPDATE_OPTIMIZE_VIS;
    state.show_velocities = true;

    app::load_level(app, "nav-test.glb");

    NavMeshConfig config = nav_mesh_config::create();
    config.tile_size_voxels = 64;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(config));
    state.generated = true;

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    spawn_agents();
}

static void draw_agent(DrawList &list, const Agent &agent, Color color) {
    CrowdExample &state = crowd_example;
    renderer::cylinder(list, agent.position, agent.radius, agent.height, color);
    renderer::circle(list, {agent.position.x, agent.position.y + 0.02f, agent.position.z}, agent.radius * 1.2f, {1, 1, 1, 0.6f});

    Vec3 lifted = {agent.position.x, agent.position.y + agent.height + 0.05f, agent.position.z};
    if (state.show_velocities) {
        renderer::arrow(list, lifted, vec3::add(lifted, vec3::scale(agent.velocity, 0.5f)), {1, 1, 1, 1});
        renderer::arrow(list, lifted, vec3::add(lifted, vec3::scale(agent.desired_velocity, 0.5f)), {0.3f, 1, 0.3f, 0.8f});
    }

    if (state.show_corners) {
        Vec3 previous = agent.position;
        for (const StraightPathPoint &corner : agent.corners) {
            renderer::line(list, previous, corner.position, {1, 0.5f, 0, 1});
            renderer::point(list, corner.position, 0.06f, {1, 0.5f, 0, 1});
            previous = corner.position;
        }
    }

    if (state.show_boundaries) {
        for (uint32_t i = 0; i < agent.boundary.segment_count; i++) {
            Vec3 lift = {0, 0.1f, 0};
            renderer::line(list, vec3::add(agent.boundary.segments[i].start, lift), vec3::add(agent.boundary.segments[i].end, lift), {1, 0.2f, 0.2f, 1});
        }
    }

    if (state.show_neighbours) {
        for (const AgentNeighbour &neighbour : agent.neis) {
            if (!crowd::is_agent(state.crowd, neighbour.agent_id)) continue;
            renderer::line(list, lifted, vec3::add(state.crowd.agents[neighbour.agent_id].position, {0, agent.height + 0.05f, 0}), {0.5f, 0.5f, 1, 0.5f});
        }
    }
}

static void update(App &app, float delta_time) {
    CrowdExample &state = crowd_example;
    const NavMesh &nav_mesh = state.result.nav_mesh;

    if (app.click.clicked && app.click.hit) {
        FindNearestPolyResult nearest = nav_mesh_api::find_nearest_poly(nav_mesh, app.click.point, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        if (nearest.success) {
            for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
                if (crowd::is_agent(state.crowd, id)) crowd::request_move_target(state.crowd, id, nearest.node_ref, nearest.position);
            }
            state.has_target = true;
            state.target = nearest.position;
        }
    }

    if (!state.paused) {
        double start = app::now_milliseconds();
        crowd::update(state.crowd, nav_mesh, fminf(delta_time, 0.1f));
        state.update_milliseconds = app::now_milliseconds() - start;
    }

    app::draw_static(app, state.nav_mesh_drawing);

    DrawList &list = app::frame(app);
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        const Agent &agent = state.crowd.agents[id];
        if (state.show_corridors) {
            for (NodeRef node_ref : agent.corridor.path) {
                DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref, {1, 0.8f, 0.2f});
                renderer::add_primitives(list, poly);
                debug::release(poly);
            }
        }
        draw_agent(list, agent, AGENT_COLORS[id % (sizeof(AGENT_COLORS) / sizeof(AGENT_COLORS[0]))]);
    }

    if (state.has_target) renderer::circle(list, {state.target.x, state.target.y + 0.05f, state.target.z}, 0.3f, {1, 1, 0, 1});
}

static void flag_checkbox(const char *label, uint32_t flag) {
    CrowdExample &state = crowd_example;
    bool enabled = (state.update_flags & flag) != 0;
    if (!ImGui::Checkbox(label, &enabled)) return;
    state.update_flags = enabled ? state.update_flags | flag : state.update_flags & ~flag;
    for (Agent &agent : state.crowd.agents) {
        if (agent.allocated) agent.update_flags = state.update_flags;
    }
}

static void ui(App &) {
    CrowdExample &state = crowd_example;
    ImGui::TextWrapped("click to send every agent to a point");
    ImGui::Text("crowd update %.3f ms", state.update_milliseconds);
    ImGui::Checkbox("paused", &state.paused);
    ImGui::SliderInt("agents", &state.agent_count, 1, 200);
    if (ImGui::Button("respawn")) spawn_agents();
    ImGui::SameLine();
    if (ImGui::Button("scatter")) scatter();

    if (ImGui::CollapsingHeader("Update flags", ImGuiTreeNodeFlags_DefaultOpen)) {
        flag_checkbox("anticipate turns", CROWD_UPDATE_ANTICIPATE_TURNS);
        flag_checkbox("obstacle avoidance", CROWD_UPDATE_OBSTACLE_AVOIDANCE);
        flag_checkbox("separation", CROWD_UPDATE_SEPARATION);
        flag_checkbox("optimize visibility", CROWD_UPDATE_OPTIMIZE_VIS);
        flag_checkbox("optimize topology", CROWD_UPDATE_OPTIMIZE_TOPO);
    }

    if (ImGui::CollapsingHeader("Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("velocities", &state.show_velocities);
        ImGui::Checkbox("corners", &state.show_corners);
        ImGui::Checkbox("corridors", &state.show_corridors);
        ImGui::Checkbox("local boundaries", &state.show_boundaries);
        ImGui::Checkbox("neighbours", &state.show_neighbours);
    }
}

static void shutdown(App &) {
    CrowdExample &state = crowd_example;
    crowd::release(state.crowd);
    renderer::release(state.nav_mesh_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_CROWD_SIMULATION = {"crowd simulation", init, update, ui, shutdown};
