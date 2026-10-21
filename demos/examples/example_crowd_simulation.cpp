#include <math.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/util/map.h"

using namespace roam;

static const Color AGENT_COLORS[] = {
    {0, 0, 1, 1}, {0, 1, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {1, 0, 1, 1},
    {0, 1, 1, 1}, {1, 0.65f, 0, 1}, {0.5f, 0, 0.5f, 1}, {1, 0.75f, 0.8f, 1}, {0.56f, 0.93f, 0.56f, 1},
};

static const OffMeshConnectionParams OFF_MESH_CONNECTIONS[] = {
    {{0.39257542778564014f, 3.9164539337158204f, 2.7241512942770267f}, {1.2915380743929097f, 2.8616158587143867f, 3.398593875470379f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 0xffffff, 0},
    {{3.491345350637368f, 3.169861227710937f, 2.8419154179454473f}, {4.0038066734125435f, 0.466454005241394f, 1.686211347289651f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 0xffffff, 0},
    {{4.612475330561077f, 0.466454005241394f, 2.7619018768157435f}, {6.696740007427642f, 0.5132029874438654f, 2.5838885990777243f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 0xffffff, 0},
    {{3.8221359252929688f, 0.47645399570465086f, -4.391971844600165f}, {5.91173484469572f, 0.6573111525835266f, -4.671632275169128f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 0xffffff, 0},
    {{8.354324172733968f, 0.5340897451517822f, -3.2333049546492223f}, {8.461111697936666f, 0.8365034207348984f, -1.0863215738579806f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 0xffffff, 0},
};

static const double SCATTER_TIMEOUT_MILLISECONDS = 5000;
static const float OFF_MESH_DURATION = 0.8f;
static const float OFF_MESH_ARC_HEIGHT = 1.0f;

struct CrowdExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing off_mesh_connections_drawing;
    Crowd crowd;
    uint32_t random_state;
    int agent_count;
    uint32_t update_flags;
    Array<bool> selected;
    bool paused;
    bool periodic_scatter;
    bool show_velocity_vectors;
    bool show_poly_helpers;
    bool show_local_boundary;
    bool show_obstacle_segments;
    bool show_path_line;
    bool show_obstacle_avoidance_debug;
    int debug_agent_index;
    double last_scatter_time;
    double update_milliseconds;
};

static CrowdExample crowd_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static Color agent_color(uint32_t index) { return AGENT_COLORS[index % (sizeof(AGENT_COLORS) / sizeof(AGENT_COLORS[0]))]; }

static bool is_selected(uint32_t agent_id) {
    CrowdExample &state = crowd_example;
    return agent_id < state.selected.length && state.selected[agent_id];
}

static void set_selected(uint32_t agent_id, bool selected) {
    CrowdExample &state = crowd_example;
    if (state.selected.length <= agent_id) state.selected.resize(agent_id + 1, false);
    state.selected[agent_id] = selected;
}

static void clear_selection() {
    CrowdExample &state = crowd_example;
    for (bool &selected : state.selected) selected = false;
}

static uint32_t selected_count() {
    CrowdExample &state = crowd_example;
    uint32_t count = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) count += crowd::is_agent(state.crowd, id) && is_selected(id) ? 1 : 0;
    return count;
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
        .auto_traverse_off_mesh_connections = false,
    };
}

static void scatter() {
    CrowdExample &state = crowd_example;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id) || is_selected(id)) continue;
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::request_move_target(state.crowd, id, point.node_ref, point.position);
    }
}

static void spawn_agents() {
    CrowdExample &state = crowd_example;
    crowd::release(state.crowd);
    state.selected.clear();
    state.crowd = crowd::create(1);
    for (int i = 0; i < state.agent_count; i++) {
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::add_agent(state.crowd, state.result.nav_mesh, point.position, agent_params());
    }
    scatter();
}

static void init(App &app) {
    CrowdExample &state = crowd_example;
    state = {};
    state.random_state = 42;
    state.agent_count = 10;
    state.update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_OPTIMIZE_TOPO |
                         CROWD_UPDATE_OPTIMIZE_VIS;
    state.periodic_scatter = true;
    state.show_velocity_vectors = true;
    state.show_poly_helpers = true;

    app::load_level(app, "nav-test.glb");
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});

    NavMeshConfig config = nav_mesh_config::create();
    config.tile_size_voxels = 64;
    config.walkable_radius_world = 0.15f;
    config.walkable_height_world = 1;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions, level.walkable_indices}, nav_mesh_config::tiled_options(config));
    state.generated = true;
    NavMesh &nav_mesh = state.result.nav_mesh;

    for (const OffMeshConnectionParams &connection : OFF_MESH_CONNECTIONS) nav_mesh_api::add_off_mesh_connection(nav_mesh, connection);

    DebugPrimitives nav_mesh_helper = debug::create_nav_mesh_helper(nav_mesh);
    app::lift(nav_mesh_helper, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh_helper);
    debug::release(nav_mesh_helper);

    DebugPrimitives off_mesh_helper = debug::create_nav_mesh_off_mesh_connections_helper(nav_mesh);
    state.off_mesh_connections_drawing = renderer::create_static(off_mesh_helper);
    debug::release(off_mesh_helper);

    spawn_agents();
    state.last_scatter_time = app::now_milliseconds();
}

static float agent_body_height(const Agent &agent) { return agent.height + agent.radius; }

static bool ray_hits_agent(Ray ray, const Agent &agent, float &out_t) {
    Vec3 offset = vec3::subtract(ray.origin, agent.position);
    float a = ray.direction.x * ray.direction.x + ray.direction.z * ray.direction.z;
    float b = 2 * (offset.x * ray.direction.x + offset.z * ray.direction.z);
    float c = offset.x * offset.x + offset.z * offset.z - agent.radius * agent.radius;
    if (a < 1e-8f) return false;
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;
    float root = sqrtf(discriminant);
    float candidates[2] = {(-b - root) / (2 * a), (-b + root) / (2 * a)};
    for (float t : candidates) {
        if (t < 0) continue;
        float y = ray.origin.y + ray.direction.y * t - agent.position.y;
        if (y >= 0 && y <= agent_body_height(agent)) {
            out_t = t;
            return true;
        }
    }
    float top = agent.position.y + agent_body_height(agent);
    if (fabsf(ray.direction.y) > 1e-6f) {
        float t = (top - ray.origin.y) / ray.direction.y;
        Vec3 point = vec3::scale_and_add(ray.origin, ray.direction, t);
        float dx = point.x - agent.position.x;
        float dz = point.z - agent.position.z;
        if (t >= 0 && dx * dx + dz * dz <= agent.radius * agent.radius) {
            out_t = t;
            return true;
        }
    }
    return false;
}

static void handle_click(const App &app) {
    CrowdExample &state = crowd_example;
    const Click &click = app.click;
    if (!click.clicked) return;

    if (click.secondary) {
        if (!click.hit || selected_count() == 0) return;
        FindNearestPolyResult nearest = {};
        nav_mesh_api::find_nearest_poly(nearest, state.result.nav_mesh, click.point, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        if (!nearest.success) return;
        for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
            if (crowd::is_agent(state.crowd, id) && is_selected(id)) crowd::request_move_target(state.crowd, id, nearest.node_ref, nearest.position);
        }
        return;
    }

    int64_t clicked_agent = -1;
    if (click.hit) {
        Ray ray = {app.camera.eye, vec3::normalize(vec3::subtract(click.point, app.camera.eye))};
        float best = vec3::distance(click.point, app.camera.eye) + 0.01f;
        for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
            if (!crowd::is_agent(state.crowd, id)) continue;
            float t = 0;
            if (ray_hits_agent(ray, state.crowd.agents[id], t) && t < best) {
                best = t;
                clicked_agent = id;
            }
        }
    }

    if (clicked_agent >= 0) {
        uint32_t id = (uint32_t)clicked_agent;
        if (click.shift) {
            set_selected(id, !is_selected(id));
        } else {
            clear_selection();
            set_selected(id, true);
        }
    } else if (!click.shift) {
        clear_selection();
    }
}

static void animate_off_mesh_connections(float delta_time) {
    CrowdExample &state = crowd_example;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        Agent &agent = state.crowd.agents[id];
        if (agent.state != AGENT_STATE_OFFMESH || !agent.off_mesh_animation.active) continue;
        OffMeshAnimation &animation = agent.off_mesh_animation;
        animation.t += delta_time;
        if (animation.t >= OFF_MESH_DURATION) {
            crowd::complete_off_mesh_connection(state.crowd, id);
            continue;
        }
        float progress = animation.t / OFF_MESH_DURATION;
        Vec3 start = animation.start_position;
        Vec3 end = animation.end_position;
        float parabola = -4 * OFF_MESH_ARC_HEIGHT * (progress - 0.5f) * (progress - 0.5f) + OFF_MESH_ARC_HEIGHT;
        agent.position = {
            start.x + (end.x - start.x) * progress,
            start.y + (end.y - start.y) * progress + parabola,
            start.z + (end.z - start.z) * progress,
        };
    }
}

static void velocity_arrow(DrawList &list, Vec3 origin, Vec3 velocity, Color color) {
    if (vec3::length(velocity) <= 0.01f) return;
    renderer::arrow(list, origin, vec3::scale_and_add(origin, velocity, 0.5f), color);
}

static Color sample_color(float penalty) {
    if (penalty < 0.01f) return {1, 1, 1, 0.7f};
    float normalized = fminf(penalty / 10.0f, 1.0f);
    if (normalized < 0.5f) {
        float t = normalized * 2;
        return {1, 1 - t * 0.5f, 1 - t, 0.7f};
    }
    float t = (normalized - 0.5f) * 2;
    return {1, 0.5f - t * 0.5f, 0, 0.7f};
}

static void draw_obstacle_avoidance_debug(DrawList &list, const Agent &agent) {
    Vec3 base = vec3::add(agent.position, {0, 0.05f, 0});
    renderer::circle(list, base, agent.max_speed, {1, 1, 1, 0.3f});
    for (const ObstacleAvoidanceSample &sample : agent.obstacle_avoidance_debug_data.samples) {
        Vec3 center = {agent.position.x + sample.velocity.x, base.y, agent.position.z + sample.velocity.z};
        float half = sample.sample_size * 0.25f;
        Color color = sample_color(sample.penalty);
        Vec3 a = {center.x - half, center.y, center.z - half};
        Vec3 b = {center.x + half, center.y, center.z - half};
        Vec3 c = {center.x + half, center.y, center.z + half};
        Vec3 d = {center.x - half, center.y, center.z + half};
        renderer::triangle(list, a, c, b, color);
        renderer::triangle(list, a, d, c, color);
    }
}

static void draw_agent(DrawList &list, uint32_t agent_id, uint32_t index) {
    CrowdExample &state = crowd_example;
    const Agent &agent = state.crowd.agents[agent_id];
    Color color = agent_color(index);

    renderer::cylinder(list, agent.position, agent.radius, agent_body_height(agent), color);
    renderer::point(list, vec3::add(agent.target_position, {0, 0.1f, 0}), 0.1f, color);
    if (is_selected(agent_id)) renderer::circle(list, vec3::add(agent.position, {0, 0.05f, 0}), agent.radius * 1.5f, {0, 0.67f, 1, 1});

    if (state.show_path_line) {
        StraightPathPoint corners[3];
        FindCornersResult found = {};
        path_corridor::find_corners(found, agent.corridor, state.result.nav_mesh, corners);
        if (found.success && found.corner_count > 1) {
            const Vec3 lift = {0, 0.2f, 0};
            Vec3 previous = vec3::add(agent.position, lift);
            for (uint32_t i = 0; i < found.corner_count; i++) {
                Vec3 point = vec3::add(corners[i].position, lift);
                renderer::line(list, previous, point, color);
                previous = point;
            }
        }
    }

    if (state.show_obstacle_segments) {
        const Vec3 lift = {0, 0.3f, 0};
        for (const ObstacleSegment &segment : agent.obstacle_avoidance_query.segments) {
            renderer::line(list, vec3::add(segment.p, lift), vec3::add(segment.q, lift), segment.touch ? Color{1, 0, 0, 1} : Color{1, 0.53f, 0, 1});
        }
    }

    if (state.show_local_boundary) {
        const Vec3 lift = {0, 0.25f, 0};
        for (uint32_t i = 0; i < agent.boundary.segment_count; i++) {
            renderer::line(list, vec3::add(agent.boundary.segments[i].start, lift), vec3::add(agent.boundary.segments[i].end, lift), {0, 1, 1, 1});
        }
    }

    if (state.show_velocity_vectors) {
        velocity_arrow(list, vec3::add(agent.position, {0, 0.5f, 0}), agent.velocity, {0, 1, 0, 1});
        velocity_arrow(list, vec3::add(agent.position, {0, 0.6f, 0}), agent.desired_velocity, {1, 0, 0, 1});
    }

    if (state.show_obstacle_avoidance_debug && (int)index == state.debug_agent_index) draw_obstacle_avoidance_debug(list, agent);
}

struct PolyColorSum {
    float r, g, b;
    uint32_t count;
};

static void draw_corridor_polys(DrawList &list) {
    CrowdExample &state = crowd_example;
    Map<NodeRef, PolyColorSum> sums = {};
    Array<NodeRef> order = {};
    uint32_t index = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        Color color = agent_color(index++);
        for (NodeRef node_ref : state.crowd.agents[id].corridor.path) {
            PolyColorSum *sum = sums.get(node_ref);
            if (!sum) {
                sum = &sums.set(node_ref, {0, 0, 0, 0});
                order.push(node_ref);
            }
            sum->r += floorf(color.r * 255);
            sum->g += floorf(color.g * 255);
            sum->b += floorf(color.b * 255);
            sum->count++;
        }
    }

    for (NodeRef node_ref : order) {
        if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
        const PolyColorSum &sum = *sums.get(node_ref);
        Vec3 mixed = {floorf(sum.r / (float)sum.count) / 255, floorf(sum.g / (float)sum.count) / 255, floorf(sum.b / (float)sum.count) / 255};
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(state.result.nav_mesh, node_ref, {0.3f, 0.3f, 1});
        for (DebugPrimitive &primitive : poly) {
            primitive.transparent = true;
            primitive.opacity = 0.5f;
            for (uint32_t i = 0; i + 2 < primitive.colors.length; i += 3) {
                primitive.colors[i] = mixed.x;
                primitive.colors[i + 1] = mixed.y;
                primitive.colors[i + 2] = mixed.z;
            }
        }
        app::lift(poly, 0.15f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    order.release();
    sums.release();
}

static void update(App &app, float delta_time) {
    CrowdExample &state = crowd_example;
    const NavMesh &nav_mesh = state.result.nav_mesh;
    float clamped = fminf(delta_time, 0.1f);
    double now = app::now_milliseconds();

    handle_click(app);

    uint32_t index = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        state.crowd.agents[id].debug_obstacle_avoidance = state.show_obstacle_avoidance_debug && (int)index == state.debug_agent_index;
        index++;
    }

    if (!state.paused) {
        if (state.periodic_scatter && now - state.last_scatter_time > SCATTER_TIMEOUT_MILLISECONDS) {
            scatter();
            state.last_scatter_time = now;
        }

        double start = app::now_milliseconds();
        crowd::update(state.crowd, nav_mesh, clamped);
        state.update_milliseconds = app::now_milliseconds() - start;

        animate_off_mesh_connections(clamped);
    }

    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.off_mesh_connections_drawing);

    DrawList &list = app::frame(app);
    if (state.show_poly_helpers) draw_corridor_polys(list);
    index = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (crowd::is_agent(state.crowd, id)) draw_agent(list, id, index++);
    }
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

static void selected_agents_ui() {
    CrowdExample &state = crowd_example;
    uint32_t count = selected_count();
    if (count == 0) return;
    if (!ImGui::CollapsingHeader("Selected Agents", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const char *state_names[] = {"Invalid", "Walking", "Offmesh"};
    uint32_t shown = 0;
    uint32_t index = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        uint32_t agent_index = index++;
        if (!is_selected(id)) continue;
        if (shown == 5) {
            ImGui::Text("... and %u more", count - 5);
            break;
        }
        shown++;
        const Agent &agent = state.crowd.agents[id];
        Color color = agent_color(agent_index);
        ImGui::TextColored(ImVec4(color.r, color.g, color.b, 1), "Agent %u", id);
        ImGui::Text("pos: (%.2f, %.2f, %.2f)", agent.position.x, agent.position.y, agent.position.z);
        ImGui::Text("target: (%.2f, %.2f, %.2f)", agent.target_position.x, agent.target_position.y, agent.target_position.z);
        ImGui::Text("target path is partial: %s", agent.target_path_is_partial ? "yes" : "no");
        ImGui::Text("at target: %s", crowd::is_agent_at_target(state.crowd, id, agent.radius) ? "yes" : "no");
        ImGui::Text("velocity: %.2f m/s", vec3::length(agent.velocity));
        if (agent.corridor.path.length > 0) {
            ImGui::Text("poly: %llu", (unsigned long long)agent.corridor.path[0]);
        } else {
            ImGui::Text("poly: none");
        }
        if (agent.target_ref != INVALID_NODE_REF) ImGui::Text("target poly: %llu", (unsigned long long)agent.target_ref);
        ImGui::Text("state: %s", agent.state < 3 ? state_names[agent.state] : "Unknown");
        ImGui::Text("corridor: %u polys", agent.corridor.path.length);
        ImGui::Separator();
    }
}

static void ui(App &) {
    CrowdExample &state = crowd_example;
    ImGui::TextWrapped("click an agent to select it, shift click to add or remove it, right click to send the selected agents to a point");
    ImGui::Text("crowd update %.3f ms", state.update_milliseconds);
    ImGui::Checkbox("paused", &state.paused);
    ImGui::SliderInt("agents", &state.agent_count, 1, 200);
    if (ImGui::Button("respawn")) spawn_agents();
    ImGui::SameLine();
    if (ImGui::Button("scatter")) scatter();

    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("show velocity vectors", &state.show_velocity_vectors);
        ImGui::Checkbox("show poly helpers", &state.show_poly_helpers);
        ImGui::Checkbox("show local boundary", &state.show_local_boundary);
        ImGui::Checkbox("show obstacle segments", &state.show_obstacle_segments);
        ImGui::Checkbox("show path line", &state.show_path_line);
        ImGui::Checkbox("show obstacle avoidance debug", &state.show_obstacle_avoidance_debug);
        ImGui::SliderInt("debug agent index", &state.debug_agent_index, 0, 9);
    }

    if (ImGui::CollapsingHeader("Behavior", ImGuiTreeNodeFlags_DefaultOpen)) ImGui::Checkbox("periodic scatter", &state.periodic_scatter);

    if (ImGui::CollapsingHeader("Crowd Update Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
        flag_checkbox("anticipate turns", CROWD_UPDATE_ANTICIPATE_TURNS);
        flag_checkbox("obstacle avoidance", CROWD_UPDATE_OBSTACLE_AVOIDANCE);
        flag_checkbox("separation", CROWD_UPDATE_SEPARATION);
        flag_checkbox("optimize visibility", CROWD_UPDATE_OPTIMIZE_VIS);
        flag_checkbox("optimize topology", CROWD_UPDATE_OPTIMIZE_TOPO);
    }

    selected_agents_ui();
}

static void shutdown(App &) {
    CrowdExample &state = crowd_example;
    crowd::release(state.crowd);
    state.selected.release();
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.off_mesh_connections_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_CROWD_SIMULATION = {"crowd simulation", init, update, ui, shutdown};
