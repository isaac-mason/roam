#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/debug/debug.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

static const float PLATFORM_HEIGHT = 2.5f;
static const float OFF_MESH_DURATION = 0.8f;
static const float OFF_MESH_ARC_HEIGHT = 0.8f;
static const int MAX_AGENTS = 64;

static const Color AGENT_COLORS[] = {
    {0.98f, 0.36f, 0.36f, 1}, {0.36f, 0.75f, 0.98f, 1}, {0.55f, 0.9f, 0.4f, 1}, {0.98f, 0.8f, 0.3f, 1},
    {0.8f, 0.5f, 0.98f, 1},   {0.98f, 0.6f, 0.3f, 1},   {0.4f, 0.95f, 0.85f, 1},
};

enum SegmentLinksMoving {
    SEGMENT_LINKS_MOVING_NONE,
    SEGMENT_LINKS_MOVING_START,
    SEGMENT_LINKS_MOVING_END,
};

struct SegmentLinksExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing links_drawing;
    StaticDrawing path_drawing;
    QueryWorkspace workspace;
    Vec3 start;
    Vec3 end;
    SegmentLinksMoving moving;
    bool path_success;
    uint32_t point_count;
    StraightPathPoint points[MAX_PATH_POINTS];
    bool show_crowd;
    bool ledge_one_way;
    int agent_count;
    Crowd crowd;
    uint32_t random_state;
    double last_scatter_time;
    uint32_t ledge_link;
    uint32_t gap_link;
};

static SegmentLinksExample segment_links_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static void push_vertex(Model &model, Vec3 position, Color color, bool walkable) {
    model.positions.push(position.x);
    model.positions.push(position.y);
    model.positions.push(position.z);
    model.colors.push(color.r);
    model.colors.push(color.g);
    model.colors.push(color.b);
    model.bounds = box3::expand_by_point(model.bounds, position);
    if (!walkable) return;
    model.walkable_positions.push(position.x);
    model.walkable_positions.push(position.y);
    model.walkable_positions.push(position.z);
}

static void push_quad(Model &model, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Color color, bool walkable) {
    uint32_t base = model.positions.length / 3;
    uint32_t walkable_base = model.walkable_positions.length / 3;
    push_vertex(model, a, color, walkable);
    push_vertex(model, b, color, walkable);
    push_vertex(model, c, color, walkable);
    push_vertex(model, d, color, walkable);
    const uint32_t triangles[6] = {0, 1, 2, 0, 2, 3};
    for (uint32_t index : triangles) model.indices.push(base + index);
    if (!walkable) return;
    for (uint32_t index : triangles) model.walkable_indices.push(walkable_base + index);
}

static void push_box(Model &model, Vec3 min, Vec3 max, Color top, Color side) {
    push_quad(model, {min.x, max.y, min.z}, {min.x, max.y, max.z}, {max.x, max.y, max.z}, {max.x, max.y, min.z}, top, true);
    push_quad(model, {min.x, min.y, max.z}, {max.x, min.y, max.z}, {max.x, max.y, max.z}, {min.x, max.y, max.z}, side, false);
    push_quad(model, {max.x, min.y, min.z}, {min.x, min.y, min.z}, {min.x, max.y, min.z}, {max.x, max.y, min.z}, side, false);
    push_quad(model, {min.x, min.y, min.z}, {min.x, min.y, max.z}, {min.x, max.y, max.z}, {min.x, max.y, min.z}, side, false);
    push_quad(model, {max.x, min.y, max.z}, {max.x, min.y, min.z}, {max.x, max.y, min.z}, {max.x, max.y, max.z}, side, false);
}

static Model create_level() {
    const Color ground = {0.55f, 0.6f, 0.5f, 1};
    const Color platform = {0.62f, 0.58f, 0.52f, 1};
    const Color wall = {0.42f, 0.4f, 0.38f, 1};
    const Color ramp = {0.6f, 0.55f, 0.45f, 1};

    Model model = {};
    model.bounds = box3::empty();
    push_quad(model, {-12, 0, -1}, {-12, 0, 12}, {9, 0, 12}, {9, 0, -1}, ground, true);
    push_box(model, {-12, 0, -8}, {12, PLATFORM_HEIGHT, -1}, platform, wall);
    push_box(model, {-12, 0, -16}, {12, PLATFORM_HEIGHT, -11}, platform, wall);
    push_box(model, {-1.5f, 0, -1}, {1.5f, 1.4f, 1.5f}, wall, wall);
    push_quad(model, {9, 0, 12}, {12, 0, 12}, {12, PLATFORM_HEIGHT, -1}, {9, PLATFORM_HEIGHT, -1}, ramp, true);
    model.loaded = true;
    return model;
}

static void add_links() {
    SegmentLinksExample &state = segment_links_example;
    NavMesh &nav_mesh = state.result.nav_mesh;
    state.ledge_link = nav_mesh_api::add_off_mesh_segment_connection(
        nav_mesh, {
                      .start_a = {-10, PLATFORM_HEIGHT, -1},
                      .start_b = {8, PLATFORM_HEIGHT, -1},
                      .end_a = {-10, 0, 0},
                      .end_b = {8, 0, 0},
                      .radius = 0.6f,
                      .inset = 0.3f,
                      .direction = state.ledge_one_way ? OFF_MESH_CONNECTION_DIRECTION_START_TO_END : OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
                      .flags = 0xffffff,
                      .area = 1,
                      .align = false,
                  });
    state.gap_link = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, {
                                                                                 .start_a = {-10, PLATFORM_HEIGHT, -8},
                                                                                 .start_b = {10, PLATFORM_HEIGHT, -8},
                                                                                 .end_a = {-10, PLATFORM_HEIGHT, -11},
                                                                                 .end_b = {10, PLATFORM_HEIGHT, -11},
                                                                                 .radius = 0.6f,
                                                                                 .inset = 0.3f,
                                                                                 .direction = OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
                                                                                 .flags = 0xffffff,
                                                                                 .area = 2,
                                                                                 .align = false,
                                                                             });
}

static void update_links_drawing() {
    SegmentLinksExample &state = segment_links_example;
    renderer::release(state.links_drawing);
    DebugPrimitives links = debug::create_nav_mesh_off_mesh_connections_helper(state.result.nav_mesh);
    state.links_drawing = renderer::create_static(links);
    debug::release(links);
}

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.533f, 0.533f, 0.533f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void update_path() {
    SegmentLinksExample &state = segment_links_example;
    renderer::release(state.path_drawing);

    DrawList list = {};
    flag(list, state.start, {0.129f, 0.588f, 0.953f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, state.result.nav_mesh, state.start, state.end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER,
                         state.points);
    state.path_success = path.success;
    state.point_count = path.success ? path.point_count : 0;

    for (uint32_t i = 0; i < state.point_count; i++) {
        const StraightPathPoint &point = state.points[i];
        bool off_mesh = point.type == NODE_TYPE_OFFMESH;
        renderer::point(list, point.position, off_mesh ? 0.25f : 0.15f, off_mesh ? Color{1, 0.5f, 0, 1} : Color{1, 0, 0, 1});
        if (i == 0) continue;
        Vec3 previous = state.points[i - 1].position;
        bool jump = state.points[i - 1].type == NODE_TYPE_OFFMESH;
        renderer::thick_line(list, vec3::add(previous, {0, 0.1f, 0}), vec3::add(point.position, {0, 0.1f, 0}), 0.06f,
                             jump ? Color{1, 0.5f, 0, 1} : Color{1, 1, 0, 1});
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static AgentParams agent_params() {
    return {
        .radius = 0.3f,
        .height = 0.6f,
        .max_acceleration = 15.0f,
        .max_speed = 3.5f,
        .collision_query_range = 2,
        .separation_weight = 0.5f,
        .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OPTIMIZE_VIS |
                        CROWD_UPDATE_OPTIMIZE_TOPO,
        .auto_traverse_off_mesh_connections = false,
    };
}

static void scatter() {
    SegmentLinksExample &state = segment_links_example;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::request_move_target(state.crowd, id, point.node_ref, point.position);
    }
}

static void spawn_agents() {
    SegmentLinksExample &state = segment_links_example;
    crowd::release(state.crowd);
    state.crowd = crowd::create(1);
    for (int i = 0; i < state.agent_count; i++) {
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::add_agent(state.crowd, state.result.nav_mesh, point.position, agent_params());
    }
    scatter();
    state.last_scatter_time = app::now_milliseconds();
}

static void rebuild_links() {
    SegmentLinksExample &state = segment_links_example;
    nav_mesh_api::remove_off_mesh_connection(state.result.nav_mesh, state.ledge_link);
    nav_mesh_api::remove_off_mesh_connection(state.result.nav_mesh, state.gap_link);
    add_links();
    update_links_drawing();
    update_path();
    if (state.show_crowd) spawn_agents();
}

static void animate_off_mesh_connections(float delta_time) {
    SegmentLinksExample &state = segment_links_example;
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

static void init(App &app) {
    SegmentLinksExample &state = segment_links_example;
    state = {};
    state.workspace = nav_mesh_search::create_query_workspace();
    state.start = {-6, PLATFORM_HEIGHT, -14};
    state.end = {5, 0, 8};
    state.ledge_one_way = true;
    state.agent_count = 24;
    state.random_state = 7;
    app.want_hover = true;

    app::set_level(app, create_level());

    NavMeshConfig config = nav_mesh_config::create();
    config.cell_size = 0.2f;
    config.cell_height = 0.1f;
    config.walkable_radius_world = 0.3f;
    config.walkable_climb_world = 0.4f;
    config.walkable_height_world = 1.5f;
    config.border_size = 4;
    config.tile_size_voxels = 48;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh({level.walkable_positions, level.walkable_indices},
                                                                     nav_mesh_config::tiled_options(config));
    state.generated = true;

    add_links();

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.05f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);
    update_links_drawing();
    update_path();
}

static void update(App &app, float delta_time) {
    SegmentLinksExample &state = segment_links_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary) {
            state.moving = state.moving == SEGMENT_LINKS_MOVING_END ? SEGMENT_LINKS_MOVING_NONE : SEGMENT_LINKS_MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == SEGMENT_LINKS_MOVING_START ? SEGMENT_LINKS_MOVING_NONE : SEGMENT_LINKS_MOVING_START;
            state.start = app.click.point;
        }
        update_path();
    } else if (state.moving != SEGMENT_LINKS_MOVING_NONE && app.hover.hit) {
        if (state.moving == SEGMENT_LINKS_MOVING_START) state.start = app.hover.point;
        else state.end = app.hover.point;
        update_path();
    }

    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.links_drawing);
    if (!state.show_crowd) {
        app::draw_static(app, state.path_drawing);
        return;
    }

    float clamped = delta_time > 0.1f ? 0.1f : delta_time;
    double now = app::now_milliseconds();
    if (now - state.last_scatter_time > 6000) {
        scatter();
        state.last_scatter_time = now;
    }
    crowd::update(state.crowd, state.result.nav_mesh, clamped);
    animate_off_mesh_connections(clamped);

    DrawList &list = app::frame(app);
    uint32_t index = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        const Agent &agent = state.crowd.agents[id];
        Color color = AGENT_COLORS[index++ % (sizeof(AGENT_COLORS) / sizeof(AGENT_COLORS[0]))];
        renderer::cylinder(list, agent.position, agent.radius, agent.height, color);
        renderer::point(list, vec3::add(agent.target_position, {0, 0.1f, 0}), 0.08f, color);
    }
}

static void ui(App &) {
    SegmentLinksExample &state = segment_links_example;
    ImGui::TextWrapped("Segment off-mesh connections: a one-way ledge drop (the pillar splits it) and a two-way gap jump. "
                       "Paths cross where they meet the link, not at one fixed point.");
    if (ImGui::Checkbox("crowd", &state.show_crowd)) {
        if (state.show_crowd) spawn_agents();
        else crowd::release(state.crowd);
    }
    if (ImGui::Checkbox("one-way ledge drop", &state.ledge_one_way)) rebuild_links();
    if (state.show_crowd) {
        if (ImGui::SliderInt("agents", &state.agent_count, 1, MAX_AGENTS)) spawn_agents();
        if (ImGui::Button("scatter")) scatter();
        return;
    }

    ImGui::TextWrapped("click to move the start, right click to move the end, click again to place it");
    ImGui::Text("path %s, %u points", state.path_success ? "found" : "not found", state.point_count);
    for (uint32_t i = 0; i < state.point_count; i++) {
        const StraightPathPoint &point = state.points[i];
        ImGui::Text("%u %s (%.2f, %.2f, %.2f)", i, point.type == NODE_TYPE_OFFMESH ? "jump" : "walk", point.position.x, point.position.y,
                    point.position.z);
    }
}

static void shutdown(App &) {
    SegmentLinksExample &state = segment_links_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.links_drawing);
    renderer::release(state.path_drawing);
    crowd::release(state.crowd);
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_SEGMENT_OFF_MESH_CONNECTIONS = {"segment off mesh connections", init, update, ui, shutdown};
