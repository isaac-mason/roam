#include <math.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/find_smooth_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum Moving { MOVING_NONE, MOVING_START, MOVING_END };

static const OffMeshConnectionParams OFF_MESH_CONNECTIONS[] = {
    {
        .start = {3.54f, 0.27f, -3.89f},
        .end = {6.09f, 0.69f, -3.59f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = 0,
    },
    {
        .start = {6.09f, 0.69f, -3.59f},
        .end = {6.55f, 0.39f, -0.68f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = 0,
    },
};

constexpr uint32_t MAX_SMOOTH_PATH_POINTS = 1024;

struct FindSmoothPathExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing path_drawing;
    Vec3 start;
    Vec3 end;
    float step_size;
    float slop;
    Moving moving;
    double query_milliseconds;
    bool success;
    uint32_t flags;
    uint32_t node_count;
    uint32_t point_count;
    SearchWorkspace workspace;
    SmoothPathPoint points[MAX_SMOOTH_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
};

static FindSmoothPathExample find_smooth_path_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static Color hsl(float hue_degrees, float saturation, float lightness) {
    float chroma = (1 - fabsf(2 * lightness - 1)) * saturation;
    float h = hue_degrees / 60.0f;
    float x = chroma * (1 - fabsf(fmodf(h, 2) - 1));
    float r = 0, g = 0, b = 0;
    if (h < 1) {
        r = chroma, g = x;
    } else if (h < 2) {
        r = x, g = chroma;
    } else if (h < 3) {
        g = chroma, b = x;
    } else if (h < 4) {
        g = x, b = chroma;
    } else if (h < 5) {
        r = x, b = chroma;
    } else {
        r = chroma, b = x;
    }
    float m = lightness - chroma * 0.5f;
    return {r + m, g + m, b + m, 1};
}

static void update_path() {
    FindSmoothPathExample &state = find_smooth_path_example;
    renderer::release(state.path_drawing);

    const NavMesh &nav_mesh = state.result.nav_mesh;
    DrawList list = {};
    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    double begin = app::now_milliseconds();
    FindSmoothPathResult path = {};
    find_smooth_path::find_smooth_path(path, state.workspace, nav_mesh, state.start, state.end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER,
                                       state.points,
                                       {.step_size = state.step_size,
                                        .slop = state.slop,
                                        .raycast_distance = 0,
                                        .node_path = state.node_path});
    state.query_milliseconds = app::now_milliseconds() - begin;
    state.success = path.success;
    state.flags = path.flags;
    state.node_count = path.node_path_count;
    state.point_count = path.point_count;

    if (path.success) {
        DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace);
        renderer::add_primitives(list, nodes);
        debug::release(nodes);

        for (uint32_t i = 0; i < path.node_path_count; i++) {
            NodeRef node_ref = state.node_path[i];
            if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
            DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
            app::lift(poly, 0.15f);
            renderer::add_primitives(list, poly);
            debug::release(poly);
        }

        const Vec3 lift = {0, 0.2f, 0};
        for (uint32_t i = 0; i < path.point_count; i++) {
            Color color = hsl((float)i / (float)path.point_count * 360.0f, 0.9f, 0.5f);
            Vec3 position = vec3::add(state.points[i].position, lift);
            renderer::point(list, position, 0.1f, color);
            if (i > 0) renderer::thick_line(list, vec3::add(state.points[i - 1].position, lift), position, 0.1f, color);
        }
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    FindSmoothPathExample &state = find_smooth_path_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.start = {-3.94f, 0.26f, 4.71f};
    state.end = {1.01f, 2.38f, -1.93f};
    state.step_size = 0.3f;
    state.slop = 0.01f;
    app.want_hover = true;

    app::load_level(app, "nav-test.glb");
    camera::look_from(app.camera, {-2, 10, 10}, {0, 0, 0}, scalar::degrees_to_radians(75));

    NavMeshConfig config = nav_mesh_config::create();
    config.tile_size_voxels = 64;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(config));
    state.generated = true;

    for (const OffMeshConnectionParams &connection : OFF_MESH_CONNECTIONS) nav_mesh_api::add_off_mesh_connection(state.result.nav_mesh, connection);

    DrawList list = {};
    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.1f);
    renderer::add_primitives(list, nav_mesh);
    debug::release(nav_mesh);
    DebugPrimitives off_mesh_connections = debug::create_nav_mesh_off_mesh_connections_helper(state.result.nav_mesh);
    renderer::add_primitives(list, off_mesh_connections);
    debug::release(off_mesh_connections);
    state.nav_mesh_drawing = renderer::create_static(list);
    renderer::release(list);

    update_path();
}

static void update(App &app, float) {
    FindSmoothPathExample &state = find_smooth_path_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary || app.click.shift) {
            state.moving = state.moving == MOVING_END ? MOVING_NONE : MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == MOVING_START ? MOVING_NONE : MOVING_START;
            state.start = app.click.point;
        }
        update_path();
    } else if (state.moving != MOVING_NONE && app.hover.hit) {
        Vec3 &moved = state.moving == MOVING_START ? state.start : state.end;
        if (!vec3::exactly_equals(moved, app.hover.point)) {
            moved = app.hover.point;
            update_path();
        }
    }
    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    FindSmoothPathExample &state = find_smooth_path_example;
    ImGui::TextWrapped("click to move the start, right click or shift click to move the end, click again to place it");
    if (ImGui::CollapsingHeader("Smooth Path", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SliderFloat("Step Size", &state.step_size, 0.1f, 1, "%.1f")) update_path();
        if (ImGui::SliderFloat("Slop", &state.slop, 0.01f, 0.2f, "%.2f")) update_path();
    }
    ImGui::Text("%s in %.3f ms", state.success ? "found" : "failed", state.query_milliseconds);
    ImGui::Text("%u nodes, %u points", state.node_count, state.point_count);
    if (state.flags & FIND_SMOOTH_PATH_RESULT_PARTIAL_PATH) ImGui::Text("partial path");
}

static void shutdown(App &) {
    FindSmoothPathExample &state = find_smooth_path_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.path_drawing);
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_SMOOTH_PATH = {"find smooth path", init, update, ui, shutdown};
