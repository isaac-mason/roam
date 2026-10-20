#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

struct FindPathExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing path_drawing;
    SearchWorkspace workspace;
    Vec3 start;
    Vec3 end;
    bool any_angle;
    bool show_search_nodes;
    double query_milliseconds;
    bool success;
    uint32_t flags;
    uint32_t node_count;
    uint32_t point_count;
};

static FindPathExample find_path_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::cylinder(list, position, 0.03f, 1.2f, {0.8f, 0.8f, 0.8f, 1});
    Vec3 top = {position.x, position.y + 1.2f, position.z};
    renderer::triangle(list, top, {top.x + 0.5f, top.y - 0.15f, top.z}, {top.x, top.y - 0.3f, top.z}, color);
}

static void update_path() {
    FindPathExample &state = find_path_example;
    renderer::release(state.path_drawing);

    const NavMesh &nav_mesh = state.result.nav_mesh;
    double start = app::now_milliseconds();
    FindPathResult path = find_path::find_path(nav_mesh, state.start, state.end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER,
                                               {state.any_angle ? 10.0f : 0.0f}, &state.workspace);
    state.query_milliseconds = app::now_milliseconds() - start;
    state.success = path.success;
    state.flags = path.flags;
    state.node_count = path.node_path.path.length;
    state.point_count = path.path.length;

    DrawList list = {};

    if (state.show_search_nodes) {
        DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace.nodes);
        renderer::add_primitives(list, nodes);
        debug::release(nodes);
    }

    for (NodeRef node_ref : path.node_path.path) {
        if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
        app::lift(poly, 0.15f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    const Vec3 lift = {0, 0.25f, 0};
    for (uint32_t i = 0; i < path.path.length; i++) {
        Vec3 position = vec3::add(path.path[i].position, lift);
        renderer::point(list, position, 0.1f, {1, 0, 0, 1});
        if (i > 0) renderer::line(list, vec3::add(path.path[i - 1].position, lift), position, {1, 1, 0, 1});
    }

    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
    find_path::release(path);
}

static void init(App &app) {
    FindPathExample &state = find_path_example;
    state = {};
    state.start = {-3.94f, 0.26f, 4.71f};
    state.end = {2.52f, 2.39f, -2.2f};
    state.show_search_nodes = true;

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

    update_path();
}

static void update(App &app, float) {
    FindPathExample &state = find_path_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary || app.click.shift) {
            state.end = app.click.point;
        } else {
            state.start = app.click.point;
        }
        update_path();
    }
    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    FindPathExample &state = find_path_example;
    ImGui::TextWrapped("click to set the start, right click or shift click to set the end");
    if (ImGui::Checkbox("any angle", &state.any_angle)) update_path();
    if (ImGui::Checkbox("show search nodes", &state.show_search_nodes)) update_path();
    ImGui::Text("%s in %.3f ms", state.success ? "found" : "failed", state.query_milliseconds);
    ImGui::Text("%u nodes, %u points", state.node_count, state.point_count);
    if (state.flags & FIND_PATH_RESULT_PARTIAL_PATH) ImGui::Text("partial path");
}

static void shutdown(App &) {
    FindPathExample &state = find_path_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.path_drawing);
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_PATH = {"find path", init, update, ui, shutdown};
