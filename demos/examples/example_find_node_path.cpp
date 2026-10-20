#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

using namespace roam;

enum Moving { MOVING_NONE, MOVING_START, MOVING_END };

struct FindNodePathExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing path_drawing;
    Vec3 start;
    Vec3 end;
    Moving moving;
    double query_milliseconds;
    bool searched;
    bool success;
    uint32_t flags;
    uint32_t node_count;
    uint32_t search_node_count;
    SearchWorkspace workspace;
};

static FindNodePathExample find_node_path_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void update_path() {
    FindNodePathExample &state = find_node_path_example;
    renderer::release(state.path_drawing);
    state.searched = false;

    const NavMesh &nav_mesh = state.result.nav_mesh;
    DrawList list = {};
    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    const Vec3 half_extents = {1, 1, 1};
    FindNearestPolyResult start = nav_mesh_api::find_nearest_poly(nav_mesh, state.start, half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER);
    FindNearestPolyResult end = nav_mesh_api::find_nearest_poly(nav_mesh, state.end, half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER);

    if (start.success && end.success) {
        double begin = app::now_milliseconds();
        FindNodePathResult path = nav_mesh_search::find_node_path(nav_mesh, start.node_ref, end.node_ref, start.position, end.position,
                                                                  nav_mesh_api::DEFAULT_QUERY_FILTER, {}, &state.workspace);
        state.query_milliseconds = app::now_milliseconds() - begin;
        state.searched = true;
        state.success = path.success;
        state.flags = path.flags;
        state.node_count = path.path.length;
        state.search_node_count = state.workspace.nodes.nodes.length;

        DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace.nodes);
        renderer::add_primitives(list, nodes);
        debug::release(nodes);

        for (NodeRef node_ref : path.path) {
            if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
            DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
            app::lift(poly, 0.15f);
            renderer::add_primitives(list, poly);
            debug::release(poly);
        }

        nav_mesh_search::release(path);
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    FindNodePathExample &state = find_node_path_example;
    state = {};
    state.start = {-3.94f, 0.26f, 4.71f};
    state.end = {2.52f, 2.39f, -2.2f};
    app.want_hover = true;

    app::load_level(app, "nav-test.glb");
    camera::look_from(app.camera, {-2, 10, 10}, {0, 0, 0}, scalar::degrees_to_radians(75));

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
    FindNodePathExample &state = find_node_path_example;
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
    FindNodePathExample &state = find_node_path_example;
    ImGui::TextWrapped("click to move the start, right click or shift click to move the end, click again to place it");
    if (!state.searched) {
        ImGui::Text("start or end is not on the nav mesh");
        return;
    }
    ImGui::Text("%s in %.3f ms", state.success ? "found" : "failed", state.query_milliseconds);
    ImGui::Text("%u path nodes, %u search nodes", state.node_count, state.search_node_count);
    if (state.flags & FIND_NODE_PATH_RESULT_PARTIAL_PATH) ImGui::Text("partial path");
}

static void shutdown(App &) {
    FindNodePathExample &state = find_node_path_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.path_drawing);
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_NODE_PATH = {"find node path", init, update, ui, shutdown};
