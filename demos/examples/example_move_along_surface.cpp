#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

using namespace roam;

enum Moving { MOVING_NONE, MOVING_START, MOVING_END };

struct MoveAlongSurfaceExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing move_drawing;
    Vec3 start;
    Vec3 end;
    Moving moving;
    double query_milliseconds;
    bool on_nav_mesh;
    bool success;
    uint32_t visited_count;
    float requested_distance;
    float actual_distance;
    SearchWorkspace workspace;
    NodeRef visited[MAX_NODE_PATH];
};

static MoveAlongSurfaceExample move_along_surface_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void update_move_along_surface() {
    MoveAlongSurfaceExample &state = move_along_surface_example;
    renderer::release(state.move_drawing);

    const NavMesh &nav_mesh = state.result.nav_mesh;
    DrawList list = {};
    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, state.start, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    state.on_nav_mesh = start.success;

    if (start.success) {
        double begin = app::now_milliseconds();
        MoveAlongSurfaceResult result = {};
        nav_mesh_search::move_along_surface(result, state.workspace, nav_mesh, start.node_ref, state.start, state.end,
                                            nav_mesh_api::DEFAULT_QUERY_FILTER, state.visited);
        state.query_milliseconds = app::now_milliseconds() - begin;
        state.success = result.success;
        state.visited_count = result.visited_count;
        state.requested_distance = vec3::distance(state.start, state.end);
        state.actual_distance = vec3::distance(state.start, result.position);

        renderer::point(list, result.position, 0.2f, {1, 0, 0, 1});
        renderer::line(list, state.start, state.end, {1, 1, 0, 1});
        renderer::line(list, state.start, result.position, {0, 1, 1, 1});

        for (uint32_t i = 0; i < result.visited_count; i++) {
            DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, state.visited[i]);
            app::lift(poly, 0.3f);
            renderer::add_primitives(list, poly);
            debug::release(poly);
        }
    }

    state.move_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    MoveAlongSurfaceExample &state = move_along_surface_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.start = {-3.641054110753002f, 1.3775759935379028f, 2.7671564887530034f};
    state.end = {-2.857031378804761f, 0.2670568953640924f, 3.088813745131878f};
    app.want_hover = true;

    app::load_level(app, "nav-test.glb");
    camera::look_from(app.camera, {-2, 10, 10}, {0, 0, 0}, scalar::degrees_to_radians(75));

    NavMeshConfig config = nav_mesh_config::create();
    config.tile_size_voxels = 64;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions, level.walkable_indices}, nav_mesh_config::tiled_options(config));
    state.generated = true;

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    update_move_along_surface();
}

static void update(App &app, float) {
    MoveAlongSurfaceExample &state = move_along_surface_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary || app.click.shift) {
            state.moving = state.moving == MOVING_END ? MOVING_NONE : MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == MOVING_START ? MOVING_NONE : MOVING_START;
            state.start = app.click.point;
        }
        update_move_along_surface();
    } else if (state.moving != MOVING_NONE && app.hover.hit) {
        Vec3 &moved = state.moving == MOVING_START ? state.start : state.end;
        if (!vec3::exactly_equals(moved, app.hover.point)) {
            moved = app.hover.point;
            update_move_along_surface();
        }
    }
    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.move_drawing);
}

static void ui(App &) {
    MoveAlongSurfaceExample &state = move_along_surface_example;
    ImGui::TextWrapped("click to move the start, right click or shift click to move the end, click again to place it");
    ImGui::TextColored({0.13f, 0.59f, 0.95f, 1}, "blue flag: start position");
    ImGui::TextColored({0, 1, 0, 1}, "green flag: end position");
    ImGui::TextColored({1, 0, 0, 1}, "red sphere: result position");
    ImGui::TextColored({1, 1, 0, 1}, "yellow line: requested movement");
    ImGui::TextColored({0, 1, 1, 1}, "cyan line: actual constrained movement");
    ImGui::Separator();
    if (!state.on_nav_mesh) {
        ImGui::Text("start position not on the nav mesh");
        return;
    }
    ImGui::Text("%s in %.3f ms", state.success ? "moved" : "failed", state.query_milliseconds);
    ImGui::Text("%u visited polys", state.visited_count);
    ImGui::Text("requested %.2f, actual %.2f", state.requested_distance, state.actual_distance);
}

static void shutdown(App &) {
    MoveAlongSurfaceExample &state = move_along_surface_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.move_drawing);
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_MOVE_ALONG_SURFACE = {"move along surface", init, update, ui, shutdown};
