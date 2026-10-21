#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

struct FindNearestPolyExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing poly_drawing;
    Vec3 query_point;
    bool found;
    Vec3 nearest_position;
    NodeRef nearest_node_ref;
};

static FindNearestPolyExample find_nearest_poly_example;

static void update_nearest_poly(const NavMesh &nav_mesh, Vec3 point) {
    FindNearestPolyExample &state = find_nearest_poly_example;
    state.query_point = point;

    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, point, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!nearest.success) return;

    state.nearest_position = nearest.position;

    if (!state.found || state.nearest_node_ref != nearest.node_ref) {
        renderer::release(state.poly_drawing);
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, nearest.node_ref, {1, 0.5f, 0});
        app::lift(poly, 0.15f);
        state.poly_drawing = renderer::create_static(poly);
        debug::release(poly);
        state.nearest_node_ref = nearest.node_ref;
    }
    state.found = true;
}

static void init(App &app) {
    FindNearestPolyExample &state = find_nearest_poly_example;
    state = {};
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
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    update_nearest_poly(state.result.nav_mesh, {-3.94f, 0.26f, 4.71f});
}

static void update(App &app, float) {
    FindNearestPolyExample &state = find_nearest_poly_example;
    if (app.hover.hit) update_nearest_poly(state.result.nav_mesh, app.hover.point);

    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.poly_drawing);

    DrawList &frame = app::frame(app);
    renderer::point(frame, state.query_point, 0.1f, {0, 0, 1, 1});
    if (state.found) {
        renderer::point(frame, state.nearest_position, 0.1f, {1, 0, 0, 1});
        Vec3 arrow_origin = {state.nearest_position.x, state.nearest_position.y + 1.5f, state.nearest_position.z};
        renderer::arrow_helper(frame, arrow_origin, {0, -1, 0}, 1, {1, 1, 0, 1}, 0.2f, 0.2f * 0.2f);
    }
}

static void ui(App &) {
    FindNearestPolyExample &state = find_nearest_poly_example;
    ImGui::TextWrapped("move the pointer over the level to find the nearest poly");
    if (state.found) {
        ImGui::Text("nearest poly: %llu", (unsigned long long)state.nearest_node_ref);
    } else {
        ImGui::Text("nearest poly: none");
    }
}

static void shutdown(App &) {
    FindNearestPolyExample &state = find_nearest_poly_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.poly_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_NEAREST_POLY = {"find nearest poly", init, update, ui, shutdown};
