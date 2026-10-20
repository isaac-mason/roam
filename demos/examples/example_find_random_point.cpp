#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

using namespace roam;

struct FindRandomPointExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    uint32_t random_state;
    bool found;
    Vec3 position;
};

static FindRandomPointExample find_random_point_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static void update_random_point() {
    FindRandomPointExample &state = find_random_point_example;
    FindRandomPointResult point =
        nav_mesh_search::find_random_point(state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
    if (!point.success) return;
    state.found = true;
    state.position = point.position;
}

static void init(App &app) {
    FindRandomPointExample &state = find_random_point_example;
    state = {};
    state.random_state = 42;

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

    update_random_point();
}

static void update(App &app, float) {
    FindRandomPointExample &state = find_random_point_example;
    app::draw_static(app, state.nav_mesh_drawing);
    if (!state.found) return;

    DrawList &frame = app::frame(app);
    renderer::point(frame, state.position, 0.1f, {1, 0, 0, 1});
    Vec3 arrow_origin = {state.position.x, state.position.y + 1.5f, state.position.z};
    renderer::arrow_helper(frame, arrow_origin, {0, -1, 0}, 1, {1, 1, 0, 1}, 0.2f, 0.2f * 0.2f);
}

static void ui(App &) {
    ImGui::TextWrapped("press the button to find a random point on the nav mesh");
    if (ImGui::CollapsingHeader("Random Point", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Find Random Point")) update_random_point();
    }
}

static void shutdown(App &) {
    FindRandomPointExample &state = find_random_point_example;
    renderer::release(state.nav_mesh_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_RANDOM_POINT = {"find random point", init, update, ui, shutdown};
