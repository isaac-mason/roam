#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

using namespace roam;

constexpr int MAX_RANDOM_POINTS = 500;

struct FindRandomPointAroundCircleExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    uint32_t random_state;
    float radius;
    int point_count;
    bool has_center;
    Vec3 center;
    Array<Vec3> points;
    SearchWorkspace workspace;
};

static FindRandomPointAroundCircleExample find_random_point_around_circle_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static void update_random_points(Vec3 point) {
    FindRandomPointAroundCircleExample &state = find_random_point_around_circle_example;
    const NavMesh &nav_mesh = state.result.nav_mesh;

    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, point, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!nearest.success) return;

    state.has_center = true;
    state.center = nearest.position;
    state.points.clear();

    for (int i = 0; i < state.point_count; i++) {
        FindRandomPointResult result = {};
        nav_mesh_search::find_random_point_around_circle(result, state.workspace, nav_mesh, nearest.node_ref, nearest.position, state.radius,
                                                         nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (result.success) state.points.push(result.position);
    }
}

static void init(App &app) {
    FindRandomPointAroundCircleExample &state = find_random_point_around_circle_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.random_state = 42;
    state.radius = 1.0f;
    state.point_count = 100;

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

    update_random_points({-3.94f, 0.26f, 4.71f});
}

static void update(App &app, float) {
    FindRandomPointAroundCircleExample &state = find_random_point_around_circle_example;
    if (app.click.clicked && app.click.hit) update_random_points(app.click.point);

    app::draw_static(app, state.nav_mesh_drawing);

    DrawList &frame = app::frame(app);
    for (Vec3 point : state.points) renderer::point(frame, point, 0.05f, {1, 0, 0, 1});
    if (state.has_center) {
        Vec3 arrow_origin = {state.center.x, state.center.y + 1.5f, state.center.z};
        renderer::arrow_helper(frame, arrow_origin, {0, -1, 0}, 1, {1, 1, 0, 1}, 0.2f, 0.2f * 0.2f);
    }
}

static void ui(App &) {
    FindRandomPointAroundCircleExample &state = find_random_point_around_circle_example;
    ImGui::TextWrapped("click the level to find random points around a circle");
    if (ImGui::CollapsingHeader("Random Points", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Radius", &state.radius, 0.1f, 10.0f, "%.2f");
        ImGui::SliderInt("Number of Points", &state.point_count, 1, MAX_RANDOM_POINTS);
    }
    ImGui::Text("%u points", state.points.length);
}

static void shutdown(App &) {
    FindRandomPointAroundCircleExample &state = find_random_point_around_circle_example;
    renderer::release(state.nav_mesh_drawing);
    state.points.release();
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_RANDOM_POINT_AROUND_CIRCLE = {"find random point around circle", init, update, ui, shutdown};
