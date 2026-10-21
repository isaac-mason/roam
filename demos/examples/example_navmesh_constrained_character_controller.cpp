#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"
#include "sokol_app.h"

using namespace roam;

struct CharacterControllerExample {
    bool generated;
    SoloNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    bool show_nav_mesh_helper;
    bool show_agent_helper;
    float cell_size;
    float cell_height;
    float walkable_radius;
    float walkable_slope_angle;
    float walkable_climb;
    float walkable_height;
    float walking_speed;
    float running_speed;
    float offset_above;
    float offset_behind;
    Vec3 position;
    float rotation;
    Vec3 movement;
    bool sprinting;
    bool first_position_update;
    Vec3 camera_position;
    SearchWorkspace workspace;
};

static CharacterControllerExample character_controller_example;

static void release() {
    CharacterControllerExample &state = character_controller_example;
    renderer::release(state.nav_mesh_drawing);
    if (state.generated) {
        generate_solo_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

static void generate_nav_mesh(const Model &level) {
    CharacterControllerExample &state = character_controller_example;
    release();

    SoloNavMeshOptions options = {
        .cell_size = state.cell_size,
        .cell_height = state.cell_height,
        .walkable_radius_voxels = (int)ceilf(state.walkable_radius / state.cell_size),
        .walkable_radius_world = state.walkable_radius,
        .walkable_climb_voxels = (int)ceilf(state.walkable_climb / state.cell_height),
        .walkable_climb_world = state.walkable_climb,
        .walkable_height_voxels = (int)ceilf(state.walkable_height / state.cell_height),
        .walkable_height_world = state.walkable_height,
        .walkable_slope_angle_degrees = state.walkable_slope_angle,
        .border_size = 4,
        .min_region_area = 12,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 6,
        .detail_sample_max_error = 1,
    };
    state.result = generate_solo_nav_mesh::generate_solo_nav_mesh(
        {level.walkable_positions, level.walkable_indices}, options);
    state.generated = true;

    DebugPrimitives helper = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(helper, 0.15f);
    state.nav_mesh_drawing = renderer::create_static(helper);
    debug::release(helper);
}

static void init(App &app) {
    CharacterControllerExample &state = character_controller_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.show_nav_mesh_helper = true;
    state.cell_size = 0.2f;
    state.cell_height = 0.2f;
    state.walkable_radius = 0.3f;
    state.walkable_slope_angle = 45;
    state.walkable_climb = 0.4f;
    state.walkable_height = 1.5f;
    state.walking_speed = 4;
    state.running_speed = 8;
    state.offset_above = 15;
    state.offset_behind = 10;
    state.position = {0, 2, 0};
    state.first_position_update = true;
    state.camera_position = {5, 8, 5};

    app::load_level(app, "game-level.glb");
    camera::look_at(app.camera, state.camera_position, {0, 0, 0});

    generate_nav_mesh(app.level);

    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, state.result.nav_mesh, {0, 1, 0}, {5, 5, 5}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (nearest.success) state.position = nearest.position;
}

static void movement_update(const App &app, float delta) {
    CharacterControllerExample &state = character_controller_example;
    bool forward = app.keys[SAPP_KEYCODE_W] || app.keys[SAPP_KEYCODE_UP];
    bool back = app.keys[SAPP_KEYCODE_S] || app.keys[SAPP_KEYCODE_DOWN];
    bool left = app.keys[SAPP_KEYCODE_A] || app.keys[SAPP_KEYCODE_LEFT];
    bool right = app.keys[SAPP_KEYCODE_D] || app.keys[SAPP_KEYCODE_RIGHT];
    bool sprint = app.keys[SAPP_KEYCODE_LEFT_SHIFT] || app.keys[SAPP_KEYCODE_RIGHT_SHIFT];

    Vec3 movement = {0, 0, 0};
    if (forward) movement.z -= 1;
    if (back) movement.z += 1;
    if (left) movement.x -= 1;
    if (right) movement.x += 1;

    float speed = sprint ? state.running_speed : state.walking_speed;
    float length = vec3::length(movement);
    movement = length > 0 ? vec3::scale(movement, speed * delta / length) : movement;
    state.movement = movement;

    if (vec3::length(movement) > 0 || state.first_position_update) {
        const NavMesh &nav_mesh = state.result.nav_mesh;
        Vec3 target = vec3::add(state.position, movement);
        FindNearestPolyResult nearest = {};
        nav_mesh_api::find_nearest_poly(nearest, nav_mesh, state.position, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        if (nearest.success) {
            MoveAlongSurfaceResult moved = {};
            nav_mesh_search::move_along_surface(moved, state.workspace, nav_mesh, nearest.node_ref, state.position, target,
                                                nav_mesh_api::DEFAULT_QUERY_FILTER, {});
            if (moved.success) state.position = moved.position;
        }
        state.first_position_update = false;
    }

    state.sprinting = sprint;
}

static void rotation_update(float delta) {
    CharacterControllerExample &state = character_controller_example;
    if (vec3::length(state.movement) <= 0) return;
    float t = 1.0f - powf(0.01f, delta);
    float target = atan2f(state.movement.x, state.movement.z);
    float difference = remainderf(target - state.rotation, scalar::PI * 2);
    state.rotation += difference * fminf(t * 5, 1);
}

static void camera_update(App &app, float delta) {
    CharacterControllerExample &state = character_controller_example;
    Vec3 offset = {0, state.offset_above, state.offset_behind};
    Vec3 target = vec3::add(state.position, offset);
    float t = 1.0f - powf(0.01f, delta);
    state.camera_position = vec3::lerp(state.camera_position, target, t / 1.1f);
    camera::look_at(app.camera, state.camera_position, vec3::subtract(state.camera_position, offset));
}

static void draw_character(DrawList &list) {
    CharacterControllerExample &state = character_controller_example;
    Vec3 base = state.position;
    renderer::cylinder(list, base, 0.25f, 1.6f, {0.55f, 0.27f, 0.2f, 1});
    Vec3 facing = {sinf(state.rotation), 0, cosf(state.rotation)};
    Vec3 chest = vec3::add(base, {0, 1.2f, 0});
    renderer::box(list, vec3::scale_and_add(chest, facing, 0.25f), {0.08f, 0.08f, 0.08f}, {0.9f, 0.8f, 0.6f, 1});

    if (state.show_agent_helper) {
        float radius = state.walkable_radius;
        float half_height = state.walkable_height * 0.5f;
        Vec3 center = vec3::add(base, {0, 0.9f, 0});
        Color red = {1, 0, 0, 1};
        Vec3 bottom = vec3::add(center, {0, -half_height, 0});
        Vec3 top = vec3::add(center, {0, half_height, 0});
        renderer::circle(list, bottom, radius, red);
        renderer::circle(list, top, radius, red);
        renderer::circle(list, center, radius, red);
        for (int i = 0; i < 8; i++) {
            float angle = (float)i / 8 * scalar::PI * 2;
            Vec3 side = {cosf(angle) * radius, 0, sinf(angle) * radius};
            renderer::line(list, vec3::add(bottom, side), vec3::add(top, side), red);
            renderer::line(list, vec3::add(top, side), vec3::add(top, {0, radius, 0}), red);
            renderer::line(list, vec3::add(bottom, side), vec3::add(bottom, {0, -radius, 0}), red);
        }
    }
}

static void update(App &app, float delta_time) {
    CharacterControllerExample &state = character_controller_example;
    float delta = fminf(delta_time, 0.1f);

    movement_update(app, delta);
    rotation_update(delta);
    camera_update(app, delta);

    if (state.show_nav_mesh_helper) app::draw_static(app, state.nav_mesh_drawing);
    draw_character(app::frame(app));
}

static void ui(App &app) {
    CharacterControllerExample &state = character_controller_example;
    ImGui::TextWrapped("WASD or the arrow keys to move, shift to run");
    if (ImGui::CollapsingHeader("Nav Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("show helper", &state.show_nav_mesh_helper);
        ImGui::Checkbox("show agent helper", &state.show_agent_helper);
        ImGui::SliderFloat("cell size", &state.cell_size, 0.05f, 0.3f, "%.2f");
        ImGui::SliderFloat("cell height", &state.cell_height, 0.05f, 0.3f, "%.2f");
        ImGui::SliderFloat("walkable radius", &state.walkable_radius, 0.1f, 1, "%.1f");
        ImGui::SliderFloat("walkable slope angle", &state.walkable_slope_angle, 0, 90, "%.0f");
        ImGui::SliderFloat("walkable climb", &state.walkable_climb, 0.1f, 1, "%.1f");
        ImGui::SliderFloat("walkable height", &state.walkable_height, 0.1f, 3, "%.1f");
        if (ImGui::Button("generate nav mesh")) generate_nav_mesh(app.level);
    }
    if (ImGui::CollapsingHeader("Player Speed")) {
        ImGui::SliderFloat("walking speed", &state.walking_speed, 0.1f, 50, "%.1f");
        ImGui::SliderFloat("running speed", &state.running_speed, 0.1f, 50, "%.1f");
    }
    if (ImGui::CollapsingHeader("Camera")) {
        ImGui::SliderFloat("offset behind", &state.offset_behind, 5, 30, "%.0f");
        ImGui::SliderFloat("offset above", &state.offset_above, 2, 15, "%.0f");
    }
}

static void shutdown(App &) {
    CharacterControllerExample &state = character_controller_example;
    release();
    nav_mesh_search::release(state.workspace);
}

const Example EXAMPLE_NAVMESH_CONSTRAINED_CHARACTER_CONTROLLER = {"navmesh constrained character controller", init, update, ui, shutdown};
