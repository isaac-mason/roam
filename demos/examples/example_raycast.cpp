#include <float.h>
#include <math.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

using namespace roam;

enum RaycastStatus { RAYCAST_STATUS_REACHED_END, RAYCAST_STATUS_REACHED_XZ, RAYCAST_STATUS_BLOCKED };

struct RaycastExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing raycast_drawing;
    Vec3 clicked_start;
    Vec3 clicked_end;
    bool calculate_costs;
    bool valid;
    NodeRef start_node_ref;
    Vec3 start_position;
    NodeRef end_node_ref;
    Vec3 end_position;
    NodeRef hit_node_ref;
    Vec3 hit_position;
    bool has_actual_hit_position;
    Vec3 actual_hit_position;
    RaycastStatus status;
    float path_cost;
    uint32_t path_length;
    float ray_length;
    NodeRef path[MAX_NODE_PATH];
};

static RaycastExample raycast_example;

static Vec3 hsl(float hue, float saturation, float lightness) {
    float chroma = (1 - fabsf(2 * lightness - 1)) * saturation;
    float h = hue * 6.0f;
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
    return {r + m, g + m, b + m};
}

static void add_poly(DrawList &list, const NavMesh &nav_mesh, NodeRef node_ref, Vec3 color, float height) {
    DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref, color);
    app::lift(poly, height);
    renderer::add_primitives(list, poly);
    debug::release(poly);
}

static void perform_raycast() {
    RaycastExample &state = raycast_example;
    renderer::release(state.raycast_drawing);
    state.valid = false;

    const NavMesh &nav_mesh = state.result.nav_mesh;
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, state.clicked_start, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!start.success) return;
    FindNearestPolyResult end = {};
    nav_mesh_api::find_nearest_poly(end, nav_mesh, state.clicked_end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!end.success) return;

    state.valid = true;
    state.start_node_ref = start.node_ref;
    state.start_position = start.position;
    state.end_node_ref = end.node_ref;
    state.end_position = end.position;

    RaycastResult raycast = {};
    if (state.calculate_costs) {
        nav_mesh_search::raycast_with_costs(raycast, nav_mesh, start.node_ref, start.position, end.position, nav_mesh_api::DEFAULT_QUERY_FILTER,
                                            INVALID_NODE_REF, state.path);
    } else {
        nav_mesh_search::raycast(raycast, nav_mesh, start.node_ref, start.position, end.position, nav_mesh_api::DEFAULT_QUERY_FILTER, state.path);
    }

    bool reached_end = raycast.t == FLT_MAX;
    state.path_cost = raycast.path_cost;
    state.path_length = raycast.path_count;

    if (reached_end) {
        state.hit_position = end.position;
        state.hit_node_ref = raycast.path_count > 0 ? state.path[raycast.path_count - 1] : end.node_ref;
    } else {
        state.hit_position = vec3::lerp(start.position, end.position, raycast.t);
        state.hit_node_ref = raycast.path_count > 0 ? state.path[raycast.path_count - 1] : start.node_ref;
    }
    state.ray_length = vec3::distance(start.position, state.hit_position);

    state.has_actual_hit_position = false;
    if (reached_end && state.hit_node_ref != end.node_ref) {
        TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, state.hit_node_ref);
        if (tile_and_poly.success) {
            GetPolyHeightResult height = {};
            nav_mesh_api::get_poly_height(height, *tile_and_poly.tile, *tile_and_poly.poly, tile_and_poly.poly_index, state.hit_position);
            if (height.success) {
                state.has_actual_hit_position = true;
                state.actual_hit_position = {state.hit_position.x, height.height, state.hit_position.z};
            }
        }
    }

    if (reached_end && state.hit_node_ref == end.node_ref) {
        state.status = RAYCAST_STATUS_REACHED_END;
    } else if (reached_end) {
        state.status = RAYCAST_STATUS_REACHED_XZ;
    } else {
        state.status = RAYCAST_STATUS_BLOCKED;
    }

    DrawList list = {};
    const Vec3 lift = {0, 0.5f, 0};

    renderer::point(list, vec3::add(start.position, lift), 0.12f, {0, 0.5f, 0, 1});
    renderer::point(list, vec3::add(end.position, lift), 0.12f, {0, 0, 1, 1});

    const Color hit_colors[] = {{0, 0.5f, 0, 1}, {1, 0.65f, 0, 1}, {1, 0, 0, 1}};
    renderer::point(list, vec3::add(state.hit_position, lift), 0.12f, hit_colors[state.status]);

    if (state.has_actual_hit_position) renderer::point(list, vec3::add(state.actual_hit_position, lift), 0.08f, {1, 1, 0, 1});

    Vec3 actual_hit = state.has_actual_hit_position ? state.actual_hit_position : state.hit_position;
    Vec3 to_hit = vec3::subtract(actual_hit, start.position);
    Vec3 to_end = vec3::subtract(end.position, state.hit_position);
    float to_hit_length = vec3::length(to_hit);
    float to_end_length = vec3::length(to_end);

    if (to_hit_length > 0.01f) {
        renderer::arrow_helper(list, vec3::add(start.position, lift), vec3::scale(to_hit, 1.0f / to_hit_length), to_hit_length, {0, 1, 0, 1}, 0.18f,
                               0.09f);
    }
    if (!reached_end && to_end_length > 0.01f) {
        renderer::arrow_helper(list, vec3::add(state.hit_position, lift), vec3::scale(to_end, 1.0f / to_end_length), to_end_length, {1, 0, 0, 1},
                               0.18f, 0.09f);
    }

    if (raycast.t < FLT_MAX && vec3::length(raycast.hit_normal) > 0) {
        Vec3 normal_end = vec3::scale_and_add(state.hit_position, raycast.hit_normal, 0.5f);
        renderer::thick_line(list, vec3::add(state.hit_position, lift), vec3::add(normal_end, lift), 0.08f, {1, 1, 0, 1});
    }

    add_poly(list, nav_mesh, start.node_ref, {0, 1, 0}, 0.25f);
    add_poly(list, nav_mesh, end.node_ref, {0, 0, 1}, 0.25f);
    if (state.hit_node_ref != end.node_ref) add_poly(list, nav_mesh, state.hit_node_ref, {1, 0.6f, 0}, 0.25f);

    for (uint32_t i = 0; i < raycast.path_count; i++) {
        NodeRef node_ref = state.path[i];
        if (node_ref == start.node_ref || node_ref == end.node_ref || node_ref == state.hit_node_ref) continue;
        add_poly(list, nav_mesh, node_ref, hsl(0.8f, 0.9f, 0.4f + ((float)i / (float)raycast.path_count) * 0.3f), 0.35f);
    }

    state.raycast_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    RaycastExample &state = raycast_example;
    state = {};
    state.clicked_start = {-0.8f, 0.27f, 5.1f};
    state.clicked_end = {0.53f, 0.26f, 3.59f};

    app::load_level(app, "nav-test.glb");
    camera::look_from(app.camera, {0, 5, 8}, {0, 0, 4}, scalar::degrees_to_radians(75));

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

    perform_raycast();
}

static void update(App &app, float) {
    RaycastExample &state = raycast_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary || app.click.shift) {
            state.clicked_end = app.click.point;
        } else {
            state.clicked_start = app.click.point;
        }
        perform_raycast();
    }
    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.raycast_drawing);
}

static void vec3_text(const char *label, Vec3 v) { ImGui::Text("  %s: (%.2f, %.2f, %.2f)", label, v.x, v.y, v.z); }

static void ui(App &) {
    RaycastExample &state = raycast_example;
    ImGui::TextWrapped("click to set the start, right click or shift click to set the end. the raycast is 2D in the XZ plane and follows the nav mesh surface");
    if (ImGui::Checkbox("Calculate Costs", &state.calculate_costs)) perform_raycast();
    if (!state.valid) return;

    const ImVec4 green = {0.4f, 1, 0.4f, 1};
    const ImVec4 blue = {0.4f, 0.6f, 1, 1};
    const ImVec4 orange = {1, 0.67f, 0, 1};
    const ImVec4 red = {1, 0.4f, 0.4f, 1};
    const ImVec4 status_colors[] = {green, orange, red};
    const char *status_texts[] = {"REACHED END", "REACHED XZ (Different Poly)", "BLOCKED"};

    ImGui::TextColored(green, "Start:");
    ImGui::Text("  Poly: %llu", (unsigned long long)state.start_node_ref);
    vec3_text("Pos", state.start_position);
    ImGui::TextColored(blue, "End Target:");
    ImGui::Text("  Poly: %llu", (unsigned long long)state.end_node_ref);
    vec3_text("Pos", state.end_position);
    ImGui::TextColored(status_colors[state.status], "Raycast Hit:");
    ImGui::Text("  Poly: %llu", (unsigned long long)state.hit_node_ref);
    vec3_text("Pos", state.hit_position);
    if (state.has_actual_hit_position) {
        ImGui::TextColored({1, 1, 0, 1}, "Actual Pos on Hit Poly:");
        vec3_text("Pos", state.actual_hit_position);
    }
    ImGui::TextColored(status_colors[state.status], "%s", status_texts[state.status]);

    if (state.calculate_costs) {
        ImGui::Separator();
        ImGui::Text("Path Cost: %.3f", state.path_cost);
        ImGui::Text("Ray Length: %.3f", state.ray_length);
        ImGui::Text("Polys Traversed: %u", state.path_length);
    }
}

static void shutdown(App &) {
    RaycastExample &state = raycast_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.raycast_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_RAYCAST = {"raycast", init, update, ui, shutdown};
