#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/util/map.h"

using namespace roam;

struct FlowFieldEntry {
    NodeRef node_ref;
    int cost;
    NodeRef next;
};

struct FlowField {
    Array<FlowFieldEntry> entries;
    Map<NodeRef, uint32_t> entry_by_ref;
};

enum InteractionMode {
    INTERACTION_IDLE,
    INTERACTION_MOVING_FLOW_FIELD,
    INTERACTION_MOVING_PATH,
};

struct FlowFieldExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing flow_field_drawing;
    StaticDrawing path_drawing;
    FlowField flow_field;
    bool has_flow_field;
    Vec3 flow_field_target;
    bool has_path_start;
    Vec3 path_start;
    bool path_found;
    InteractionMode mode;
    Vec3 last_hover;
    double flow_field_milliseconds;
    double path_milliseconds;
};

static FlowFieldExample flow_field_example;

static const int MAX_ITERATIONS = 1000;

static void release_flow_field(FlowField &flow_field) {
    flow_field.entries.release();
    flow_field.entry_by_ref.release();
}

static void add_flow_field_entry(FlowField &flow_field, NodeRef node_ref, int cost, NodeRef next) {
    flow_field.entry_by_ref.set(node_ref, flow_field.entries.length);
    flow_field.entries.push({node_ref, cost, next});
}

static FlowField compute_uniform_cost_flow_field(const NavMesh &nav_mesh, NodeRef target_ref, const QueryFilter &filter, int max_iterations) {
    FlowField flow_field = {};
    add_flow_field_entry(flow_field, target_ref, 0, INVALID_NODE_REF);

    uint32_t queue_head = 0;
    int iterations = 0;
    while (queue_head < flow_field.entries.length && iterations < max_iterations) {
        FlowFieldEntry current = flow_field.entries[queue_head++];
        iterations++;

        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, current.node_ref);
        for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
            NodeRef neighbour_ref = nav_mesh.links[(uint32_t)link].to_node_ref;
            if (flow_field.entry_by_ref.has(neighbour_ref)) continue;
            if (!filter.pass_filter(filter, neighbour_ref, nav_mesh)) continue;
            add_flow_field_entry(flow_field, neighbour_ref, current.cost + 1, current.node_ref);
        }
    }

    return flow_field;
}

static bool get_node_path_from_flow_field(const FlowField &flow_field, NodeRef start_node_ref, Array<NodeRef> &out_path) {
    out_path.clear();
    Map<NodeRef, bool> visited = {};
    NodeRef current = start_node_ref;
    bool reached_target = false;

    while (current != INVALID_NODE_REF && flow_field.entry_by_ref.has(current) && !visited.has(current)) {
        out_path.push(current);
        visited.set(current, true);
        NodeRef next = flow_field.entries[*flow_field.entry_by_ref.get(current)].next;
        if (next == INVALID_NODE_REF) {
            reached_target = true;
            break;
        }
        current = next;
    }

    visited.release();
    return reached_target;
}

static bool get_poly_center(const NavMesh &nav_mesh, NodeRef node_ref, Vec3 &out_center) {
    if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) return false;
    TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(node_ref, nav_mesh);
    if (!tile_and_poly.success) return false;
    const NavMeshTile &tile = *tile_and_poly.tile;
    const NavMeshPoly &poly = *tile_and_poly.poly;

    Vec3 sum = {0, 0, 0};
    for (uint32_t i = 0; i < poly.vertex_count; i++) {
        sum = vec3::add(sum, vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3));
    }
    out_center = vec3::scale(sum, 1.0f / (float)poly.vertex_count);
    return true;
}

static void build_flow_field_drawing() {
    FlowFieldExample &state = flow_field_example;
    renderer::release(state.flow_field_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;
    const FlowField &flow_field = state.flow_field;

    int min_cost = INT32_MAX;
    int max_cost = INT32_MIN;
    for (const FlowFieldEntry &entry : flow_field.entries) {
        if (entry.cost < min_cost) min_cost = entry.cost;
        if (entry.cost > max_cost) max_cost = entry.cost;
    }
    if (min_cost == max_cost) max_cost = min_cost + 1;

    DrawList list = {};
    for (const FlowFieldEntry &entry : flow_field.entries) {
        float t = (float)(entry.cost - min_cost) / (float)(max_cost - min_cost);
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, entry.node_ref, {1 - t, 0, t});
        app::lift(poly, 0.15f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    for (const FlowFieldEntry &entry : flow_field.entries) {
        if (entry.next == INVALID_NODE_REF) continue;
        Vec3 center_a;
        Vec3 center_b;
        if (!get_poly_center(nav_mesh, entry.node_ref, center_a) || !get_poly_center(nav_mesh, entry.next, center_b)) continue;
        Vec3 direction = vec3::subtract(center_b, center_a);
        float length = vec3::length(direction);
        if (length < 0.01f) continue;
        direction = vec3::scale(direction, 1.0f / length);
        Vec3 origin = vec3::add(center_a, {0, 0.3f, 0});
        renderer::arrow(list, origin, vec3::scale_and_add(origin, direction, fmaxf(length * 0.7f, 0.3f)), {1, 1, 1, 1});
    }

    state.flow_field_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void clear_path() {
    FlowFieldExample &state = flow_field_example;
    renderer::release(state.path_drawing);
    state.path_found = false;
}

static void update_path(Vec3 position) {
    FlowFieldExample &state = flow_field_example;
    if (!state.has_flow_field) return;
    const NavMesh &nav_mesh = state.result.nav_mesh;

    FindNearestPolyResult start = nav_mesh_api::find_nearest_poly(nav_mesh, position, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!start.success) return;

    state.has_path_start = true;
    state.path_start = start.position;

    FindNearestPolyResult end = nav_mesh_api::find_nearest_poly(nav_mesh, state.flow_field_target, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);

    double begin = app::now_milliseconds();
    Array<NodeRef> poly_path = {};
    bool found = get_node_path_from_flow_field(state.flow_field, start.node_ref, poly_path);
    state.path_milliseconds = app::now_milliseconds() - begin;

    if (!found || !end.success) {
        poly_path.release();
        clear_path();
        return;
    }

    FindStraightPathResult straight = find_straight_path::find_straight_path(nav_mesh, start.position, end.position, poly_path.data, poly_path.length);

    renderer::release(state.path_drawing);
    DrawList list = {};
    for (NodeRef node_ref : poly_path) {
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref, {1, 0.7f, 0.2f});
        app::lift(poly, 0.15f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }
    const Vec3 lift = {0, 0.3f, 0};
    for (uint32_t i = 0; i < straight.path.length; i++) {
        Vec3 point = vec3::add(straight.path[i].position, lift);
        renderer::point(list, point, 0.15f, {1, 0, 0, 1});
        if (i > 0) renderer::line(list, vec3::add(straight.path[i - 1].position, lift), point, {1, 1, 0, 1});
    }
    state.path_drawing = renderer::create_static(list);
    state.path_found = true;
    renderer::release(list);

    find_straight_path::release(straight);
    poly_path.release();
}

static void update_flow_field(Vec3 position) {
    FlowFieldExample &state = flow_field_example;
    const NavMesh &nav_mesh = state.result.nav_mesh;

    FindNearestPolyResult result = nav_mesh_api::find_nearest_poly(nav_mesh, position, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!result.success) return;

    state.flow_field_target = result.position;

    double begin = app::now_milliseconds();
    release_flow_field(state.flow_field);
    state.flow_field = compute_uniform_cost_flow_field(nav_mesh, result.node_ref, nav_mesh_api::DEFAULT_QUERY_FILTER, MAX_ITERATIONS);
    state.flow_field_milliseconds = app::now_milliseconds() - begin;
    state.has_flow_field = true;

    build_flow_field_drawing();

    if (state.has_path_start) {
        update_path(state.path_start);
    } else {
        clear_path();
    }
}

static void init(App &app) {
    FlowFieldExample &state = flow_field_example;
    state = {};
    app.want_hover = true;

    app::load_level(app, "nav-test.glb");
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});

    NavMeshConfig config = nav_mesh_config::create();
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(config));
    state.generated = true;

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    update_flow_field({-3.94f, 0.26f, 4.71f});
    update_path({2.52f, 2.39f, -2.2f});
}

static void flag(DrawList &list, Vec3 position) {
    renderer::box(list, vec3::add(position, {0, 0.6f, 0}), {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, vec3::add(position, {0.23f, 1.0f, 0}), {0.16f, 0.11f, 0.02f}, {0, 1, 0, 1});
}

static void update(App &app, float) {
    FlowFieldExample &state = flow_field_example;

    if (app.click.clicked) {
        InteractionMode toggled = app.click.secondary ? INTERACTION_MOVING_PATH : INTERACTION_MOVING_FLOW_FIELD;
        state.mode = state.mode == toggled ? INTERACTION_IDLE : toggled;
    }

    if (state.mode != INTERACTION_IDLE && app.hover.hit && vec3::distance(app.hover.point, state.last_hover) > 1e-4f) {
        state.last_hover = app.hover.point;
        if (state.mode == INTERACTION_MOVING_FLOW_FIELD) {
            update_flow_field(app.hover.point);
        } else {
            update_path(app.hover.point);
        }
    }

    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.flow_field_drawing);
    app::draw_static(app, state.path_drawing);
    if (state.has_flow_field) flag(app::frame(app), state.flow_field_target);
}

static void ui(App &) {
    FlowFieldExample &state = flow_field_example;
    ImGui::TextWrapped("click to start or stop moving the flow field target with the mouse, right click to start or stop moving the path start");
    const char *mode = state.mode == INTERACTION_MOVING_FLOW_FIELD ? "moving flow field target"
                       : state.mode == INTERACTION_MOVING_PATH    ? "moving path start"
                                                                  : "idle";
    ImGui::Text("mode: %s", mode);
    ImGui::Text("flow field: %u polys in %.3f ms", state.flow_field.entries.length, state.flow_field_milliseconds);
    ImGui::Text("path: %s in %.3f ms", state.path_found ? "found" : "none", state.path_milliseconds);
}

static void shutdown(App &) {
    FlowFieldExample &state = flow_field_example;
    release_flow_field(state.flow_field);
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.flow_field_drawing);
    renderer::release(state.path_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FLOW_FIELD_PATHFINDING = {"flow field pathfinding", init, update, ui, shutdown};
