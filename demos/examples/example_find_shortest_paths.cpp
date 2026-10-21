#include <math.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"
#include "roam/util/sort.h"

using namespace roam;

static const Color ROUTE_COLORS[] = {
    {1, 1, 0, 1},     {1, 0.647f, 0, 1},     {0, 1, 1, 1},         {1, 0, 1, 1},     {0, 1, 0, 1},
    {0, 1, 1, 1},     {1, 0.753f, 0.796f, 1}, {1, 0.843f, 0, 1}, {0.98f, 0.502f, 0.447f, 1}, {0.933f, 0.51f, 0.933f, 1},
};

struct ShortestNodePath {
    Array<NodeRef> path;
    float cost;
};

struct ShortestPathEdge {
    NodeRef from;
    NodeRef to;
};

struct BlockingFilterData {
    const QueryFilter *base;
    const Map<NodeRef, uint8_t> *blocked_nodes;
    const Array<ShortestPathEdge> *blocked_edges;
};

enum ShortestPathsMoving {
    SHORTEST_PATHS_MOVING_NONE,
    SHORTEST_PATHS_MOVING_START,
    SHORTEST_PATHS_MOVING_END,
};

struct FindShortestPathsExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing path_drawing;
    Vec3 start;
    Vec3 end;
    int k;
    ShortestPathsMoving moving;
    uint32_t path_count;
    double query_milliseconds;
    SearchWorkspace workspace;
    NodeRef node_path[MAX_NODE_PATH];
    StraightPathPoint points[MAX_PATH_POINTS];
};

static FindShortestPathsExample find_shortest_paths_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.533f, 0.533f, 0.533f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static bool has_edge(const Array<ShortestPathEdge> &edges, NodeRef from, NodeRef to) {
    for (const ShortestPathEdge &edge : edges) {
        if (edge.from == from && edge.to == to) return true;
    }
    return false;
}

static bool blocking_pass_filter(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh) {
    const BlockingFilterData &data = *(const BlockingFilterData *)filter.user_data;
    if (data.blocked_nodes->has(node_ref)) return false;
    return data.base->pass_filter(*data.base, node_ref, nav_mesh);
}

static float blocking_get_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef previous_ref, NodeRef current_ref,
                               NodeRef next_ref) {
    const BlockingFilterData &data = *(const BlockingFilterData *)filter.user_data;
    if (next_ref != INVALID_NODE_REF && has_edge(*data.blocked_edges, current_ref, next_ref)) return INFINITY;
    return data.base->get_cost(*data.base, pa, pb, nav_mesh, previous_ref, current_ref, next_ref);
}

static bool paths_equal(const Array<NodeRef> &a, const Array<NodeRef> &b) {
    if (a.length != b.length) return false;
    for (uint32_t i = 0; i < a.length; i++) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

static float calculate_path_cost(const NavMesh &nav_mesh, const Array<NodeRef> &path, Vec3 start, Vec3 end) {
    if (path.length == 0) return INFINITY;
    if (path.length == 1) return vec3::distance(start, end);

    float total_cost = 0;
    Vec3 previous_position = start;
    for (uint32_t i = 0; i < path.length; i++) {
        Vec3 current_position = previous_position;
        if (i == path.length - 1) {
            current_position = end;
        } else {
            GetEdgeMidPointResult midpoint;
            nav_mesh_search::get_edge_mid_point(midpoint, nav_mesh, path[i], path[i + 1]);
            if (midpoint.success) current_position = midpoint.position;
        }
        total_cost += vec3::distance(previous_position, current_position);
        previous_position = current_position;
    }
    return total_cost;
}

static Array<ShortestNodePath> find_shortest_paths(SearchWorkspace &workspace, Span<NodeRef> buffer, const NavMesh &nav_mesh, Vec3 start, Vec3 end,
                                                   Vec3 half_extents, const QueryFilter &base_filter, int k) {
    Array<ShortestNodePath> accepted = {};
    Array<ShortestNodePath> candidates = {};

    FindNearestPolyResult start_nearest = {};
    nav_mesh_api::find_nearest_poly(start_nearest, nav_mesh, start, half_extents, base_filter);
    if (!start_nearest.success) return accepted;
    FindNearestPolyResult end_nearest = {};
    nav_mesh_api::find_nearest_poly(end_nearest, nav_mesh, end, half_extents, base_filter);
    if (!end_nearest.success) return accepted;

    NodeRef start_node_ref = start_nearest.node_ref;
    NodeRef end_node_ref = end_nearest.node_ref;

    FindNodePathResult first_path = {};
    nav_mesh_search::find_node_path(first_path, workspace, nav_mesh, start_node_ref, end_node_ref, start, end, base_filter, buffer);
    if (!first_path.success) return accepted;
    Array<NodeRef> first_node_path = {};
    for (uint32_t i = 0; i < first_path.path_count; i++) first_node_path.push(buffer[i]);
    ShortestNodePath first = {first_node_path, calculate_path_cost(nav_mesh, first_node_path, start, end)};
    accepted.push(first);

    Map<NodeRef, uint8_t> blocked_nodes = {};
    Array<ShortestPathEdge> blocked_edges = {};
    BlockingFilterData data = {&base_filter, &blocked_nodes, &blocked_edges};
    QueryFilter spur_filter = {blocking_pass_filter, blocking_get_cost, base_filter.include_flags, base_filter.exclude_flags, &data};

    for (int k_iter = 1; k_iter < k; k_iter++) {
        const Array<NodeRef> &previous_path = accepted[(uint32_t)k_iter - 1].path;

        for (uint32_t i = 0; i + 1 < previous_path.length; i++) {
            NodeRef spur_node = previous_path[i];

            blocked_edges.clear();
            blocked_nodes.clear();

            for (uint32_t j = 0; j < i; j++) blocked_nodes.set(previous_path[j], 1);

            for (const ShortestNodePath &route : accepted) {
                const Array<NodeRef> &p = route.path;
                bool shares_root = true;
                for (uint32_t j = 0; j <= i && j < p.length; j++) {
                    if (p[j] != previous_path[j]) {
                        shares_root = false;
                        break;
                    }
                }
                if (shares_root && i + 1 < p.length && !has_edge(blocked_edges, p[i], p[i + 1])) blocked_edges.push({p[i], p[i + 1]});
            }

            Vec3 spur_node_position = start;
            if (i != 0) {
                GetEdgeMidPointResult midpoint;
                nav_mesh_search::get_edge_mid_point(midpoint, nav_mesh, spur_node, previous_path[i + 1]);
                spur_node_position = midpoint.position;
            }

            FindNodePathResult spur = {};
            nav_mesh_search::find_node_path(spur, workspace, nav_mesh, spur_node, end_node_ref, spur_node_position, end, spur_filter, buffer);
            if (spur.success) {
                Array<NodeRef> total_path = {};
                for (uint32_t j = 0; j <= i; j++) total_path.push(previous_path[j]);
                for (uint32_t j = 1; j < spur.path_count; j++) total_path.push(buffer[j]);

                bool duplicate = false;
                for (const ShortestNodePath &route : accepted) duplicate = duplicate || paths_equal(route.path, total_path);
                for (const ShortestNodePath &candidate : candidates) duplicate = duplicate || paths_equal(candidate.path, total_path);

                if (duplicate) {
                    total_path.release();
                } else {
                    candidates.push({total_path, calculate_path_cost(nav_mesh, total_path, start, end)});
                }
            }
        }

        if (candidates.length == 0) break;

        sort::stable(candidates.data, candidates.length, [](const ShortestNodePath &a, const ShortestNodePath &b) { return a.cost < b.cost; });
        ShortestNodePath next = candidates[0];
        for (uint32_t i = 1; i < candidates.length; i++) candidates[i - 1] = candidates[i];
        candidates.pop();
        accepted.push(next);
    }

    for (ShortestNodePath &candidate : candidates) candidate.path.release();
    candidates.release();
    blocked_nodes.release();
    blocked_edges.release();
    return accepted;
}

static void update_path() {
    FindShortestPathsExample &state = find_shortest_paths_example;
    renderer::release(state.path_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;

    DrawList list = {};
    flag(list, state.start, {0.129f, 0.588f, 0.953f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    double started = app::now_milliseconds();
    Array<ShortestNodePath> node_paths = find_shortest_paths(state.workspace, state.node_path, nav_mesh, state.start, state.end, {1, 1, 1},
                                                             nav_mesh_api::DEFAULT_QUERY_FILTER, state.k);
    state.query_milliseconds = app::now_milliseconds() - started;
    state.path_count = node_paths.length;

    for (uint32_t path_index = 0; path_index < node_paths.length; path_index++) {
        const Array<NodeRef> &node_path = node_paths[path_index].path;
        Color route_color = path_index < sizeof(ROUTE_COLORS) / sizeof(ROUTE_COLORS[0]) ? ROUTE_COLORS[path_index] : Color{1, 1, 1, 1};

        if (path_index == 0) {
            for (NodeRef node_ref : node_path) {
                if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
                DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
                app::lift(poly, 0.15f);
                renderer::add_primitives(list, poly);
                debug::release(poly);
            }
        }

        FindStraightPathResult straight = {};
        find_straight_path::find_straight_path(straight, nav_mesh, state.start, state.end, node_path, state.points);
        if (straight.success) {
            const Vec3 lift = {0, 0.1f, 0};
            for (uint32_t i = 0; i < straight.point_count; i++) {
                Vec3 position = vec3::add(state.points[i].position, lift);
                renderer::point(list, position, path_index == 0 ? 0.2f : 0.15f, route_color);
                if (i > 0) renderer::line(list, vec3::add(state.points[i - 1].position, lift), position, route_color);
            }
        }
    }

    for (ShortestNodePath &node_path : node_paths) node_path.path.release();
    node_paths.release();

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    FindShortestPathsExample &state = find_shortest_paths_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.start = {-3.94f, 0.26f, 4.71f};
    state.end = {2.52f, 2.39f, -2.2f};
    state.k = 3;
    app.want_hover = true;

    app::load_level(app, "nav-test.glb");

    NavMeshConfig config = nav_mesh_config::create();
    config.border_size = 4;
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
    FindShortestPathsExample &state = find_shortest_paths_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary) {
            state.moving = state.moving == SHORTEST_PATHS_MOVING_END ? SHORTEST_PATHS_MOVING_NONE : SHORTEST_PATHS_MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == SHORTEST_PATHS_MOVING_START ? SHORTEST_PATHS_MOVING_NONE : SHORTEST_PATHS_MOVING_START;
            state.start = app.click.point;
        }
        update_path();
    } else if (state.moving != SHORTEST_PATHS_MOVING_NONE && app.hover.hit) {
        if (state.moving == SHORTEST_PATHS_MOVING_START) {
            state.start = app.hover.point;
        } else {
            state.end = app.hover.point;
        }
        update_path();
    }
    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    FindShortestPathsExample &state = find_shortest_paths_example;
    ImGui::TextWrapped("click to move the start, right click to move the end, click again to place it");
    if (ImGui::CollapsingHeader("Algorithm Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SliderInt("k (number of paths)", &state.k, 1, 10)) update_path();
    }
    ImGui::Text("found %u paths in %.3f ms", state.path_count, state.query_milliseconds);
}

static void shutdown(App &) {
    FindShortestPathsExample &state = find_shortest_paths_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.path_drawing);
    nav_mesh_search::release(state.workspace);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_SHORTEST_PATHS = {"find shortest paths", init, update, ui, shutdown};
