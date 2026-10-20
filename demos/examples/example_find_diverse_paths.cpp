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

static const Color ROUTE_COLORS[] = {{1, 1, 0, 1}, {1, 0.647f, 0, 1}, {0, 1, 1, 1}};

struct FindDiversePathsOptions {
    int k;
    float max_stretch;
    float max_shared;
    float base_penalty;
    int neighbour_depth;
    float neighbour_decay;
    int max_tries;
};

struct DiverseNodePath {
    Array<NodeRef> path;
    Map<NodeRef, uint8_t> node_set;
    float cost;
};

struct PenalizedFilterData {
    const QueryFilter *base;
    const Map<NodeRef, float> *penalty;
};

enum DiversePathsMoving {
    DIVERSE_PATHS_MOVING_NONE,
    DIVERSE_PATHS_MOVING_START,
    DIVERSE_PATHS_MOVING_END,
};

struct FindDiversePathsExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing path_drawing;
    Vec3 start;
    Vec3 end;
    FindDiversePathsOptions options;
    DiversePathsMoving moving;
    uint32_t path_count;
    double query_milliseconds;
};

static FindDiversePathsExample find_diverse_paths_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.533f, 0.533f, 0.533f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void release(DiverseNodePath &node_path) {
    node_path.path.release();
    node_path.node_set.release();
}

static bool penalized_pass_filter(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh) {
    const PenalizedFilterData &data = *(const PenalizedFilterData *)filter.user_data;
    return data.base->pass_filter(*data.base, node_ref, nav_mesh);
}

static float penalized_get_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef previous_ref, NodeRef current_ref,
                                NodeRef next_ref) {
    const PenalizedFilterData &data = *(const PenalizedFilterData *)filter.user_data;
    float base_cost = data.base->get_cost(*data.base, pa, pb, nav_mesh, previous_ref, current_ref, next_ref);
    const float *current_penalty = data.penalty->get(current_ref);
    const float *next_penalty = next_ref != INVALID_NODE_REF ? data.penalty->get(next_ref) : nullptr;
    return base_cost + (current_penalty ? *current_penalty : 0) + 0.5f * (next_penalty ? *next_penalty : 0);
}

static float compute_unpenalized_cost(const NavMesh &nav_mesh, const Array<NodeRef> &path, Vec3 start, Vec3 end, const QueryFilter &base_filter) {
    if (path.length == 0) return INFINITY;

    Array<Vec3> mids = {};
    mids.push(start);
    for (uint32_t i = 0; i + 1 < path.length; i++) {
        Vec3 mid = {0, 0, 0};
        if (nav_mesh_search::get_edge_mid_point(nav_mesh, path[i], path[i + 1], mid)) mids.push(mid);
    }
    mids.push(end);

    uint32_t last = path.length - 1;
    float total = 0;
    for (uint32_t j = 0; j + 1 < mids.length; j++) {
        NodeRef previous_ref = j > 0 ? path[j - 1 < last ? j - 1 : last] : INVALID_NODE_REF;
        NodeRef current_ref = path[j < last ? j : last];
        NodeRef next_ref = j + 1 < path.length ? path[j + 1] : INVALID_NODE_REF;
        total += base_filter.get_cost(base_filter, mids[j], mids[j + 1], nav_mesh, previous_ref, current_ref, next_ref);
    }
    mids.release();
    return total;
}

static Map<NodeRef, uint8_t> node_set_of(const Array<NodeRef> &path) {
    Map<NodeRef, uint8_t> set = {};
    for (NodeRef node_ref : path) set.set(node_ref, 1);
    return set;
}

static float jaccard(const Map<NodeRef, uint8_t> &a, const Map<NodeRef, uint8_t> &b) {
    uint32_t intersection = 0;
    for (uint32_t i = 0; i < a.capacity; i++) {
        if (a.slots[i] == MAP_SLOT_OCCUPIED && b.has(a.keys[i])) intersection++;
    }
    uint32_t union_count = a.length + b.length - intersection;
    return union_count == 0 ? 0 : (float)intersection / (float)union_count;
}

static void add_penalty(Map<NodeRef, float> &penalty, NodeRef node_ref, float amount) {
    const float *existing = penalty.get(node_ref);
    penalty.set(node_ref, (existing ? *existing : 0) + amount);
}

static void spread_penalty(const NavMesh &nav_mesh, Map<NodeRef, float> &penalty, const Array<NodeRef> &path, const FindDiversePathsOptions &options) {
    Map<NodeRef, uint8_t> seen = {};
    Array<NodeRef> seen_order = {};
    auto see = [&](NodeRef node_ref) {
        if (seen.has(node_ref)) return;
        seen.set(node_ref, 1);
        seen_order.push(node_ref);
    };

    for (NodeRef node_ref : path) {
        add_penalty(penalty, node_ref, options.base_penalty);
        see(node_ref);
    }

    Array<NodeRef> frontier = {};
    for (int depth = 1; depth <= options.neighbour_depth; depth++) {
        float amount = options.base_penalty * powf(options.neighbour_decay, (float)depth);
        frontier.clear();
        uint32_t seen_count = seen_order.length;
        for (uint32_t i = 0; i < seen_count; i++) {
            const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, seen_order[i]);
            for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
                NodeRef neighbour = nav_mesh.links[(uint32_t)link].to_node_ref;
                if (!seen.has(neighbour)) frontier.push(neighbour);
            }
        }
        for (NodeRef node_ref : frontier) {
            add_penalty(penalty, node_ref, amount);
            see(node_ref);
        }
    }

    frontier.release();
    seen.release();
    seen_order.release();
}

static Array<DiverseNodePath> find_diverse_paths(const NavMesh &nav_mesh, Vec3 start, Vec3 end, Vec3 half_extents, const QueryFilter &base_filter,
                                                 const FindDiversePathsOptions &options) {
    Array<DiverseNodePath> accepted = {};

    FindNearestPolyResult start_nearest = nav_mesh_api::find_nearest_poly(nav_mesh, start, half_extents, base_filter);
    if (!start_nearest.success) return accepted;
    FindNearestPolyResult end_nearest = nav_mesh_api::find_nearest_poly(nav_mesh, end, half_extents, base_filter);
    if (!end_nearest.success) return accepted;

    NodeRef start_node_ref = start_nearest.node_ref;
    NodeRef end_node_ref = end_nearest.node_ref;

    FindNodePathResult best = nav_mesh_search::find_node_path(nav_mesh, start_node_ref, end_node_ref, start, end, base_filter);
    if (!best.success) {
        nav_mesh_search::release(best);
        return accepted;
    }

    Map<NodeRef, float> penalty = {};
    float best_cost = compute_unpenalized_cost(nav_mesh, best.path, start, end, base_filter);
    accepted.push({best.path, node_set_of(best.path), best_cost});
    best.path = {};
    nav_mesh_search::release(best);
    spread_penalty(nav_mesh, penalty, accepted[0].path, options);

    PenalizedFilterData data = {&base_filter, &penalty};
    QueryFilter penalized = {penalized_pass_filter, penalized_get_cost, base_filter.include_flags, base_filter.exclude_flags, &data};

    int tries = 0;
    while ((int)accepted.length < options.k && tries < options.max_tries) {
        tries++;
        FindNodePathResult candidate = nav_mesh_search::find_node_path(nav_mesh, start_node_ref, end_node_ref, start, end, penalized);
        if (!candidate.success) {
            nav_mesh_search::release(candidate);
            continue;
        }

        float cost = compute_unpenalized_cost(nav_mesh, candidate.path, start, end, base_filter);
        if (cost > (1 + options.max_stretch) * best_cost) {
            nav_mesh_search::release(candidate);
            continue;
        }

        Map<NodeRef, uint8_t> nodes = node_set_of(candidate.path);
        bool too_similar = false;
        for (const DiverseNodePath &existing : accepted) too_similar = too_similar || jaccard(existing.node_set, nodes) > options.max_shared;
        if (too_similar) {
            nodes.release();
            nav_mesh_search::release(candidate);
            continue;
        }

        accepted.push({candidate.path, nodes, cost});
        candidate.path = {};
        nav_mesh_search::release(candidate);
        spread_penalty(nav_mesh, penalty, accepted.back().path, options);
    }

    sort::stable(accepted.data, accepted.length, [](const DiverseNodePath &a, const DiverseNodePath &b) { return a.cost < b.cost; });
    while ((int)accepted.length > options.k) {
        DiverseNodePath removed = accepted.pop();
        release(removed);
    }

    penalty.release();
    return accepted;
}

static void update_path() {
    FindDiversePathsExample &state = find_diverse_paths_example;
    renderer::release(state.path_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;

    DrawList list = {};
    flag(list, state.start, {0.129f, 0.588f, 0.953f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    double started = app::now_milliseconds();
    Array<DiverseNodePath> node_paths =
        find_diverse_paths(nav_mesh, state.start, state.end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, state.options);
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

        FindStraightPathResult straight = find_straight_path::find_straight_path(nav_mesh, state.start, state.end, node_path.data, node_path.length);
        if (straight.success) {
            const Vec3 lift = {0, 0.1f, 0};
            for (uint32_t i = 0; i < straight.path.length; i++) {
                Vec3 position = vec3::add(straight.path[i].position, lift);
                renderer::point(list, position, path_index == 0 ? 0.2f : 0.15f, route_color);
                if (i > 0) renderer::line(list, vec3::add(straight.path[i - 1].position, lift), position, route_color);
            }
        }
        find_straight_path::release(straight);
    }

    for (DiverseNodePath &node_path : node_paths) release(node_path);
    node_paths.release();

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    FindDiversePathsExample &state = find_diverse_paths_example;
    state = {};
    state.start = {-3.94f, 0.26f, 4.71f};
    state.end = {2.52f, 2.39f, -2.2f};
    state.options = {3, 0.2f, 0.65f, 120, 2, 0.5f, 20};
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
    FindDiversePathsExample &state = find_diverse_paths_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary) {
            state.moving = state.moving == DIVERSE_PATHS_MOVING_END ? DIVERSE_PATHS_MOVING_NONE : DIVERSE_PATHS_MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == DIVERSE_PATHS_MOVING_START ? DIVERSE_PATHS_MOVING_NONE : DIVERSE_PATHS_MOVING_START;
            state.start = app.click.point;
        }
        update_path();
    } else if (state.moving != DIVERSE_PATHS_MOVING_NONE && app.hover.hit) {
        if (state.moving == DIVERSE_PATHS_MOVING_START) {
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
    FindDiversePathsExample &state = find_diverse_paths_example;
    ImGui::TextWrapped("click to move the start, right click to move the end, click again to place it");
    FindDiversePathsOptions &options = state.options;
    bool changed = false;
    if (ImGui::CollapsingHeader("Path Finding", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderInt("number of paths", &options.k, 1, 10);
        changed |= ImGui::SliderFloat("max stretch", &options.max_stretch, 0, 1, "%.2f");
        changed |= ImGui::SliderFloat("max shared", &options.max_shared, 0, 1, "%.2f");
    }
    if (ImGui::CollapsingHeader("Penalty Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderFloat("base penalty", &options.base_penalty, 0, 500, "%.0f");
        changed |= ImGui::SliderInt("neighbour depth", &options.neighbour_depth, 0, 5);
        changed |= ImGui::SliderFloat("neighbour decay", &options.neighbour_decay, 0, 1, "%.2f");
    }
    if (ImGui::CollapsingHeader("Advanced", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderInt("max tries", &options.max_tries, 1, 50);
    }
    if (changed) update_path();
    ImGui::Text("found %u routes in %.3f ms", state.path_count, state.query_milliseconds);
}

static void shutdown(App &) {
    FindDiversePathsExample &state = find_diverse_paths_example;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.path_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FIND_DIVERSE_PATHS = {"find diverse paths", init, update, ui, shutdown};
