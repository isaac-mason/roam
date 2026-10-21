#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum OffMeshConnectionAreaType : uint8_t {
    OFF_MESH_CONNECTION_AREA_TELEPORTER = 1,
    OFF_MESH_CONNECTION_AREA_JUMP = 2,
    OFF_MESH_CONNECTION_AREA_CLIMB = 3,
};

static const OffMeshConnectionParams OFF_MESH_CONNECTIONS[] = {
    {
        .start = {-2.4799404316645157f, 0.26716880587122915f, 4.039628947351325f},
        .end = {-2.735661224133032f, 2.3264200687408447f, 0.9084349415865054f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = OFF_MESH_CONNECTION_AREA_TELEPORTER,
    },
    {
        .start = {0.43153271761444056f, 3.788429404449852f, 2.549912418335899f},
        .end = {1.6203363597139502f, 2.7055995008052136f, 3.3892644209191634f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = OFF_MESH_CONNECTION_AREA_JUMP,
    },
    {
        .start = {0.5997826320925559f, 0.2668087168256541f, 4.967287730406272f},
        .end = {1.580858144475107f, 3.112976869830365f, 4.670723413649996f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = OFF_MESH_CONNECTION_AREA_CLIMB,
    },
    {
        .start = {3.54f, 0.27f, -3.89f},
        .end = {6.09f, 0.69f, -3.59f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = 0,
    },
    {
        .start = {6.09f, 0.69f, -3.59f},
        .end = {6.55f, 0.39f, -0.68f},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffff,
        .area = 0,
    },
};

enum OffMeshConnectionsMoving {
    OFF_MESH_CONNECTIONS_MOVING_NONE,
    OFF_MESH_CONNECTIONS_MOVING_START,
    OFF_MESH_CONNECTIONS_MOVING_END,
};

struct OffMeshConnectionsExample {
    bool generated;
    TiledNavMeshResult result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing off_mesh_connections_drawing;
    StaticDrawing path_drawing;
    QueryFilter filter;
    Vec3 start;
    Vec3 end;
    OffMeshConnectionsMoving moving;
    bool success;
    uint32_t flags;
    uint32_t straight_path_flags;
    uint32_t node_path_flags;
    uint32_t node_count;
    uint32_t off_mesh_node_count;
    uint32_t point_count;
    SearchWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
};

static OffMeshConnectionsExample off_mesh_connections_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.533f, 0.533f, 0.533f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static bool pass_all(const QueryFilter &, NodeRef, const NavMesh &) { return true; }

static float off_mesh_connection_cost(const QueryFilter &, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef, NodeRef current_ref, NodeRef) {
    if (current_ref != INVALID_NODE_REF && node::get_node_ref_type(current_ref) == NODE_TYPE_OFFMESH) {
        uint8_t area = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref).area;
        if (area == OFF_MESH_CONNECTION_AREA_JUMP) return vec3::distance(pa, pb);
        if (area == OFF_MESH_CONNECTION_AREA_CLIMB) return vec3::distance(pa, pb) * 4;
        if (area == OFF_MESH_CONNECTION_AREA_TELEPORTER) return 1;
    }
    return vec3::distance(pa, pb);
}

static void update_path() {
    OffMeshConnectionsExample &state = off_mesh_connections_example;
    renderer::release(state.path_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;

    DrawList list = {};
    flag(list, state.start, {0.129f, 0.588f, 0.953f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, nav_mesh, state.start, state.end, {1, 1, 1}, state.filter, state.points,
                         {.node_path = state.node_path});
    state.success = path.success;
    state.flags = path.flags;
    state.straight_path_flags = path.straight_path_flags;
    state.node_path_flags = path.node_path_flags;
    state.node_count = path.node_path_count;
    state.off_mesh_node_count = 0;
    state.point_count = path.point_count;

    if (path.success) {
        DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace);
        renderer::add_primitives(list, nodes);
        debug::release(nodes);

        for (uint32_t n = 0; n < path.node_path_count; n++) {
            NodeRef node_ref = state.node_path[n];
            if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) {
                state.off_mesh_node_count++;
                continue;
            }
            DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
            app::lift(poly, 0.15f);
            renderer::add_primitives(list, poly);
            debug::release(poly);
        }

        for (uint32_t i = 0; i < path.point_count; i++) {
            Vec3 position = state.points[i].position;
            renderer::point(list, position, 0.2f, {1, 0, 0, 1});
            if (i > 0) renderer::line(list, state.points[i - 1].position, position, {1, 1, 0, 1});
        }
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    OffMeshConnectionsExample &state = off_mesh_connections_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.filter = {pass_all, off_mesh_connection_cost, 0xffffffffu, 0, nullptr};
    state.start = {-2.2f, 0.26f, 4.71f};
    state.end = {3.4f, 2.8f, 3.6f};
    app.want_hover = true;

    app::load_level(app, "nav-test.glb");

    NavMeshConfig config = nav_mesh_config::create();
    config.border_size = 4;
    config.tile_size_voxels = 64;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(config));
    state.generated = true;

    for (const OffMeshConnectionParams &connection : OFF_MESH_CONNECTIONS) nav_mesh_api::add_off_mesh_connection(state.result.nav_mesh, connection);

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    DebugPrimitives off_mesh_connections = debug::create_nav_mesh_off_mesh_connections_helper(state.result.nav_mesh);
    state.off_mesh_connections_drawing = renderer::create_static(off_mesh_connections);
    debug::release(off_mesh_connections);

    update_path();
}

static void update(App &app, float) {
    OffMeshConnectionsExample &state = off_mesh_connections_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary) {
            state.moving = state.moving == OFF_MESH_CONNECTIONS_MOVING_END ? OFF_MESH_CONNECTIONS_MOVING_NONE : OFF_MESH_CONNECTIONS_MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == OFF_MESH_CONNECTIONS_MOVING_START ? OFF_MESH_CONNECTIONS_MOVING_NONE : OFF_MESH_CONNECTIONS_MOVING_START;
            state.start = app.click.point;
        }
        update_path();
    } else if (state.moving != OFF_MESH_CONNECTIONS_MOVING_NONE && app.hover.hit) {
        if (state.moving == OFF_MESH_CONNECTIONS_MOVING_START) {
            state.start = app.hover.point;
        } else {
            state.end = app.hover.point;
        }
        update_path();
    }
    app::draw_static(app, state.nav_mesh_drawing);
    app::draw_static(app, state.off_mesh_connections_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    OffMeshConnectionsExample &state = off_mesh_connections_example;
    ImGui::TextWrapped("click to move the start, right click to move the end, click again to place it");
    if (ImGui::CollapsingHeader("Path Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextColored({0.129f, 0.588f, 0.953f, 1}, "start");
        ImGui::Text("x %.2f  y %.2f  z %.2f", state.start.x, state.start.y, state.start.z);
        ImGui::TextColored({0, 1, 0, 1}, "end");
        ImGui::Text("x %.2f  y %.2f  z %.2f", state.end.x, state.end.y, state.end.z);
        ImGui::TextColored({1, 1, 0, 1}, "path info");
        ImGui::Text("success %s, flags %u", state.success ? "true" : "false", state.flags);
        ImGui::Text("node path flags %u, straight path flags %u", state.node_path_flags, state.straight_path_flags);
        ImGui::Text("%u nodes, %u off mesh connections", state.node_count, state.off_mesh_node_count);
        for (uint32_t i = 0; i < state.point_count; i++) {
            const StraightPathPoint &point = state.points[i];
            ImGui::Text("%u %s (%.2f, %.2f, %.2f)", i, point.type == NODE_TYPE_OFFMESH ? "offmesh" : "poly", point.position.x, point.position.y,
                        point.position.z);
        }
    }
}

static void shutdown(App &) {
    OffMeshConnectionsExample &state = off_mesh_connections_example;
    renderer::release(state.nav_mesh_drawing);
    nav_mesh_search::release(state.workspace);
    renderer::release(state.off_mesh_connections_drawing);
    renderer::release(state.path_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_OFF_MESH_CONNECTIONS = {"off mesh connections", init, update, ui, shutdown};
