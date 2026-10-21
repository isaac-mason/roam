#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/find_path.h"
#include "roam/math/scalar.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

struct CustomGltfNavMeshExample {
    Model nav_mesh_geometry;
    NavMesh nav_mesh;
    bool generated;
    StaticDrawing nav_mesh_geometry_drawing;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing nav_mesh_links_drawing;
    StaticDrawing path_drawing;
    bool show_nav_mesh_geometry;
    bool show_nav_mesh;
    bool show_nav_mesh_links;
    Vec3 start;
    Vec3 end;
    bool success;
    uint32_t point_count;
    float previous_fov_y;
    SearchWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
};

static CustomGltfNavMeshExample custom_gltf;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void sphere(DrawList &list, Vec3 center, float radius, Color color) {
    const int rings = 8;
    const int segments = 12;
    auto at = [&](int ring, int segment) {
        float phi = (float)ring / rings * scalar::PI;
        float theta = (float)segment / segments * scalar::PI * 2;
        return Vec3{center.x + sinf(phi) * cosf(theta) * radius, center.y + cosf(phi) * radius, center.z + sinf(phi) * sinf(theta) * radius};
    };
    for (int ring = 0; ring < rings; ring++) {
        for (int segment = 0; segment < segments; segment++) {
            renderer::triangle(list, at(ring, segment), at(ring + 1, segment), at(ring + 1, segment + 1), color);
            renderer::triangle(list, at(ring, segment), at(ring + 1, segment + 1), at(ring, segment + 1), color);
        }
    }
}

static void update_path() {
    CustomGltfNavMeshExample &state = custom_gltf;
    renderer::release(state.path_drawing);

    DrawList list = {};
    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, state.nav_mesh, state.start, state.end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER,
                         state.points, {.node_path = state.node_path});
    state.success = path.success;
    state.point_count = path.point_count;

    DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace);
    renderer::add_primitives(list, nodes);
    debug::release(nodes);

    for (uint32_t n = 0; n < path.node_path_count; n++) {
        NodeRef node_ref = state.node_path[n];
        if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(state.nav_mesh, node_ref);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    for (uint32_t i = 0; i < path.point_count; i++) {
        sphere(list, state.points[i].position, 0.2f, {1, 0, 0, 1});
        if (i > 0) renderer::line(list, state.points[i - 1].position, state.points[i].position, {1, 1, 0, 1});
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    CustomGltfNavMeshExample &state = custom_gltf;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.show_nav_mesh = true;
    state.start = {-2.6f, 1.1f, 2.1f};
    state.end = {0.5f, 11.1f, 0.5f};

    app::load_level(app, "tower-big.glb", {.node_name = "level"});
    custom_gltf.previous_fov_y = app.camera.fov_y;
    app.camera.fov_y = 75.0f * scalar::PI / 180.0f;
    camera::look_at(app.camera, {-2, 15, 20}, {0, 6, 0});

    state.nav_mesh_geometry = model::load("tower-big.glb", {.node_name = "navmesh"});
    const Array<float> &positions = state.nav_mesh_geometry.walkable_positions;
    const Array<uint32_t> &indices = state.nav_mesh_geometry.walkable_indices;

    Box3 bounds = box3::empty();
    for (uint32_t i = 0; i < positions.length / 3; i++) bounds = box3::expand_by_point(bounds, vec3::load(positions.data, i * 3));

    Array<ExternalPolygon> polys = {};
    for (uint32_t i = 0; i + 2 < indices.length; i += 3) polys.push({&indices[i], 3, 1, 0});

    NavMeshTilePolys tile_polys =
        nav_mesh_tile::polygons_to_nav_mesh_tile_polys(polys.data, polys.length, positions.data, positions.length / 3, 0, bounds);
    NavMeshTileDetailMesh tile_detail_mesh = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);

    NavMeshTileParams tile_params = {
        .bounds = bounds,
        .vertices = tile_polys.vertices,
        .polys = tile_polys.polys,
        .poly_vertices = tile_polys.poly_vertices,
        .poly_neis = tile_polys.poly_neis,
        .detail_meshes = tile_detail_mesh.detail_meshes,
        .detail_vertices = tile_detail_mesh.detail_vertices,
        .detail_triangles = tile_detail_mesh.detail_triangles,
        .tile_x = 0,
        .tile_y = 0,
        .tile_layer = 0,
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .walkable_height = 0.5f,
        .walkable_radius = 0.5f,
        .walkable_climb = 0.5f,
    };

    state.nav_mesh = nav_mesh_api::create_nav_mesh();
    state.nav_mesh.tile_width = bounds.max.x - bounds.min.x;
    state.nav_mesh.tile_height = bounds.max.z - bounds.min.z;
    state.nav_mesh.origin = bounds.min;
    nav_mesh_api::add_tile(state.nav_mesh, nav_mesh_api::build_tile(tile_params));
    state.generated = true;

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(tile_detail_mesh);
    polys.release();

    DrawList geometry = {};
    const Model &navmesh_model = state.nav_mesh_geometry;
    for (uint32_t i = 0; i + 2 < navmesh_model.indices.length; i += 3) {
        Vec3 a = vec3::load(navmesh_model.positions.data, navmesh_model.indices[i] * 3);
        Vec3 b = vec3::load(navmesh_model.positions.data, navmesh_model.indices[i + 1] * 3);
        Vec3 c = vec3::load(navmesh_model.positions.data, navmesh_model.indices[i + 2] * 3);
        Vec3 color = vec3::load(navmesh_model.colors.data, navmesh_model.indices[i] * 3);
        renderer::triangle(geometry, a, b, c, {color.x, color.y, color.z, 1});
    }
    state.nav_mesh_geometry_drawing = renderer::create_static(geometry);
    renderer::release(geometry);

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.nav_mesh);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    DebugPrimitives links = debug::create_nav_mesh_links_helper(state.nav_mesh);
    state.nav_mesh_links_drawing = renderer::create_static(links);
    debug::release(links);

    update_path();
}

static void update(App &app, float) {
    CustomGltfNavMeshExample &state = custom_gltf;
    if (app.click.clicked) {
        RayHit hit = model::raycast(state.nav_mesh_geometry, app.click.ray);
        if (hit.hit) {
            if (app.click.secondary || app.click.shift) {
                state.end = hit.point;
            } else {
                state.start = hit.point;
            }
            update_path();
        }
    }
    if (state.show_nav_mesh_geometry) app::draw_static(app, state.nav_mesh_geometry_drawing);
    if (state.show_nav_mesh) app::draw_static(app, state.nav_mesh_drawing);
    if (state.show_nav_mesh_links) app::draw_static(app, state.nav_mesh_links_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &app) {
    CustomGltfNavMeshExample &state = custom_gltf;
    ImGui::TextWrapped("click the nav mesh to set the start, right click or shift click to set the end");
    ImGui::Text("%s, %u points", state.success ? "path found" : "no path", state.point_count);
    if (ImGui::CollapsingHeader("Debug Views", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("level", &app.show_level);
        ImGui::Checkbox("nav mesh geometry", &state.show_nav_mesh_geometry);
        ImGui::Checkbox("nav mesh", &state.show_nav_mesh);
        ImGui::Checkbox("nav mesh links", &state.show_nav_mesh_links);
    }
}

static void shutdown(App &app) {
    CustomGltfNavMeshExample &state = custom_gltf;
    renderer::release(state.nav_mesh_geometry_drawing);
    renderer::release(state.nav_mesh_drawing);
    nav_mesh_search::release(state.workspace);
    renderer::release(state.nav_mesh_links_drawing);
    renderer::release(state.path_drawing);
    if (state.generated) nav_mesh_api::release(state.nav_mesh);
    state.generated = false;
    model::release(state.nav_mesh_geometry);
    app.show_level = true;
    app.camera.fov_y = custom_gltf.previous_fov_y;
}

const Example EXAMPLE_CUSTOM_GLTF_NAV_MESH = {"custom gltf nav mesh", init, update, ui, shutdown};
