#include <float.h>
#include <math.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/blocks/search/flood_fill_nav_mesh.h"
#include "roam/debug/debug.h"
#include "roam/generate/common.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

static const Vec3 POLY_COLOR = {0.3f, 0.8f, 0.3f};
static const Vec3 COLOR_KEPT = {0.2f, 0.8f, 0.2f};
static const Vec3 COLOR_PRUNED = {0.8f, 0.2f, 0.2f};

struct FloodFillPruningExample {
    bool generated;
    TiledNavMeshResult result;
    bool pruned;
    NavMesh pruned_nav_mesh;
    Map<NodeRef, uint8_t> reachable;
    StaticDrawing polys_drawing;
};

static FloodFillPruningExample flood_fill_pruning_example;

static bool sanitize_tile_polys(const NavMeshTile &tile, const Array<bool> &keep, NavMeshTileParams &params) {
    Array<int> poly_remap = {};
    poly_remap.resize(tile.polys.length, -1);
    Array<uint32_t> survivors = {};
    for (uint32_t i = 0; i < tile.polys.length; i++) {
        if (!keep[i]) continue;
        poly_remap[i] = (int)survivors.length;
        survivors.push(i);
    }

    if (survivors.length == 0) {
        poly_remap.release();
        survivors.release();
        return false;
    }

    params = {};
    Map<uint32_t, uint32_t> vertex_remap = {};
    auto remap_vertex = [&](uint32_t old_vertex) {
        const uint32_t *existing = vertex_remap.get(old_vertex);
        if (existing) return *existing;
        uint32_t new_vertex = params.vertices.length / 3;
        vertex_remap.set(old_vertex, new_vertex);
        params.vertices.push(tile.vertices[old_vertex * 3]);
        params.vertices.push(tile.vertices[old_vertex * 3 + 1]);
        params.vertices.push(tile.vertices[old_vertex * 3 + 2]);
        return new_vertex;
    };

    for (uint32_t old_poly : survivors) {
        const NavMeshPoly &poly = tile.polys[old_poly];

        NavMeshPoly sanitized = {params.poly_vertices.length, poly.vertex_count, poly.flags, poly.area};
        for (uint32_t j = 0; j < poly.vertex_count; j++) {
            params.poly_vertices.push(remap_vertex(tile.poly_vertices[poly.vertex_base + j]));
            int nei = tile.poly_neis[poly.vertex_base + j];
            if (nei != 0 && (nei & POLY_NEIS_FLAG_EXT_LINK) == 0) {
                int new_neighbour = poly_remap[(uint32_t)nei - 1];
                nei = new_neighbour == -1 ? 0 : new_neighbour + 1;
            }
            params.poly_neis.push(nei);
        }
        params.polys.push(sanitized);

        const NavMeshPolyDetail &detail = tile.detail_meshes[old_poly];
        uint32_t vertices_base = params.detail_vertices.length / 3;
        uint32_t triangles_base = params.detail_triangles.length / 4;

        for (uint32_t v = 0; v < detail.vertices_count * 3; v++) params.detail_vertices.push(tile.detail_vertices[detail.vertices_base * 3 + v]);
        for (uint32_t t = 0; t < detail.triangles_count * 4; t++) params.detail_triangles.push(tile.detail_triangles[detail.triangles_base * 4 + t]);

        params.detail_meshes.push({vertices_base, triangles_base, detail.vertices_count, detail.triangles_count});
    }

    params.tile_x = tile.tile_x;
    params.tile_y = tile.tile_y;
    params.tile_layer = tile.tile_layer;
    params.bounds = tile.bounds;
    params.cell_size = tile.cell_size;
    params.cell_height = tile.cell_height;
    params.walkable_height = tile.walkable_height;
    params.walkable_radius = tile.walkable_radius;
    params.walkable_climb = tile.walkable_climb;

    vertex_remap.release();
    poly_remap.release();
    survivors.release();
    return true;
}

static void release(NavMeshTileParams &params) {
    params.vertices.release();
    params.polys.release();
    params.poly_vertices.release();
    params.poly_neis.release();
    params.detail_meshes.release();
    params.detail_vertices.release();
    params.detail_triangles.release();
}

static NavMesh prune_nav_mesh(const NavMesh &nav_mesh, const Map<NodeRef, uint8_t> &keep) {
    NavMesh result = nav_mesh_api::create_nav_mesh();
    result.origin = nav_mesh.origin;
    result.tile_width = nav_mesh.tile_width;
    result.tile_height = nav_mesh.tile_height;

    Array<bool> keep_poly = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        keep_poly.clear();
        for (uint32_t node_index : tile.poly_nodes) keep_poly.push(keep.has(nav_mesh.nodes[node_index].ref));
        NavMeshTileParams params = {};
        if (sanitize_tile_polys(tile, keep_poly, params)) {
            nav_mesh_api::add_tile(result, nav_mesh_api::build_tile(params));
            release(params);
        }
    }
    keep_poly.release();
    return result;
}

static uint32_t count_tiles(const NavMesh &nav_mesh) {
    uint32_t count = 0;
    for (const NavMeshTile &tile : nav_mesh.tiles) count += tile.allocated ? 1 : 0;
    return count;
}

static uint32_t count_polys(const NavMesh &nav_mesh) {
    uint32_t count = 0;
    for (const NavMeshTile &tile : nav_mesh.tiles) count += tile.allocated ? tile.polys.length : 0;
    return count;
}

static float srgb_to_linear(float c) { return c < 0.04045f ? c * 0.0773993808f : powf(c * 0.9478672986f + 0.0521327014f, 2.4f); }

static float linear_to_srgb(float c) { return c < 0.0031308f ? c * 12.92f : 1.055f * powf(c, 0.41666f) - 0.055f; }

static void tint(DebugPrimitives &primitives, Vec3 color, float opacity) {
    for (DebugPrimitive &primitive : primitives) {
        for (uint32_t i = 0; i + 2 < primitive.colors.length; i += 3) {
            primitive.colors[i] = linear_to_srgb(primitive.colors[i] * srgb_to_linear(color.x));
            primitive.colors[i + 1] = linear_to_srgb(primitive.colors[i + 1] * srgb_to_linear(color.y));
            primitive.colors[i + 2] = linear_to_srgb(primitive.colors[i + 2] * srgb_to_linear(color.z));
        }
        primitive.transparent = opacity < 1;
        primitive.opacity = opacity;
    }
}

static void update_polys_drawing() {
    FloodFillPruningExample &state = flood_fill_pruning_example;
    renderer::release(state.polys_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;

    DrawList list = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
            NodeRef node_ref = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, tile, (int)poly_index).ref;
            DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref, POLY_COLOR);
            app::lift(poly, 0.1f);
            if (state.pruned) {
                bool kept = state.reachable.has(node_ref);
                tint(poly, kept ? COLOR_KEPT : COLOR_PRUNED, kept ? 1 : 0.35f);
            }
            renderer::add_primitives(list, poly);
            debug::release(poly);
        }
    }
    state.polys_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void clear_prune() {
    FloodFillPruningExample &state = flood_fill_pruning_example;
    if (state.pruned) nav_mesh_api::release_nav_mesh(state.pruned_nav_mesh);
    state.pruned = false;
    state.reachable.clear();
}

static float ray_triangle(Ray ray, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 ab = vec3::subtract(b, a);
    Vec3 ac = vec3::subtract(c, a);
    Vec3 p = vec3::cross(ray.direction, ac);
    float determinant = vec3::dot(ab, p);
    if (fabsf(determinant) < 1e-9f) return -1;
    float inverse = 1.0f / determinant;
    Vec3 s = vec3::subtract(ray.origin, a);
    float u = vec3::dot(s, p) * inverse;
    if (u < 0 || u > 1) return -1;
    Vec3 q = vec3::cross(s, ab);
    float v = vec3::dot(ray.direction, q) * inverse;
    if (v < 0 || u + v > 1) return -1;
    return vec3::dot(ac, q) * inverse;
}

static NodeRef find_clicked_polygon(const NavMesh &nav_mesh, Ray ray) {
    float closest_distance = FLT_MAX;
    NodeRef closest_poly_ref = INVALID_NODE_REF;
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
            const NavMeshPoly &poly = tile.polys[poly_index];
            auto vertex = [&](uint32_t i) {
                Vec3 position = vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3);
                return Vec3{position.x, position.y + 0.1f, position.z};
            };
            for (uint32_t i = 1; i + 1 < poly.vertex_count; i++) {
                float distance = ray_triangle(ray, vertex(0), vertex(i), vertex(i + 1));
                if (distance > 0 && distance < closest_distance) {
                    closest_distance = distance;
                    closest_poly_ref = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, tile, (int)poly_index).ref;
                }
            }
        }
    }
    return closest_poly_ref;
}

static void init(App &app) {
    FloodFillPruningExample &state = flood_fill_pruning_example;
    state = {};

    app::load_level(app, "nav-test.glb");

    NavMeshConfig config = nav_mesh_config::create();
    config.border_size = 4;
    config.tile_size_voxels = 64;
    const Model &level = app.level;
    state.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(config));
    state.generated = true;

    update_polys_drawing();
}

static void update(App &app, float) {
    FloodFillPruningExample &state = flood_fill_pruning_example;
    if (app.click.clicked) {
        const NavMesh &nav_mesh = state.result.nav_mesh;
        NodeRef clicked_poly_ref = find_clicked_polygon(nav_mesh, app.click.ray);
        if (clicked_poly_ref != INVALID_NODE_REF) {
            clear_prune();
            Array<NodeRef> reachable = {};
            reachable.resize(nav_mesh_api::get_max_node_count(nav_mesh));
            FloodFillNavMeshResult flood = {};
            flood_fill_nav_mesh::flood_fill_nav_mesh(flood, nav_mesh, &clicked_poly_ref, 1, reachable);
            reachable.resize(flood.reachable_count);
            for (NodeRef ref : reachable) state.reachable.set(ref, 1);
            reachable.release();
            state.pruned_nav_mesh = prune_nav_mesh(nav_mesh, state.reachable);
            state.pruned = true;
            update_polys_drawing();
        }
    }
    app::draw_static(app, state.polys_drawing);
}

static void ui(App &) {
    FloodFillPruningExample &state = flood_fill_pruning_example;
    ImGui::TextWrapped("click a poly to prune everything unreachable from it");
    const NavMesh &nav_mesh = state.result.nav_mesh;
    if (state.pruned) {
        uint32_t total_polys = count_polys(nav_mesh);
        uint32_t kept_polys = count_polys(state.pruned_nav_mesh);
        ImGui::Text("tiles: %u -> %u", count_tiles(nav_mesh), count_tiles(state.pruned_nav_mesh));
        ImGui::Text("polys: %u -> %u", total_polys, kept_polys);
        ImGui::Text("kept (green): %u", kept_polys);
        ImGui::Text("removed (red): %u", total_polys - kept_polys);
    } else {
        ImGui::Text("tiles: %u", count_tiles(nav_mesh));
        ImGui::Text("polys: %u", count_polys(nav_mesh));
        ImGui::TextDisabled("kept (green): -");
        ImGui::TextDisabled("removed (red): -");
    }
    if (ImGui::Button("reset navmesh")) {
        clear_prune();
        update_polys_drawing();
    }
}

static void shutdown(App &) {
    FloodFillPruningExample &state = flood_fill_pruning_example;
    clear_prune();
    state.reachable.release();
    renderer::release(state.polys_drawing);
    if (state.generated) {
        generate_tiled_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release_nav_mesh(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_FLOOD_FILL_PRUNING = {"flood fill pruning", init, update, ui, shutdown};
