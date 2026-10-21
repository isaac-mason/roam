#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "roam/math/scalar.h"
#include "roam/util/map.h"

using namespace roam;

struct ChunkyTriMeshExample {
    ChunkyTriMesh chunky_tri_mesh;
    double create_milliseconds;
    bool show_triangles;
    bool show_chunk_bounds;
    bool show_query_region;
    bool wireframe;
    float query_region_x;
    float query_region_z;
    float query_region_size;
    float triangle_in_query_color[3];
    float triangle_out_query_color[3];
    float chunk_bounds_color[3];
    float query_region_color[3];
    uint32_t triangles_in_query;
    uint32_t leaf_count;
    StaticDrawing drawing;
    float previous_fov_y;
};

static ChunkyTriMeshExample chunky;

static Color to_color(const float *rgb, float alpha = 1) { return {rgb[0], rgb[1], rgb[2], alpha}; }

static uint64_t triangle_key(uint32_t a, uint32_t b, uint32_t c) { return ((uint64_t)a << 42) | ((uint64_t)b << 21) | (uint64_t)c; }

static void rect_edges(DrawList &list, float min_x, float min_z, float max_x, float max_z, Color color) {
    const Vec3 corners[4] = {{min_x, 0.05f, min_z}, {max_x, 0.05f, min_z}, {max_x, 0.05f, max_z}, {min_x, 0.05f, max_z}};
    for (int i = 0; i < 4; i++) renderer::line(list, corners[i], corners[(i + 1) % 4], color);
}

static void visualize_triangles(const App &app, DrawList &list, Vec2 query_min, Vec2 query_max) {
    chunky.triangles_in_query = 0;
    if (!chunky.show_triangles) return;

    const float *positions = app.level.walkable_positions.data;
    const Array<uint32_t> &indices = app.level.walkable_indices;

    Array<uint32_t> in_query = chunky_tri_mesh::get_triangles_in_rect(chunky.chunky_tri_mesh, query_min, query_max);
    Map<uint64_t, uint8_t> in_query_set = {};
    for (uint32_t j = 0; j + 2 < in_query.length; j += 3) in_query_set.set(triangle_key(in_query[j], in_query[j + 1], in_query[j + 2]), 1);

    Color in_color = to_color(chunky.triangle_in_query_color);
    Color out_color = to_color(chunky.triangle_out_query_color);

    for (uint32_t i = 0; i + 2 < indices.length; i += 3) {
        bool is_in_query = in_query_set.has(triangle_key(indices[i], indices[i + 1], indices[i + 2]));
        if (is_in_query) chunky.triangles_in_query++;
        Color color = is_in_query ? in_color : out_color;
        Vec3 a = vec3::load(positions, indices[i] * 3);
        Vec3 b = vec3::load(positions, indices[i + 1] * 3);
        Vec3 c = vec3::load(positions, indices[i + 2] * 3);
        if (chunky.wireframe) {
            renderer::line(list, a, b, color);
            renderer::line(list, b, c, color);
            renderer::line(list, c, a, color);
        } else {
            renderer::triangle(list, a, b, c, color);
        }
    }

    in_query_set.release();
    in_query.release();
}

static void visualize_chunk_bounds(DrawList &list, Vec2 query_min, Vec2 query_max) {
    if (!chunky.show_chunk_bounds) return;

    Array<uint32_t> chunk_indices = chunky_tri_mesh::get_chunks_overlapping_rect(chunky.chunky_tri_mesh, query_min, query_max);
    Map<uint32_t, uint8_t> overlapping_chunks = {};
    for (uint32_t chunk : chunk_indices) overlapping_chunks.set(chunk, 1);

    for (uint32_t i = 0; i < chunky.chunky_tri_mesh.nodes.length; i++) {
        const ChunkyTriMeshNode &node = chunky.chunky_tri_mesh.nodes[i];
        if (!chunky_tri_mesh::is_leaf(node)) continue;
        Color color = overlapping_chunks.has(i) ? to_color(chunky.triangle_in_query_color) : to_color(chunky.chunk_bounds_color);
        rect_edges(list, node.min.x, node.min.y, node.max.x, node.max.y, color);
    }

    overlapping_chunks.release();
    chunk_indices.release();
}

static void visualize_query_region(DrawList &list, Vec2 query_min, Vec2 query_max) {
    if (!chunky.show_query_region) return;
    rect_edges(list, query_min.x, query_min.y, query_max.x, query_max.y, to_color(chunky.query_region_color));
    Color fill = to_color(chunky.query_region_color, 0.15f);
    Vec3 a = {query_min.x, 0.05f, query_min.y};
    Vec3 b = {query_max.x, 0.05f, query_min.y};
    Vec3 c = {query_max.x, 0.05f, query_max.y};
    Vec3 d = {query_min.x, 0.05f, query_max.y};
    renderer::triangle(list, a, b, c, fill);
    renderer::triangle(list, a, c, d, fill);
}

static void update_visualization(const App &app) {
    renderer::release(chunky.drawing);

    float half_size = chunky.query_region_size / 2;
    Vec2 query_min = {chunky.query_region_x - half_size, chunky.query_region_z - half_size};
    Vec2 query_max = {chunky.query_region_x + half_size, chunky.query_region_z + half_size};

    DrawList list = {};
    visualize_triangles(app, list, query_min, query_max);
    visualize_chunk_bounds(list, query_min, query_max);
    visualize_query_region(list, query_min, query_max);
    chunky.drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    chunky = {};
    chunky.show_triangles = true;
    chunky.show_chunk_bounds = true;
    chunky.show_query_region = true;
    chunky.wireframe = true;
    chunky.query_region_size = 5;
    const float in_color[3] = {0, 1, 0};
    const float out_color[3] = {0.4f, 0.4f, 0.4f};
    const float bounds_color[3] = {0, 0.533f, 1};
    const float region_color[3] = {1, 1, 0};
    for (int i = 0; i < 3; i++) {
        chunky.triangle_in_query_color[i] = in_color[i];
        chunky.triangle_out_query_color[i] = out_color[i];
        chunky.chunk_bounds_color[i] = bounds_color[i];
        chunky.query_region_color[i] = region_color[i];
    }

    app::load_level(app, "dungeon.gltf");
    app.show_level = false;
    chunky.previous_fov_y = app.camera.fov_y;
    app.camera.fov_y = 10.0f * scalar::PI / 180.0f;
    camera::look_at(app.camera, {6, 750, -41.9f}, {6, 0, -42});

    double start = app::now_milliseconds();
    chunky.chunky_tri_mesh = chunky_tri_mesh::create({app.level.walkable_positions, app.level.walkable_indices}, 256);
    chunky.create_milliseconds = app::now_milliseconds() - start;
    for (const ChunkyTriMeshNode &node : chunky.chunky_tri_mesh.nodes) {
        if (chunky_tri_mesh::is_leaf(node)) chunky.leaf_count++;
    }

    update_visualization(app);
}

static void update(App &app, float) {
    if (app.click.clicked && !app.click.secondary && fabsf(app.click.ray.direction.y) > 1e-6f) {
        float t = -app.click.ray.origin.y / app.click.ray.direction.y;
        if (t > 0) {
            Vec3 point = vec3::scale_and_add(app.click.ray.origin, app.click.ray.direction, t);
            chunky.query_region_x = point.x;
            chunky.query_region_z = point.z;
            update_visualization(app);
        }
    }
    app::draw_static(app, chunky.drawing);
}

static void ui(App &app) {
    ImGui::TextWrapped("click the ground to move the query region");
    ImGui::Text("created in %.2f ms", chunky.create_milliseconds);
    ImGui::Text("%u triangles, %u nodes, %u leaves", app.level.walkable_indices.length / 3, chunky.chunky_tri_mesh.nodes.length, chunky.leaf_count);
    ImGui::Text("max tris per chunk: %u", chunky.chunky_tri_mesh.max_tris_per_chunk);
    ImGui::Text("triangles in query: %u", chunky.triangles_in_query);

    bool changed = false;
    if (ImGui::CollapsingHeader("Triangles", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::Checkbox("show triangles", &chunky.show_triangles);
        changed |= ImGui::Checkbox("wireframe", &chunky.wireframe);
        changed |= ImGui::ColorEdit3("in query color", chunky.triangle_in_query_color);
        changed |= ImGui::ColorEdit3("outside query color", chunky.triangle_out_query_color);
    }
    if (ImGui::CollapsingHeader("Chunk Bounds", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::Checkbox("show bounds", &chunky.show_chunk_bounds);
        changed |= ImGui::ColorEdit3("bounds color", chunky.chunk_bounds_color);
    }
    if (ImGui::CollapsingHeader("Query Region", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::Checkbox("show query", &chunky.show_query_region);
        changed |= ImGui::SliderFloat("query x", &chunky.query_region_x, -10, 10, "%.1f");
        changed |= ImGui::SliderFloat("query z", &chunky.query_region_z, -10, 10, "%.1f");
        changed |= ImGui::SliderFloat("query size", &chunky.query_region_size, 0.5f, 15, "%.1f");
        changed |= ImGui::ColorEdit3("region color", chunky.query_region_color);
    }
    if (changed) update_visualization(app);
}

static void shutdown(App &app) {
    renderer::release(chunky.drawing);
    chunky_tri_mesh::release(chunky.chunky_tri_mesh);
    app.show_level = true;
    app.camera.fov_y = chunky.previous_fov_y;
}

const Example EXAMPLE_CHUNKY_TRI_MESH = {"chunky tri mesh", init, update, ui, shutdown};
