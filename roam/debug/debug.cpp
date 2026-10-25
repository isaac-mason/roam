#include "roam/debug/debug.h"

#include <math.h>

#include "roam/generate/common.h"
#include "roam/math/scalar.h"

namespace roam {

namespace debug {

void release(DebugPrimitives &primitives) {
    for (DebugPrimitive &primitive : primitives) {
        primitive.positions.release();
        primitive.colors.release();
        primitive.indices.release();
        primitive.scales.release();
    }
    primitives.release();
}

static DebugPrimitive create_primitive(DebugPrimitiveType type) {
    DebugPrimitive primitive = {};
    primitive.type = type;
    primitive.opacity = 1;
    return primitive;
}

static void push3(Array<float> &values, float x, float y, float z) {
    values.push(x);
    values.push(y);
    values.push(z);
}

static void push3(Array<float> &values, Vec3 v) { push3(values, v.x, v.y, v.z); }

static void push_triangle_indices(Array<uint32_t> &indices, uint32_t base) {
    indices.push(base);
    indices.push(base + 1);
    indices.push(base + 2);
}

static void finish(DebugPrimitives &primitives, DebugPrimitive &primitive) {
    if (primitive.positions.length > 0) {
        primitives.push(primitive);
        return;
    }
    DebugPrimitives single = {};
    single.push(primitive);
    release(single);
}

static Vec3 hsl_to_rgb(float h, float s, float l) {
    h /= 360;
    float a = s * fminf(l, 1 - l);
    float channel[3];
    const float n[3] = {0, 8, 4};
    for (int i = 0; i < 3; i++) {
        float k = fmodf(n[i] + h * 12, 12);
        channel[i] = l - a * fmaxf(fminf(fminf(k - 3, 9 - k), 1), -1);
    }
    return {channel[0], channel[1], channel[2]};
}

static Vec3 hashed_color(int id, float alpha) {
    float hue = fmodf((float)id * 137.5f, 360);
    return vec3::scale(hsl_to_rgb(hue, 0.7f, 0.6f), alpha);
}

static Vec3 region_to_color(int region, float alpha = 1.0f) {
    if (region == 0) return {0, 0, 0};
    return hashed_color(region, alpha);
}

static Vec3 area_to_color(int area, float alpha = 1.0f) {
    if (area == WALKABLE_AREA) return {0, 192.0f / 255.0f, 1};
    if (area == NULL_AREA) return {0, 0, 0};
    return hashed_color(area, alpha);
}

static Vec3 walkable_area_color(int area) {
    if (area == WALKABLE_AREA) return {0, 1, 0};
    if (area == NULL_AREA) return {1, 0, 0};
    return area_to_color(area);
}

DebugPrimitives create_triangle_area_ids_helper(const InputTriangleMesh &mesh, Span<const uint8_t> tri_area_ids) {
    const float *positions = mesh.positions.data;
    const uint32_t *indices = mesh.indices.data;
    uint32_t index_count = mesh.indices.capacity;
    DebugPrimitives primitives = {};
    DebugPrimitive triangles = create_primitive(DEBUG_PRIMITIVE_TRIANGLES);
    triangles.transparent = true;

    for (uint32_t triangle = 0; triangle < index_count / 3; triangle++) {
        Vec3 color = walkable_area_color(tri_area_ids[triangle]);
        for (uint32_t k = 0; k < 3; k++) {
            push3(triangles.positions, vec3::load(positions, indices[triangle * 3 + k] * 3));
            push3(triangles.colors, color);
        }
        push_triangle_indices(triangles.indices, triangle * 3);
    }

    finish(primitives, triangles);
    return primitives;
}

DebugPrimitives create_heightfield_helper(const Heightfield &heightfield) {
    DebugPrimitives primitives = {};
    DebugPrimitive boxes = create_primitive(DEBUG_PRIMITIVE_BOXES);

    Vec3 origin = heightfield.bounds.min;
    float cell_size = heightfield.cell_size;
    float cell_height = heightfield.cell_height;

    for (int z = 0; z < heightfield.height; z++) {
        for (int x = 0; x < heightfield.width; x++) {
            for (int32_t span_index = heightfield.spans[(uint32_t)(x + z * heightfield.width)]; span_index != NULL_SPAN;
                 span_index = heightfield.span_pool[(uint32_t)span_index].next) {
                const HeightfieldSpan &span = heightfield.span_pool[(uint32_t)span_index];
                float world_x = origin.x + ((float)x + 0.5f) * cell_size;
                float world_z = origin.z + ((float)z + 0.5f) * cell_size;
                float span_height = (float)(span.max - span.min) * cell_height;
                float world_y = origin.y + ((float)span.min + (float)(span.max - span.min) * 0.5f) * cell_height;

                push3(boxes.positions, world_x, world_y, world_z);
                push3(boxes.scales, cell_size * 0.9f, span_height, cell_size * 0.9f);
                push3(boxes.colors, walkable_area_color(span.area));
            }
        }
    }

    finish(primitives, boxes);
    return primitives;
}

static void push_quad(DebugPrimitive &triangles, float x, float y, float z, float size, Vec3 color) {
    uint32_t base = triangles.positions.length / 3;
    push3(triangles.positions, x, y, z);
    push3(triangles.positions, x, y, z + size);
    push3(triangles.positions, x + size, y, z + size);
    push3(triangles.positions, x + size, y, z);
    for (int i = 0; i < 4; i++) push3(triangles.colors, color);
    triangles.indices.push(base);
    triangles.indices.push(base + 1);
    triangles.indices.push(base + 2);
    triangles.indices.push(base);
    triangles.indices.push(base + 2);
    triangles.indices.push(base + 3);
}

enum CompactHeightfieldColoring {
    COMPACT_HEIGHTFIELD_COLOR_AREAS,
    COMPACT_HEIGHTFIELD_COLOR_DISTANCES,
    COMPACT_HEIGHTFIELD_COLOR_REGIONS,
};

static DebugPrimitives create_compact_heightfield_quads(const CompactHeightfield &chf, CompactHeightfieldColoring coloring, float opacity) {
    DebugPrimitives primitives = {};
    DebugPrimitive triangles = create_primitive(DEBUG_PRIMITIVE_TRIANGLES);
    triangles.transparent = true;
    triangles.opacity = opacity;
    triangles.double_sided = true;

    float max_distance = (float)chf.max_distance < 1.0f ? 1.0f : (float)chf.max_distance;
    float distance_scale = 255.0f / max_distance;

    for (int y = 0; y < chf.height; y++) {
        for (int x = 0; x < chf.width; x++) {
            float fx = chf.bounds.min.x + (float)x * chf.cell_size;
            float fz = chf.bounds.min.z + (float)y * chf.cell_size;
            const CompactHeightfieldCell &cell = chf.cells[(uint32_t)(x + y * chf.width)];

            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                const CompactHeightfieldSpan &span = chf.spans[i];
                Vec3 color = {0, 0, 0};
                float fy = chf.bounds.min.y + (float)(span.y + 1) * chf.cell_height;

                if (coloring == COMPACT_HEIGHTFIELD_COLOR_AREAS) {
                    color = area_to_color(chf.areas[i]);
                } else if (coloring == COMPACT_HEIGHTFIELD_COLOR_DISTANCES) {
                    float shade = fminf(255, floorf((float)chf.distances[i] * distance_scale)) / 255.0f;
                    color = {shade, shade, shade};
                } else {
                    color = region_to_color(span.region);
                    fy = chf.bounds.min.y + (float)span.y * chf.cell_height;
                }

                push_quad(triangles, fx, fy, fz, chf.cell_size, color);
            }
        }
    }

    finish(primitives, triangles);
    return primitives;
}

DebugPrimitives create_compact_heightfield_solid_helper(const CompactHeightfield &compact_heightfield) {
    return create_compact_heightfield_quads(compact_heightfield, COMPACT_HEIGHTFIELD_COLOR_AREAS, 0.6f);
}

DebugPrimitives create_compact_heightfield_distances_helper(const CompactHeightfield &compact_heightfield) {
    if (compact_heightfield.distances.length == 0) return {};
    return create_compact_heightfield_quads(compact_heightfield, COMPACT_HEIGHTFIELD_COLOR_DISTANCES, 0.8f);
}

DebugPrimitives create_compact_heightfield_regions_helper(const CompactHeightfield &compact_heightfield) {
    return create_compact_heightfield_quads(compact_heightfield, COMPACT_HEIGHTFIELD_COLOR_REGIONS, 0.9f);
}

static Vec3 contour_vertex(const ContourSet &contour_set, const int *v, int contour_index, float offset) {
    return {
        contour_set.bounds.min.x + (float)v[0] * contour_set.cell_size,
        contour_set.bounds.min.y + (float)(v[1] + 1 + (contour_index & 1)) * contour_set.cell_height + offset,
        contour_set.bounds.min.z + (float)v[2] * contour_set.cell_size,
    };
}

static void push_contour_points(const ContourSet &contour_set, bool raw, DebugPrimitive &points) {
    for (uint32_t i = 0; i < contour_set.contours.length; ++i) {
        const Contour &contour = contour_set.contours[i];
        Vec3 darkened = vec3::scale(region_to_color(contour.reg, 0.8f), 0.5f);
        int count = raw ? contour.raw_vertex_count : contour.vertex_count;
        const int *vertices = raw ? contour.raw_vertices.data : contour.vertices.data;

        for (int j = 0; j < count; ++j) {
            const int *v = &vertices[j * 4];
            bool border = (v[3] & BORDER_VERTEX) != 0;
            push3(points.positions, contour_vertex(contour_set, v, (int)i, border ? contour_set.cell_height * 2 : 0));
            push3(points.colors, border ? Vec3{1, 1, 1} : darkened);
        }
    }
}

DebugPrimitives create_raw_contours_helper(const ContourSet &contour_set) {
    DebugPrimitives primitives = {};
    DebugPrimitive lines = create_primitive(DEBUG_PRIMITIVE_LINES);
    lines.transparent = true;
    lines.opacity = 0.8f;
    lines.line_width = 2.0f;

    for (uint32_t i = 0; i < contour_set.contours.length; ++i) {
        const Contour &contour = contour_set.contours[i];
        Vec3 color = region_to_color(contour.reg, 0.8f);

        for (int j = 0; j < contour.raw_vertex_count; ++j) {
            Vec3 position = contour_vertex(contour_set, &contour.raw_vertices[(uint32_t)(j * 4)], (int)i, 0);
            push3(lines.positions, position);
            push3(lines.colors, color);
            if (j > 0) {
                push3(lines.positions, position);
                push3(lines.colors, color);
            }
        }

        if (contour.raw_vertex_count > 0) {
            push3(lines.positions, contour_vertex(contour_set, &contour.raw_vertices[0], (int)i, 0));
            push3(lines.colors, color);
        }
    }

    DebugPrimitive points = create_primitive(DEBUG_PRIMITIVE_POINTS);
    points.size = 0.01f;
    points.transparent = true;
    push_contour_points(contour_set, true, points);

    finish(primitives, lines);
    finish(primitives, points);
    return primitives;
}

DebugPrimitives create_simplified_contours_helper(const ContourSet &contour_set) {
    DebugPrimitives primitives = {};
    DebugPrimitive lines = create_primitive(DEBUG_PRIMITIVE_LINES);
    lines.transparent = true;
    lines.opacity = 0.9f;
    lines.line_width = 2.5f;

    for (uint32_t i = 0; i < contour_set.contours.length; ++i) {
        const Contour &contour = contour_set.contours[i];
        if (contour.vertex_count == 0) continue;

        Vec3 base_color = region_to_color(contour.reg, 0.8f);
        Vec3 border_color = vec3::lerp(base_color, {1, 1, 1}, 128.0f / 255.0f);

        for (int j = 0, k = contour.vertex_count - 1; j < contour.vertex_count; k = j++) {
            const int *va = &contour.vertices[(uint32_t)(k * 4)];
            const int *vb = &contour.vertices[(uint32_t)(j * 4)];
            Vec3 color = (va[3] & AREA_BORDER) != 0 ? border_color : base_color;

            push3(lines.positions, contour_vertex(contour_set, va, (int)i, 0));
            push3(lines.colors, color);
            push3(lines.positions, contour_vertex(contour_set, vb, (int)i, 0));
            push3(lines.colors, color);
        }
    }

    DebugPrimitive points = create_primitive(DEBUG_PRIMITIVE_POINTS);
    points.size = 0.01f;
    points.transparent = true;
    push_contour_points(contour_set, false, points);

    finish(primitives, lines);
    finish(primitives, points);
    return primitives;
}

static Vec3 poly_mesh_vertex(const PolyMesh &poly_mesh, int vertex, float offset) {
    const int *v = &poly_mesh.vertices[(uint32_t)(vertex * 3)];
    return {
        poly_mesh.bounds.min.x + (float)v[0] * poly_mesh.cell_size,
        poly_mesh.bounds.min.y + (float)(v[1] + 1) * poly_mesh.cell_height + offset,
        poly_mesh.bounds.min.z + (float)v[2] * poly_mesh.cell_size,
    };
}

DebugPrimitives create_poly_mesh_helper(const PolyMesh &poly_mesh) {
    DebugPrimitives primitives = {};
    if (poly_mesh.poly_count == 0) return primitives;

    int nvp = poly_mesh.max_vertices_per_poly;

    DebugPrimitive triangles = create_primitive(DEBUG_PRIMITIVE_TRIANGLES);
    triangles.double_sided = true;

    for (int i = 0; i < poly_mesh.poly_count; i++) {
        const int *poly = &poly_mesh.polys[(uint32_t)(i * nvp)];
        Vec3 color = area_to_color(poly_mesh.areas[(uint32_t)i]);

        for (int j = 2; j < nvp; j++) {
            if (poly[j] == MESH_NULL_IDX) break;
            const int corners[3] = {poly[0], poly[j - 1], poly[j]};
            uint32_t base = triangles.positions.length / 3;
            for (int k = 0; k < 3; k++) {
                push3(triangles.positions, poly_mesh_vertex(poly_mesh, corners[k], 0));
                push3(triangles.colors, color);
            }
            push_triangle_indices(triangles.indices, base);
        }
    }

    DebugPrimitive edges = create_primitive(DEBUG_PRIMITIVE_LINES);
    edges.transparent = true;
    edges.opacity = 0.5f;
    edges.line_width = 1.5f;
    const Vec3 edge_color = {0, 48.0f / 255.0f, 64.0f / 255.0f};

    for (int i = 0; i < poly_mesh.poly_count; i++) {
        const int *poly = &poly_mesh.polys[(uint32_t)(i * nvp)];
        for (int j = 0; j < nvp; j++) {
            if (poly[j] == MESH_NULL_IDX) break;
            int next = j + 1 >= nvp || poly[j + 1] == MESH_NULL_IDX ? 0 : j + 1;
            push3(edges.positions, poly_mesh_vertex(poly_mesh, poly[j], 0.01f));
            push3(edges.colors, edge_color);
            push3(edges.positions, poly_mesh_vertex(poly_mesh, poly[next], 0.01f));
            push3(edges.colors, edge_color);
        }
    }

    DebugPrimitive points = create_primitive(DEBUG_PRIMITIVE_POINTS);
    points.size = 0.025f;
    points.transparent = true;
    for (int i = 0; i < poly_mesh.vertex_count; i++) {
        push3(points.positions, poly_mesh_vertex(poly_mesh, i, 0.01f));
        push3(points.colors, 1, 1, 1);
    }

    finish(primitives, triangles);
    finish(primitives, edges);
    finish(primitives, points);
    return primitives;
}

DebugPrimitives create_poly_mesh_detail_helper(const PolyMeshDetail &detail) {
    DebugPrimitives primitives = {};
    if (detail.mesh_count == 0) return primitives;

    DebugPrimitive triangles = create_primitive(DEBUG_PRIMITIVE_TRIANGLES);
    DebugPrimitive internal_edges = create_primitive(DEBUG_PRIMITIVE_LINES);
    internal_edges.transparent = true;
    internal_edges.opacity = 0.4f;
    internal_edges.line_width = 1.0f;
    DebugPrimitive external_edges = create_primitive(DEBUG_PRIMITIVE_LINES);
    external_edges.transparent = true;
    external_edges.opacity = 0.8f;
    external_edges.line_width = 10.0f;
    DebugPrimitive points = create_primitive(DEBUG_PRIMITIVE_POINTS);
    points.size = 0.025f;

    for (int i = 0; i < detail.mesh_count; ++i) {
        const int *mesh = &detail.meshes[(uint32_t)(i * 4)];
        const float *vertices = &detail.vertices.data[mesh[0] * 3];
        const int *mesh_triangles = &detail.triangles.data[mesh[2] * 4];
        Vec3 color = hashed_color(i, 0.3f);

        for (int j = 0; j < mesh[3]; ++j) {
            const int *t = &mesh_triangles[j * 4];
            uint32_t base = triangles.positions.length / 3;
            for (int k = 0; k < 3; k++) {
                push3(triangles.positions, vec3::load(vertices, (uint32_t)(t[k] * 3)));
                push3(triangles.colors, color);
            }
            push_triangle_indices(triangles.indices, base);

            for (int k = 0, kp = 2; k < 3; kp = k++) {
                int edge_flags = (t[3] >> (kp * 2)) & 0x3;
                if (edge_flags == 0 && t[kp] >= t[k]) continue;
                DebugPrimitive &edges = edge_flags == 0 ? internal_edges : external_edges;
                push3(edges.positions, vec3::load(vertices, (uint32_t)(t[kp] * 3)));
                push3(edges.positions, vec3::load(vertices, (uint32_t)(t[k] * 3)));
                push3(edges.colors, 0, 0, 0);
                push3(edges.colors, 0, 0, 0);
            }
        }

        for (int j = 0; j < mesh[1]; ++j) {
            push3(points.positions, vec3::load(vertices, (uint32_t)(j * 3)));
            push3(points.colors, 1, 1, 1);
        }
    }

    finish(primitives, triangles);
    finish(primitives, internal_edges);
    finish(primitives, external_edges);
    finish(primitives, points);
    return primitives;
}

static Vec3 tile_vertex(const NavMeshTile &tile, uint32_t vertex) { return vec3::load(tile.vertices.data, vertex * 3); }

static Vec3 detail_vertex(const NavMeshTile &tile, const NavMeshPoly &poly, const NavMeshPolyDetail &detail, uint32_t vertex) {
    if (vertex < poly.vertex_count) return tile_vertex(tile, tile.poly_vertices[poly.vertex_base + vertex]);
    return vec3::load(tile.detail_vertices.data, (detail.vertices_base + vertex - poly.vertex_count) * 3);
}

struct NavMeshTileDrawing {
    DebugPrimitive triangles;
    DebugPrimitive inner_edges;
    DebugPrimitive outer_edges;
    DebugPrimitive points;
};

static NavMeshTileDrawing create_nav_mesh_tile_drawing() {
    NavMeshTileDrawing drawing = {};
    drawing.triangles = create_primitive(DEBUG_PRIMITIVE_TRIANGLES);
    drawing.triangles.transparent = true;
    drawing.triangles.opacity = 0.8f;
    drawing.triangles.double_sided = true;
    drawing.inner_edges = create_primitive(DEBUG_PRIMITIVE_LINES);
    drawing.inner_edges.transparent = true;
    drawing.inner_edges.opacity = 0.3f;
    drawing.inner_edges.line_width = 1.5f;
    drawing.outer_edges = create_primitive(DEBUG_PRIMITIVE_LINES);
    drawing.outer_edges.transparent = true;
    drawing.outer_edges.opacity = 0.9f;
    drawing.outer_edges.line_width = 2.5f;
    drawing.points = create_primitive(DEBUG_PRIMITIVE_POINTS);
    drawing.points.size = 0.025f;
    return drawing;
}

static void draw_nav_mesh_tile(NavMeshTileDrawing &drawing, const NavMeshTile &tile, const Array<uint32_t> *cluster_islands = nullptr) {
    for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
        if (poly_index >= tile.detail_meshes.length) continue;
        const NavMeshPoly &poly = tile.polys[poly_index];
        const NavMeshPolyDetail &detail = tile.detail_meshes[poly_index];
        Vec3 color = cluster_islands && poly_index < tile.poly_clusters.length
                         ? hashed_color((int)(*cluster_islands)[tile.poly_clusters[poly_index]] + 1, 0.4f)
                         : area_to_color(poly.area, 0.4f);

        for (uint32_t j = 0; j < detail.triangles_count; j++) {
            const uint8_t *t = &tile.detail_triangles[(detail.triangles_base + j) * 4];
            uint32_t base = drawing.triangles.positions.length / 3;
            for (int k = 0; k < 3; k++) {
                push3(drawing.triangles.positions, detail_vertex(tile, poly, detail, (uint32_t)t[k]));
                push3(drawing.triangles.colors, color);
            }
            push_triangle_indices(drawing.triangles.indices, base);
        }
    }

    const Vec3 inner_color = {0.2f, 0.2f, 0.2f};
    const Vec3 outer_color = {0.6f, 0.6f, 1};
    const Vec3 lift = {0, 0.01f, 0};

    for (const NavMeshPoly &poly : tile.polys) {
        for (uint32_t j = 0; j < poly.vertex_count; j++) {
            uint32_t next = (j + 1) % poly.vertex_count;
            bool boundary = (tile.poly_neis[poly.vertex_base + j] & POLY_NEIS_FLAG_EXT_LINK) != 0;
            DebugPrimitive &edges = boundary ? drawing.outer_edges : drawing.inner_edges;
            Vec3 color = boundary ? outer_color : inner_color;
            push3(edges.positions, vec3::add(tile_vertex(tile, tile.poly_vertices[poly.vertex_base + j]), lift));
            push3(edges.colors, color);
            push3(edges.positions, vec3::add(tile_vertex(tile, tile.poly_vertices[poly.vertex_base + next]), lift));
            push3(edges.colors, color);
        }
    }

    for (uint32_t i = 0; i + 2 < tile.vertices.length; i += 3) {
        push3(drawing.points.positions, vec3::load(tile.vertices.data, i));
        push3(drawing.points.colors, 1, 1, 1);
    }
}

static DebugPrimitives finish_nav_mesh_tile_drawing(NavMeshTileDrawing &drawing) {
    DebugPrimitives primitives = {};
    finish(primitives, drawing.triangles);
    finish(primitives, drawing.inner_edges);
    finish(primitives, drawing.outer_edges);
    finish(primitives, drawing.points);
    return primitives;
}

DebugPrimitives create_nav_mesh_helper(const NavMesh &nav_mesh) {
    NavMeshTileDrawing drawing = create_nav_mesh_tile_drawing();
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (tile.allocated) draw_nav_mesh_tile(drawing, tile);
    }
    return finish_nav_mesh_tile_drawing(drawing);
}

static uint32_t find_island(Array<uint32_t> &islands, uint32_t cluster) {
    while (islands[cluster] != cluster) {
        islands[cluster] = islands[islands[cluster]];
        cluster = islands[cluster];
    }
    return cluster;
}

DebugPrimitives create_nav_mesh_islands_helper(const NavMesh &nav_mesh) {
    Array<uint32_t> islands = {};
    islands.resize(nav_mesh.clusters.length);
    for (uint32_t i = 0; i < islands.length; i++) islands[i] = i;
    for (uint32_t cluster = 0; cluster < nav_mesh.clusters.length; cluster++) {
        if (!nav_mesh.clusters[cluster].allocated) continue;
        for (int32_t i = nav_mesh.clusters[cluster].first_link; i != NULL_LINK; i = nav_mesh.cluster_links[(uint32_t)i].next) {
            uint32_t a = find_island(islands, cluster);
            uint32_t b = find_island(islands, nav_mesh.cluster_links[(uint32_t)i].to_cluster);
            if (a != b) islands[a < b ? b : a] = a < b ? a : b;
        }
    }
    for (uint32_t cluster = 0; cluster < islands.length; cluster++) islands[cluster] = find_island(islands, cluster);

    NavMeshTileDrawing drawing = create_nav_mesh_tile_drawing();
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (tile.allocated) draw_nav_mesh_tile(drawing, tile, &islands);
    }
    islands.release();
    return finish_nav_mesh_tile_drawing(drawing);
}

DebugPrimitives create_nav_mesh_tile_helper(const NavMeshTile &tile) {
    NavMeshTileDrawing drawing = create_nav_mesh_tile_drawing();
    draw_nav_mesh_tile(drawing, tile);
    return finish_nav_mesh_tile_drawing(drawing);
}

DebugPrimitives create_nav_mesh_poly_helper(const NavMesh &nav_mesh, NodeRef node_ref, Vec3 color) {
    DebugPrimitives primitives = {};

    uint32_t node_index = node::get_node_ref_index(node_ref);
    if (node_index >= nav_mesh.nodes.length) return primitives;
    const NavMeshNode &node = nav_mesh.nodes[node_index];
    if (!node.allocated || node.type != NODE_TYPE_POLY || node.tile_id < 0 || (uint32_t)node.tile_id >= nav_mesh.tiles.length) return primitives;

    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
    if (!tile.allocated || node.poly_index < 0 || (uint32_t)node.poly_index >= tile.polys.length) return primitives;
    const NavMeshPoly &poly = tile.polys[(uint32_t)node.poly_index];

    DebugPrimitive triangles = create_primitive(DEBUG_PRIMITIVE_TRIANGLES);
    triangles.transparent = true;
    triangles.opacity = 0.6f;
    triangles.double_sided = true;
    Vec3 base_color = vec3::scale(color, 0.25f);

    if ((uint32_t)node.poly_index >= tile.detail_meshes.length) {
        for (uint32_t i = 2; i < poly.vertex_count; i++) {
            const uint32_t corners[3] = {0, i - 1, i};
            uint32_t base = triangles.positions.length / 3;
            for (int k = 0; k < 3; k++) {
                push3(triangles.positions, tile_vertex(tile, tile.poly_vertices[poly.vertex_base + corners[k]]));
                push3(triangles.colors, base_color);
            }
            push_triangle_indices(triangles.indices, base);
        }
    } else {
        const NavMeshPolyDetail &detail = tile.detail_meshes[(uint32_t)node.poly_index];
        for (uint32_t i = 0; i < detail.triangles_count; ++i) {
            const uint8_t *t = &tile.detail_triangles[(detail.triangles_base + i) * 4];
            for (int j = 0; j < 3; ++j) {
                push3(triangles.positions, detail_vertex(tile, poly, detail, (uint32_t)t[j]));
                push3(triangles.colors, base_color);
            }
            push_triangle_indices(triangles.indices, i * 3);
        }
    }

    finish(primitives, triangles);
    return primitives;
}

static void push_box_edges(DebugPrimitive &lines, Vec3 min, Vec3 max, Vec3 color) {
    const Vec3 corners[8] = {
        {min.x, min.y, min.z}, {max.x, min.y, min.z}, {max.x, min.y, max.z}, {min.x, min.y, max.z},
        {min.x, max.y, min.z}, {max.x, max.y, min.z}, {max.x, max.y, max.z}, {min.x, max.y, max.z},
    };
    const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const int *edge : edges) {
        push3(lines.positions, corners[edge[0]]);
        push3(lines.positions, corners[edge[1]]);
        push3(lines.colors, color);
        push3(lines.colors, color);
    }
}

static void draw_bv_tree(DebugPrimitive &lines, const NavMeshTile &tile) {
    float cs = 1.0f / tile.bv_tree.quant_factor;
    for (const NavMeshBvNode &node : tile.bv_tree.nodes) {
        if (node.i < 0) continue;
        Vec3 min = {tile.bounds.min.x + node.bmin[0] * cs, tile.bounds.min.y + node.bmin[1] * cs, tile.bounds.min.z + node.bmin[2] * cs};
        Vec3 max = {tile.bounds.min.x + node.bmax[0] * cs, tile.bounds.min.y + node.bmax[1] * cs, tile.bounds.min.z + node.bmax[2] * cs};
        push_box_edges(lines, min, max, {1, 1, 1});
    }
}

static DebugPrimitive create_bv_tree_lines() {
    DebugPrimitive lines = create_primitive(DEBUG_PRIMITIVE_LINES);
    lines.transparent = true;
    lines.opacity = 0.5f;
    lines.line_width = 1.0f;
    return lines;
}

DebugPrimitives create_nav_mesh_tile_bv_tree_helper(const NavMeshTile &tile) {
    DebugPrimitives primitives = {};
    DebugPrimitive lines = create_bv_tree_lines();
    draw_bv_tree(lines, tile);
    finish(primitives, lines);
    return primitives;
}

DebugPrimitives create_nav_mesh_bv_tree_helper(const NavMesh &nav_mesh) {
    DebugPrimitives primitives = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        DebugPrimitive lines = create_bv_tree_lines();
        draw_bv_tree(lines, tile);
        finish(primitives, lines);
    }
    return primitives;
}

static Vec3 poly_center(const NavMeshTile &tile, const NavMeshPoly &poly) {
    Vec3 center = {0, 0, 0};
    for (uint32_t i = 0; i < poly.vertex_count; i++) center = vec3::add(center, tile_vertex(tile, tile.poly_vertices[poly.vertex_base + i]));
    return vec3::scale(center, 1.0f / (float)poly.vertex_count);
}

static const NavMeshPoly *node_poly(const NavMesh &nav_mesh, uint32_t node_index, const NavMeshTile **out_tile) {
    const NavMeshNode &node = nav_mesh.nodes[node_index];
    if (node.type != NODE_TYPE_POLY || node.tile_id < 0 || (uint32_t)node.tile_id >= nav_mesh.tiles.length) return nullptr;
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
    if (!tile.allocated || node.poly_index < 0 || (uint32_t)node.poly_index >= tile.polys.length) return nullptr;
    *out_tile = &tile;
    return &tile.polys[(uint32_t)node.poly_index];
}

static Vec3 arc_point(Vec3 from, Vec3 to, float t, float height) {
    Vec3 point = vec3::lerp(from, to, t);
    point.y += sinf(t * scalar::PI) * height;
    return point;
}

DebugPrimitives create_nav_mesh_links_helper(const NavMesh &nav_mesh) {
    DebugPrimitives primitives = {};
    DebugPrimitive lines = create_primitive(DEBUG_PRIMITIVE_LINES);
    lines.transparent = true;
    lines.opacity = 0.9f;
    lines.line_width = 3.0f;
    const Vec3 link_color = {1, 1, 0};

    for (uint32_t node_index = 0; node_index < nav_mesh.nodes.length; node_index++) {
        if (!nav_mesh.nodes[node_index].allocated) continue;
        for (int32_t link_index = nav_mesh.nodes[node_index].first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];

            const NavMeshTile *source_tile = nullptr;
            const NavMeshTile *target_tile = nullptr;
            const NavMeshPoly *source_poly = node_poly(nav_mesh, node_index, &source_tile);
            const NavMeshPoly *target_poly = node_poly(nav_mesh, node::get_node_ref_index(link.to_node_ref), &target_tile);
            if (!source_poly || !target_poly) continue;

            Vec3 source_center = poly_center(*source_tile, *source_poly);
            Vec3 target_center = poly_center(*target_tile, *target_poly);

            uint32_t edge = link.edge;
            uint32_t next_edge = (edge + 1) % source_poly->vertex_count;
            Vec3 edge_start = tile_vertex(*source_tile, source_tile->poly_vertices[source_poly->vertex_base + edge]);
            Vec3 edge_end = tile_vertex(*source_tile, source_tile->poly_vertices[source_poly->vertex_base + next_edge]);
            Vec3 edge_midpoint = vec3::scale(vec3::add(edge_start, edge_end), 0.5f);

            Vec3 source_point = vec3::lerp(edge_midpoint, source_center, 0.1f);
            source_point.y += 0.05f;
            Vec3 target_point = target_center;
            target_point.y += 0.05f;

            const int segment_count = 12;
            const float arc_height = 0.3f;
            for (int i = 0; i < segment_count; i++) {
                push3(lines.positions, arc_point(source_point, target_point, (float)i / segment_count, arc_height));
                push3(lines.positions, arc_point(source_point, target_point, (float)(i + 1) / segment_count, arc_height));
                push3(lines.colors, link_color);
                push3(lines.colors, link_color);
            }

            Vec3 arrow = arc_point(source_point, target_point, 0.85f, arc_height);
            Vec3 next = arc_point(source_point, target_point, 0.95f, arc_height);
            Vec3 direction = vec3::subtract(next, arrow);
            float length = vec3::length(direction);
            if (length == 0) length = 1;
            direction = vec3::scale(direction, 1.0f / length);

            const float arrow_length = 0.15f;
            const float arrow_width = 0.08f;
            Vec3 perpendicular = {-direction.z, 0, direction.x};
            Vec3 back = vec3::scale(direction, -arrow_length);

            push3(lines.positions, arrow);
            push3(lines.positions, vec3::scale_and_add(vec3::add(arrow, back), perpendicular, arrow_width));
            push3(lines.positions, arrow);
            push3(lines.positions, vec3::scale_and_add(vec3::add(arrow, back), perpendicular, -arrow_width));
            for (int k = 0; k < 4; k++) push3(lines.colors, link_color);
        }
    }

    finish(primitives, lines);
    return primitives;
}

static void draw_tile_portals(DebugPrimitive &lines, const NavMeshTile &tile) {
    const float pad = 0.04f;
    const float pad_y = tile.walkable_climb;
    const int sides[4] = {0, 2, 4, 6};
    const Vec3 side_colors[4] = {{128.0f / 255.0f, 0, 0}, {0, 128.0f / 255.0f, 0}, {128.0f / 255.0f, 0, 128.0f / 255.0f}, {0, 128.0f / 255.0f, 128.0f / 255.0f}};

    for (int s = 0; s < 4; s++) {
        int side = sides[s];
        int match = POLY_NEIS_FLAG_EXT_LINK | side;

        for (const NavMeshPoly &poly : tile.polys) {
            for (uint32_t j = 0; j < poly.vertex_count; j++) {
                if (tile.poly_neis[poly.vertex_base + j] != match) continue;

                Vec3 a = tile_vertex(tile, tile.poly_vertices[poly.vertex_base + j]);
                Vec3 b = tile_vertex(tile, tile.poly_vertices[poly.vertex_base + (j + 1) % poly.vertex_count]);

                Vec3 corners[4];
                if (side == 0 || side == 4) {
                    float x = a.x + (side == 0 ? -pad : pad);
                    corners[0] = {x, a.y - pad_y, a.z};
                    corners[1] = {x, a.y + pad_y, a.z};
                    corners[2] = {x, b.y + pad_y, b.z};
                    corners[3] = {x, b.y - pad_y, b.z};
                } else {
                    float z = a.z + (side == 2 ? -pad : pad);
                    corners[0] = {a.x, a.y - pad_y, z};
                    corners[1] = {a.x, a.y + pad_y, z};
                    corners[2] = {b.x, b.y + pad_y, z};
                    corners[3] = {b.x, b.y - pad_y, z};
                }

                for (int k = 0; k < 4; k++) {
                    push3(lines.positions, corners[k]);
                    push3(lines.positions, corners[(k + 1) % 4]);
                    push3(lines.colors, side_colors[s]);
                    push3(lines.colors, side_colors[s]);
                }
            }
        }
    }
}

static DebugPrimitive create_portal_lines() {
    DebugPrimitive lines = create_primitive(DEBUG_PRIMITIVE_LINES);
    lines.transparent = true;
    lines.opacity = 0.5f;
    lines.line_width = 2.0f;
    return lines;
}

DebugPrimitives create_nav_mesh_tile_portals_helper(const NavMeshTile &tile) {
    DebugPrimitives primitives = {};
    DebugPrimitive lines = create_portal_lines();
    draw_tile_portals(lines, tile);
    finish(primitives, lines);
    return primitives;
}

DebugPrimitives create_nav_mesh_portals_helper(const NavMesh &nav_mesh) {
    DebugPrimitives primitives = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        DebugPrimitive lines = create_portal_lines();
        draw_tile_portals(lines, tile);
        finish(primitives, lines);
    }
    return primitives;
}

DebugPrimitives create_search_nodes_helper(const SearchNodePool &pool) {
    DebugPrimitives primitives = {};
    const float y_offset = 0.5f;
    const Vec3 color = {1.0f, 192.0f / 255.0f, 0.0f};

    DebugPrimitive points = create_primitive(DEBUG_PRIMITIVE_POINTS);
    points.size = 0.01f;
    points.transparent = true;
    DebugPrimitive lines = create_primitive(DEBUG_PRIMITIVE_LINES);
    lines.transparent = true;
    lines.opacity = 0.5f;
    lines.line_width = 2.0f;

    for (const SearchNode &node : pool.nodes) {
        push3(points.positions, node.position.x, node.position.y + y_offset, node.position.z);
        push3(points.colors, color);
    }

    for (const SearchNode &node : pool.nodes) {
        if (node.parent_node_ref == INVALID_NODE_REF || node.parent_state == NO_PARENT_STATE) continue;
        int32_t parent = nav_mesh_search::get_search_node(pool, node.parent_node_ref, node.parent_state);
        if (parent < 0) continue;
        Vec3 parent_position = pool.nodes[(uint32_t)parent].position;
        push3(lines.positions, node.position.x, node.position.y + y_offset, node.position.z);
        push3(lines.positions, parent_position.x, parent_position.y + y_offset, parent_position.z);
        push3(lines.colors, color);
        push3(lines.colors, color);
    }

    finish(primitives, points);
    finish(primitives, lines);
    return primitives;
}

DebugPrimitives create_search_nodes_helper(const QueryWorkspace &workspace) { return create_search_nodes_helper(workspace.nodes); }

static void push_circle(DebugPrimitive &lines, Vec3 center, float radius, Vec3 color) {
    const int segment_count = 20;
    const Vec3 pillar_color = {0, 48.0f / 255.0f, 64.0f / 255.0f};
    for (int i = 0; i < segment_count; i++) {
        float a0 = ((float)i / segment_count) * scalar::PI * 2;
        float a1 = ((float)(i + 1) / segment_count) * scalar::PI * 2;
        push3(lines.positions, center.x + cosf(a0) * radius, center.y + 0.1f, center.z + sinf(a0) * radius);
        push3(lines.positions, center.x + cosf(a1) * radius, center.y + 0.1f, center.z + sinf(a1) * radius);
        push3(lines.colors, color);
        push3(lines.colors, color);
    }
    push3(lines.positions, center);
    push3(lines.positions, center.x, center.y + 0.2f, center.z);
    push3(lines.colors, pillar_color);
    push3(lines.colors, pillar_color);
}

DebugPrimitives create_nav_mesh_off_mesh_connections_helper(const NavMesh &nav_mesh) {
    DebugPrimitives primitives = {};
    const int arc_segments = 16;
    const Vec3 arc_color = {1, 196.0f / 255.0f, 0};
    const Vec3 one_way_end_color = {220.0f / 255.0f, 32.0f / 255.0f, 16.0f / 255.0f};

    DebugPrimitive arcs = create_primitive(DEBUG_PRIMITIVE_LINES);
    arcs.transparent = true;
    arcs.opacity = 0.9f;
    arcs.line_width = 2.0f;
    DebugPrimitive circles = create_primitive(DEBUG_PRIMITIVE_LINES);
    circles.transparent = true;
    circles.opacity = 0.8f;
    circles.line_width = 1.5f;

    for (const OffMeshConnection &connection : nav_mesh.off_mesh_connections) {
        if (!connection.allocated) continue;
        Vec3 start = connection.start;
        Vec3 end = connection.end;

        for (int i = 0; i < arc_segments; i++) {
            push3(arcs.positions, arc_point(start, end, (float)i / arc_segments, 0.25f));
            push3(arcs.positions, arc_point(start, end, (float)(i + 1) / arc_segments, 0.25f));
            push3(arcs.colors, arc_color);
            push3(arcs.colors, arc_color);
        }

        if (connection.direction == OFF_MESH_CONNECTION_DIRECTION_START_TO_END) {
            Vec3 middle = vec3::lerp(start, end, 0.5f);
            middle.y += 0.25f;
            float dx = end.x - start.x;
            float dz = end.z - start.z;
            float length = sqrtf(dx * dx + dz * dz);
            if (length == 0) length = 1;
            float nx = dx / length;
            float nz = dz / length;
            const float back = 0.3f;
            push3(arcs.positions, middle);
            push3(arcs.positions, middle.x - nx * back + nz * back * 0.5f, middle.y - 0.05f, middle.z - nz * back - nx * back * 0.5f);
            push3(arcs.positions, middle);
            push3(arcs.positions, middle.x - nx * back - nz * back * 0.5f, middle.y - 0.05f, middle.z - nz * back + nx * back * 0.5f);
            for (int k = 0; k < 4; k++) push3(arcs.colors, arc_color);
        }

        push_circle(circles, start, connection.radius, arc_color);
        push_circle(circles, end, connection.radius,
                    connection.direction == OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL ? arc_color : one_way_end_color);
    }

    finish(primitives, arcs);
    finish(primitives, circles);
    return primitives;
}

}

}
