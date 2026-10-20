#include "roam/blocks/geometry/chunky_tri_mesh.h"

#include <math.h>

#include "roam/math/scalar.h"
#include "roam/util/sort.h"

namespace roam {

namespace chunky_tri_mesh {

struct BoundsItem {
    Vec2 min;
    Vec2 max;
    uint32_t index;
};

static BoundsItem calculate_triangle_bounds(const float *vertices, const uint32_t *indices, uint32_t triangle) {
    const float *v0 = &vertices[indices[triangle * 3 + 0] * 3];
    const float *v1 = &vertices[indices[triangle * 3 + 1] * 3];
    const float *v2 = &vertices[indices[triangle * 3 + 2] * 3];
    return {
        {fminf(fminf(v0[0], v1[0]), v2[0]), fminf(fminf(v0[2], v1[2]), v2[2])},
        {fmaxf(fmaxf(v0[0], v1[0]), v2[0]), fmaxf(fmaxf(v0[2], v1[2]), v2[2])},
        triangle,
    };
}

static void calculate_extents(const Array<BoundsItem> &items, uint32_t min, uint32_t max, ChunkyTriMeshNode &node) {
    node.min = items[min].min;
    node.max = items[min].max;
    for (uint32_t i = min + 1; i < max; i++) {
        const BoundsItem &item = items[i];
        node.min.x = fminf(node.min.x, item.min.x);
        node.min.y = fminf(node.min.y, item.min.y);
        node.max.x = fmaxf(node.max.x, item.max.x);
        node.max.y = fmaxf(node.max.y, item.max.y);
    }
}

static void subdivide(Array<BoundsItem> &items, uint32_t min, uint32_t max, uint32_t tris_per_chunk, ChunkyTriMesh &mesh,
                      const uint32_t *indices) {
    uint32_t node_index = mesh.nodes.length;
    uint32_t count = max - min;

    ChunkyTriMeshNode node = {};
    calculate_extents(items, min, max, node);
    mesh.nodes.push(node);

    if (count <= tris_per_chunk) {
        ChunkyTriMeshNode &leaf = mesh.nodes[node_index];
        leaf.index = (int32_t)(mesh.triangles.length / 3);
        leaf.count = count;
        for (uint32_t i = min; i < max; i++) {
            uint32_t triangle = items[i].index;
            mesh.triangles.push(indices[triangle * 3 + 0]);
            mesh.triangles.push(indices[triangle * 3 + 1]);
            mesh.triangles.push(indices[triangle * 3 + 2]);
        }
        return;
    }

    bool split_on_y = node.max.y - node.min.y > node.max.x - node.min.x;
    sort::stable(&items[min], count, [split_on_y](const BoundsItem &a, const BoundsItem &b) {
        return split_on_y ? a.min.y < b.min.y : a.min.x < b.min.x;
    });

    uint32_t split = min + count / 2;
    subdivide(items, min, split, tris_per_chunk, mesh, indices);
    subdivide(items, split, max, tris_per_chunk, mesh, indices);

    mesh.nodes[node_index].index = -(int32_t)(mesh.nodes.length - node_index);
}

ChunkyTriMesh create(const float *vertices, const uint32_t *indices, uint32_t index_count, uint32_t tris_per_chunk) {
    ChunkyTriMesh mesh = {};
    uint32_t triangle_count = index_count / 3;
    if (triangle_count == 0) return mesh;

    Array<BoundsItem> items = {};
    items.reserve(triangle_count);
    for (uint32_t i = 0; i < triangle_count; i++) items.push(calculate_triangle_bounds(vertices, indices, i));

    subdivide(items, 0, triangle_count, tris_per_chunk, mesh, indices);
    items.release();

    for (const ChunkyTriMeshNode &node : mesh.nodes) {
        if (is_leaf(node) && node.count > mesh.max_tris_per_chunk) mesh.max_tris_per_chunk = node.count;
    }

    return mesh;
}

void release(ChunkyTriMesh &mesh) {
    mesh.nodes.release();
    mesh.triangles.release();
}

bool is_leaf(const ChunkyTriMeshNode &node) { return node.index >= 0; }

static bool overlap_rect(Vec2 a_min, Vec2 a_max, Vec2 b_min, Vec2 b_max) {
    if (a_min.x > b_max.x || a_max.x < b_min.x) return false;
    if (a_min.y > b_max.y || a_max.y < b_min.y) return false;
    return true;
}

static bool overlap_segment(Vec2 p, Vec2 q, Vec2 b_min, Vec2 b_max) {
    float t_min = 0;
    float t_max = 1;
    const float start[2] = {p.x, p.y};
    const float delta[2] = {q.x - p.x, q.y - p.y};
    const float box_min[2] = {b_min.x, b_min.y};
    const float box_max[2] = {b_max.x, b_max.y};

    for (int i = 0; i < 2; i++) {
        if (fabsf(delta[i]) < scalar::EPSILON) {
            if (start[i] < box_min[i] || start[i] > box_max[i]) return false;
            continue;
        }

        float inverse = 1.0f / delta[i];
        float t1 = (box_min[i] - start[i]) * inverse;
        float t2 = (box_max[i] - start[i]) * inverse;
        if (t1 > t2) {
            float swap = t1;
            t1 = t2;
            t2 = swap;
        }

        t_min = fmaxf(t_min, t1);
        t_max = fminf(t_max, t2);
        if (t_min > t_max) return false;
    }

    return true;
}

struct ChunkQuery {
    bool segment;
    Vec2 a;
    Vec2 b;
};

static bool overlaps(const ChunkQuery &query, const ChunkyTriMeshNode &node) {
    return query.segment ? overlap_segment(query.a, query.b, node.min, node.max) : overlap_rect(query.a, query.b, node.min, node.max);
}

static Array<uint32_t> collect_chunks(const ChunkyTriMesh &mesh, const ChunkQuery &query) {
    Array<uint32_t> chunks = {};
    uint32_t i = 0;
    while (i < mesh.nodes.length) {
        const ChunkyTriMeshNode &node = mesh.nodes[i];
        bool overlap = overlaps(query, node);
        bool leaf = is_leaf(node);

        if (leaf && overlap) chunks.push(i);

        if (overlap || leaf) {
            i++;
        } else {
            i += (uint32_t)-node.index;
        }
    }
    return chunks;
}

Array<uint32_t> get_chunks_overlapping_rect(const ChunkyTriMesh &mesh, Vec2 min, Vec2 max) {
    return collect_chunks(mesh, {false, min, max});
}

Array<uint32_t> get_triangles_in_rect(const ChunkyTriMesh &mesh, Vec2 min, Vec2 max) {
    Array<uint32_t> chunks = get_chunks_overlapping_rect(mesh, min, max);
    Array<uint32_t> triangles = {};
    for (uint32_t chunk : chunks) {
        const ChunkyTriMeshNode &node = mesh.nodes[chunk];
        uint32_t start = (uint32_t)node.index * 3;
        for (uint32_t i = start; i < start + node.count * 3; i++) triangles.push(mesh.triangles[i]);
    }
    chunks.release();
    return triangles;
}

Array<uint32_t> get_chunks_overlapping_segment(const ChunkyTriMesh &mesh, Vec2 p, Vec2 q) {
    return collect_chunks(mesh, {true, p, q});
}

}

}
