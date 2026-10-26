#include "roam/blocks/geometry/chunky_tri_mesh.h"

#include <math.h>
#include <string.h>

#include "roam/math/scalar.h"

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

struct SortKey {
    uint32_t key;
    uint32_t item;
};

struct SortScratch {
    Array<BoundsItem> items;
    Array<SortKey> keys;
    Array<SortKey> swap;
};

static uint32_t sortable_key(float value) {
    if (value == 0.0f) value = 0.0f;
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x80000000u) ? ~bits : bits | 0x80000000u;
}

static void sort_by_min(BoundsItem *items, uint32_t count, bool split_on_y, SortScratch &scratch) {
    SortKey *keys = scratch.keys.data;
    SortKey *swap = scratch.swap.data;
    for (uint32_t i = 0; i < count; i++) keys[i] = {sortable_key(split_on_y ? items[i].min.y : items[i].min.x), i};

    for (uint32_t shift = 0; shift < 32; shift += 8) {
        uint32_t counts[257] = {};
        for (uint32_t i = 0; i < count; i++) counts[((keys[i].key >> shift) & 0xff) + 1]++;
        for (uint32_t digit = 0; digit < 256; digit++) counts[digit + 1] += counts[digit];
        for (uint32_t i = 0; i < count; i++) swap[counts[(keys[i].key >> shift) & 0xff]++] = keys[i];
        SortKey *sorted = swap;
        swap = keys;
        keys = sorted;
    }

    BoundsItem *ordered = scratch.items.data;
    for (uint32_t i = 0; i < count; i++) ordered[i] = items[keys[i].item];
    memcpy(items, ordered, count * sizeof(BoundsItem));
}

static void subdivide(Array<BoundsItem> &items, uint32_t min, uint32_t max, uint32_t tris_per_chunk, ChunkyTriMesh &mesh,
                      const uint32_t *indices, SortScratch &scratch) {
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
    sort_by_min(&items[min], count, split_on_y, scratch);

    uint32_t split = min + count / 2;
    subdivide(items, min, split, tris_per_chunk, mesh, indices, scratch);
    subdivide(items, split, max, tris_per_chunk, mesh, indices, scratch);

    mesh.nodes[node_index].index = -(int32_t)(mesh.nodes.length - node_index);
}

ChunkyTriMesh create(const InputTriangleMesh &input, uint32_t tris_per_chunk) {
    const float *vertices = input.positions.data;
    const uint32_t *indices = input.indices.data;
    uint32_t index_count = input.indices.length;
    ChunkyTriMesh mesh = {};
    uint32_t triangle_count = index_count / 3;
    if (triangle_count == 0) return mesh;

    Array<BoundsItem> items = {};
    items.reserve(triangle_count);
    for (uint32_t i = 0; i < triangle_count; i++) items.push(calculate_triangle_bounds(vertices, indices, i));

    SortScratch scratch = {};
    scratch.items.resize(triangle_count);
    scratch.keys.resize(triangle_count);
    scratch.swap.resize(triangle_count);
    subdivide(items, 0, triangle_count, tris_per_chunk, mesh, indices, scratch);
    scratch.items.release();
    scratch.keys.release();
    scratch.swap.release();
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

static void collect_chunks(ChunkyTriMeshChunksResult &out, const ChunkyTriMesh &mesh, const ChunkQuery &query, Span<uint32_t> chunks) {
    out = {};
    uint32_t i = 0;
    while (i < mesh.nodes.length) {
        const ChunkyTriMeshNode &node = mesh.nodes[i];
        bool overlap = overlaps(query, node);
        bool leaf = is_leaf(node);

        if (leaf && overlap) {
            if (out.chunk_count < chunks.length) chunks[out.chunk_count++] = i;
            out.chunk_total++;
        }

        if (overlap || leaf) {
            i++;
        } else {
            i += (uint32_t)-node.index;
        }
    }
    out.truncated = out.chunk_count < out.chunk_total;
}

void get_chunks_overlapping_rect(ChunkyTriMeshChunksResult &out, const ChunkyTriMesh &mesh, Vec2 min, Vec2 max, Span<uint32_t> chunks) {
    collect_chunks(out, mesh, {false, min, max}, chunks);
}

void get_triangles_in_rect(ChunkyTriMeshTrianglesResult &out, const ChunkyTriMesh &mesh, Vec2 min, Vec2 max, Span<uint32_t> indices) {
    out = {};
    ChunkQuery query = {false, min, max};
    uint32_t i = 0;
    while (i < mesh.nodes.length) {
        const ChunkyTriMeshNode &node = mesh.nodes[i];
        bool overlap = overlaps(query, node);
        bool leaf = is_leaf(node);

        if (leaf && overlap) {
            uint32_t start = (uint32_t)node.index * 3;
            for (uint32_t j = start; j < start + node.count * 3; j++) {
                if (out.index_count < indices.length) indices[out.index_count++] = mesh.triangles[j];
                out.index_total++;
            }
        }

        if (overlap || leaf) {
            i++;
        } else {
            i += (uint32_t)-node.index;
        }
    }
    out.truncated = out.index_count < out.index_total;
}

void get_chunks_overlapping_segment(ChunkyTriMeshChunksResult &out, const ChunkyTriMesh &mesh, Vec2 p, Vec2 q, Span<uint32_t> chunks) {
    collect_chunks(out, mesh, {true, p, q}, chunks);
}

}

}
