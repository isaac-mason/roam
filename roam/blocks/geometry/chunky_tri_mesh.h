#pragma once

#include <stdint.h>

#include "roam/math/vec2.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/util/array.h"
#include "roam/util/span.h"

namespace roam {

struct ChunkyTriMeshNode {
    Vec2 min;
    Vec2 max;
    int32_t index;
    uint32_t count;
};

struct ChunkyTriMesh {
    Array<ChunkyTriMeshNode> nodes;
    Array<uint32_t> triangles;
    uint32_t max_tris_per_chunk;
};

struct ChunkyTriMeshChunksResult {
    uint32_t chunk_count;
    uint32_t chunk_total;
    bool truncated;
};

struct ChunkyTriMeshTrianglesResult {
    uint32_t index_count;
    uint32_t index_total;
    bool truncated;
};

namespace chunky_tri_mesh {

ChunkyTriMesh create(const InputTriangleMesh &input, uint32_t tris_per_chunk = 256);

void release(ChunkyTriMesh &mesh);

bool is_leaf(const ChunkyTriMeshNode &node);

void get_chunks_overlapping_rect(ChunkyTriMeshChunksResult &out, const ChunkyTriMesh &mesh, Vec2 min, Vec2 max, Span<uint32_t> chunks);

void get_triangles_in_rect(ChunkyTriMeshTrianglesResult &out, const ChunkyTriMesh &mesh, Vec2 min, Vec2 max, Span<uint32_t> indices);

void get_chunks_overlapping_segment(ChunkyTriMeshChunksResult &out, const ChunkyTriMesh &mesh, Vec2 p, Vec2 q, Span<uint32_t> chunks);

}

}
