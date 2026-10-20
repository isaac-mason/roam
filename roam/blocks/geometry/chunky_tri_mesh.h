#pragma once

#include <stdint.h>

#include "roam/math/vec2.h"
#include "roam/util/array.h"

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

namespace chunky_tri_mesh {

ChunkyTriMesh create(const float *vertices, const uint32_t *indices, uint32_t index_count, uint32_t tris_per_chunk = 256);

void release(ChunkyTriMesh &mesh);

bool is_leaf(const ChunkyTriMeshNode &node);

Array<uint32_t> get_chunks_overlapping_rect(const ChunkyTriMesh &mesh, Vec2 min, Vec2 max);

Array<uint32_t> get_triangles_in_rect(const ChunkyTriMesh &mesh, Vec2 min, Vec2 max);

Array<uint32_t> get_chunks_overlapping_segment(const ChunkyTriMesh &mesh, Vec2 p, Vec2 q);

}

}
