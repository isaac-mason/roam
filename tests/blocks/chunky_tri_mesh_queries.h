#pragma once

#include "roam/blocks/geometry/chunky_tri_mesh.h"

using namespace roam;

inline Array<uint32_t> chunks_in_rect(const ChunkyTriMesh &mesh, Vec2 min, Vec2 max) {
    Array<uint32_t> chunks = {};
    chunks.reserve(mesh.nodes.length + 1);
    ChunkyTriMeshChunksResult found;
    chunky_tri_mesh::get_chunks_overlapping_rect(found, mesh, min, max, chunks);
    chunks.resize(found.chunk_count);
    return chunks;
}

inline Array<uint32_t> chunks_on_segment(const ChunkyTriMesh &mesh, Vec2 p, Vec2 q) {
    Array<uint32_t> chunks = {};
    chunks.reserve(mesh.nodes.length + 1);
    ChunkyTriMeshChunksResult found;
    chunky_tri_mesh::get_chunks_overlapping_segment(found, mesh, p, q, chunks);
    chunks.resize(found.chunk_count);
    return chunks;
}

inline Array<uint32_t> triangles_in_rect(const ChunkyTriMesh &mesh, Vec2 min, Vec2 max) {
    Array<uint32_t> indices = {};
    indices.reserve(mesh.triangles.length + 1);
    ChunkyTriMeshTrianglesResult found;
    chunky_tri_mesh::get_triangles_in_rect(found, mesh, min, max, indices);
    indices.resize(found.index_count);
    return indices;
}
