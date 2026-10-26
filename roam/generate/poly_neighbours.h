#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/util/span.h"

namespace roam {

namespace poly_neighbours {

void build_poly_neighbours(const Array<NavMeshPoly> &polys, const Array<uint32_t> &poly_vertices, Array<int> &poly_neis, Span<const float> vertices,
                           int border_size, float min_x, float min_z, float max_x, float max_z);

void build_mesh_adjacency(const Array<NavMeshPoly> &polys, const Array<uint32_t> &poly_vertices, Array<int> &poly_neis, uint32_t vertex_count);

void find_portal_edges(const Array<NavMeshPoly> &polys, const Array<uint32_t> &poly_vertices, Array<int> &poly_neis, Span<const float> vertices,
                       float min_x, float min_z, float max_x, float max_z);

void finalize_poly_neighbours(Array<int> &poly_neis);

}

}
