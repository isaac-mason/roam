#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"

namespace roam {

namespace poly_neighbours {

void build_poly_neighbours(Array<NavMeshPoly> &polys, const float *vertices, uint32_t vertex_count, int border_size, float min_x, float min_z,
                           float max_x, float max_z);

void build_mesh_adjacency(Array<NavMeshPoly> &polys, uint32_t vertex_count);

void find_portal_edges(Array<NavMeshPoly> &polys, const float *vertices, float min_x, float min_z, float max_x, float max_z);

void finalize_poly_neighbours(Array<NavMeshPoly> &polys);

}

}
