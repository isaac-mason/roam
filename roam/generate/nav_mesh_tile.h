#pragma once

#include <stdint.h>

#include "roam/generate/poly_mesh.h"
#include "roam/generate/poly_mesh_detail.h"
#include "roam/math/box3.h"
#include "roam/query/nav_mesh.h"

namespace roam {

struct NavMeshTilePolys {
    Array<float> vertices;
    Array<NavMeshPoly> polys;
    Array<uint32_t> poly_vertices;
    Array<int> poly_neis;
};

struct NavMeshTileDetailMesh {
    Array<NavMeshPolyDetail> detail_meshes;
    Array<float> detail_vertices;
    Array<uint8_t> detail_triangles;
};

struct ExternalPolygon {
    const uint32_t *vertices;
    uint32_t vertex_count;
    uint32_t flags;
    uint8_t area;
};

namespace nav_mesh_tile {

NavMeshTilePolys poly_mesh_to_tile_polys(const PolyMesh &poly_mesh);

NavMeshTilePolys polygons_to_nav_mesh_tile_polys(const ExternalPolygon *polygons, uint32_t polygon_count, const float *vertices,
                                                 uint32_t vertex_count, int border_size, Box3 bounds);

NavMeshTileDetailMesh polys_to_tile_detail_mesh(const Array<NavMeshPoly> &polys);

NavMeshTileDetailMesh poly_mesh_detail_to_tile_detail_mesh(const Array<NavMeshPoly> &polys, const PolyMeshDetail &poly_mesh_detail);

void release(NavMeshTilePolys &tile_polys);
void release(NavMeshTileDetailMesh &detail_mesh);

}

}
