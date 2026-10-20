#pragma once

#include <stdint.h>

#include "roam/generate/build_context.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/generate/poly_mesh.h"
#include "roam/util/array.h"

namespace roam {

constexpr int DETAIL_EDGE_BOUNDARY = 0x1;

struct PolyMeshDetail {
    Array<int> meshes;
    Array<float> vertices;
    Array<int> triangles;
    int mesh_count;
    int vertex_count;
    int triangle_count;
};

namespace poly_mesh_detail {

PolyMeshDetail build_poly_mesh_detail(BuildContext &context, const PolyMesh &poly_mesh, const CompactHeightfield &compact_heightfield,
                                      float sample_distance, float sample_max_error);

void release(PolyMeshDetail &poly_mesh_detail);

}

}
