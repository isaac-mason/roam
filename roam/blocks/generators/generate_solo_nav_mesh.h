#pragma once

#include <stdint.h>

#include "roam/generate/build_context.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/generate/contour_set.h"
#include "roam/generate/heightfield.h"
#include "roam/generate/poly_mesh.h"
#include "roam/generate/poly_mesh_detail.h"
#include "roam/query/nav_mesh.h"
#include "roam/util/array.h"

namespace roam {

struct SoloNavMeshOptions {
    float cell_size;
    float cell_height;
    int walkable_radius_voxels;
    float walkable_radius_world;
    int walkable_climb_voxels;
    float walkable_climb_world;
    int walkable_height_voxels;
    float walkable_height_world;
    float walkable_slope_angle_degrees;
    int border_size;
    int min_region_area;
    int merge_region_area;
    float max_simplification_error;
    int max_edge_length;
    int max_vertices_per_poly;
    float detail_sample_distance;
    float detail_sample_max_error;
};

struct SoloNavMeshIntermediates {
    BuildContext build_context;
    InputTriangleMesh input;
    Array<uint8_t> tri_area_ids;
    Heightfield heightfield;
    CompactHeightfield compact_heightfield;
    ContourSet contour_set;
    PolyMesh poly_mesh;
    PolyMeshDetail poly_mesh_detail;
};

struct SoloNavMeshResult {
    NavMesh nav_mesh;
    SoloNavMeshIntermediates intermediates;
};

namespace generate_solo_nav_mesh {

SoloNavMeshResult generate_solo_nav_mesh(const InputTriangleMesh &input, const SoloNavMeshOptions &options);

void release(SoloNavMeshIntermediates &intermediates);

}

}
