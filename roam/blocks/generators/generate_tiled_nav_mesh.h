#pragma once

#include <stdint.h>

#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "roam/generate/build_context.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/generate/contour_set.h"
#include "roam/generate/heightfield.h"
#include "roam/generate/poly_mesh.h"
#include "roam/generate/poly_mesh_detail.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/math/box3.h"
#include "roam/query/nav_mesh.h"
#include "roam/util/array.h"

namespace roam {

struct TiledNavMeshOptions {
    float cell_size;
    float cell_height;
    int tile_size_voxels;
    float tile_size_world;
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

struct TiledNavMeshIntermediates {
    BuildContext build_context;
    InputTriangleMesh input;
    Box3 input_bounds;
    ChunkyTriMesh chunky_tri_mesh;
    Array<Array<uint8_t>> tri_area_ids;
    Array<Heightfield> heightfield;
    Array<CompactHeightfield> compact_heightfield;
    Array<ContourSet> contour_set;
    Array<PolyMesh> poly_mesh;
    Array<PolyMeshDetail> poly_mesh_detail;
};

struct TiledNavMeshResult {
    NavMesh nav_mesh;
    TiledNavMeshIntermediates intermediates;
};

namespace generate_tiled_nav_mesh {

TiledNavMeshOptions derive_sizes(TiledNavMeshOptions options);

TiledNavMeshResult generate_tiled_nav_mesh(const InputTriangleMesh &input, const TiledNavMeshOptions &options);

void release(TiledNavMeshIntermediates &intermediates);

}

}
