#pragma once

#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"

struct NavMeshConfig {
    float cell_size;
    float cell_height;
    float walkable_radius_world;
    float walkable_climb_world;
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
    int tile_size_voxels;
};

namespace nav_mesh_config {

NavMeshConfig create();
roam::SoloNavMeshOptions solo_options(const NavMeshConfig &config);
roam::TiledNavMeshOptions tiled_options(const NavMeshConfig &config);
bool ui(NavMeshConfig &config, bool tiled);

}
