#include "app/nav_mesh_config.h"

#include <math.h>

#include "imgui.h"

using namespace roam;

namespace nav_mesh_config {

NavMeshConfig create() {
    return {
        .cell_size = 0.15f,
        .cell_height = 0.15f,
        .walkable_radius_world = 0.1f,
        .walkable_climb_world = 0.5f,
        .walkable_height_world = 0.25f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 5,
        .detail_sample_distance = 6,
        .detail_sample_max_error = 1,
        .tile_size_voxels = 32,
    };
}

SoloNavMeshOptions solo_options(const NavMeshConfig &config) {
    return {
        .cell_size = config.cell_size,
        .cell_height = config.cell_height,
        .walkable_radius_voxels = (int)ceilf(config.walkable_radius_world / config.cell_size),
        .walkable_radius_world = config.walkable_radius_world,
        .walkable_climb_voxels = (int)ceilf(config.walkable_climb_world / config.cell_height),
        .walkable_climb_world = config.walkable_climb_world,
        .walkable_height_voxels = (int)ceilf(config.walkable_height_world / config.cell_height),
        .walkable_height_world = config.walkable_height_world,
        .walkable_slope_angle_degrees = config.walkable_slope_angle_degrees,
        .border_size = config.border_size,
        .min_region_area = config.min_region_area,
        .merge_region_area = config.merge_region_area,
        .max_simplification_error = config.max_simplification_error,
        .max_edge_length = config.max_edge_length,
        .max_vertices_per_poly = config.max_vertices_per_poly,
        .detail_sample_distance = config.detail_sample_distance < 0.9f ? 0 : config.cell_size * config.detail_sample_distance,
        .detail_sample_max_error = config.cell_height * config.detail_sample_max_error,
    };
}

TiledNavMeshOptions tiled_options(const NavMeshConfig &config) {
    SoloNavMeshOptions solo = solo_options(config);
    return {
        .cell_size = solo.cell_size,
        .cell_height = solo.cell_height,
        .tile_size_voxels = config.tile_size_voxels,
        .tile_size_world = (float)config.tile_size_voxels * config.cell_size,
        .walkable_radius_voxels = solo.walkable_radius_voxels,
        .walkable_radius_world = solo.walkable_radius_world,
        .walkable_climb_voxels = solo.walkable_climb_voxels,
        .walkable_climb_world = solo.walkable_climb_world,
        .walkable_height_voxels = solo.walkable_height_voxels,
        .walkable_height_world = solo.walkable_height_world,
        .walkable_slope_angle_degrees = solo.walkable_slope_angle_degrees,
        .border_size = solo.walkable_radius_voxels + 3,
        .min_region_area = solo.min_region_area,
        .merge_region_area = solo.merge_region_area,
        .max_simplification_error = solo.max_simplification_error,
        .max_edge_length = solo.max_edge_length,
        .max_vertices_per_poly = solo.max_vertices_per_poly,
        .detail_sample_distance = solo.detail_sample_distance,
        .detail_sample_max_error = solo.detail_sample_max_error,
    };
}

bool ui(NavMeshConfig &config, bool tiled) {
    bool changed = false;
    if (ImGui::CollapsingHeader("Heightfield")) {
        changed |= ImGui::SliderFloat("cell size", &config.cell_size, 0.01f, 1, "%.2f");
        changed |= ImGui::SliderFloat("cell height", &config.cell_height, 0.01f, 1, "%.2f");
        if (tiled) changed |= ImGui::SliderInt("tile size (voxels)", &config.tile_size_voxels, 8, 256);
    }
    if (ImGui::CollapsingHeader("Agent")) {
        changed |= ImGui::SliderFloat("radius", &config.walkable_radius_world, 0, 2, "%.2f");
        changed |= ImGui::SliderFloat("climb", &config.walkable_climb_world, 0, 2, "%.2f");
        changed |= ImGui::SliderFloat("height", &config.walkable_height_world, 0, 2, "%.2f");
        changed |= ImGui::SliderFloat("slope", &config.walkable_slope_angle_degrees, 0, 90, "%.0f");
    }
    if (ImGui::CollapsingHeader("Region")) {
        if (!tiled) changed |= ImGui::SliderInt("border size", &config.border_size, 0, 10);
        changed |= ImGui::SliderInt("min region area", &config.min_region_area, 0, 50);
        changed |= ImGui::SliderInt("merge region area", &config.merge_region_area, 0, 50);
    }
    if (ImGui::CollapsingHeader("Contour")) {
        changed |= ImGui::SliderFloat("max simplification error", &config.max_simplification_error, 0.1f, 10, "%.1f");
        changed |= ImGui::SliderInt("max edge length", &config.max_edge_length, 0, 50);
    }
    if (ImGui::CollapsingHeader("Poly mesh")) {
        changed |= ImGui::SliderInt("max vertices per poly", &config.max_vertices_per_poly, 3, 12);
    }
    if (ImGui::CollapsingHeader("Detail")) {
        changed |= ImGui::SliderFloat("sample distance", &config.detail_sample_distance, 0, 16, "%.1f");
        changed |= ImGui::SliderFloat("sample max error", &config.detail_sample_max_error, 0, 16, "%.1f");
    }
    return changed;
}

}
