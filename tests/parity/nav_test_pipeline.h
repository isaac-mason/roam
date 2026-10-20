#pragma once

#include "roam/generate/common.h"
#include "roam/generate/heightfield.h"
#include "roam/generate/input_triangle_mesh.h"
#include "tests/parity/nav_test_input.h"

inline roam::Heightfield build_filtered_heightfield(roam::BuildContext &context, const NavTestInput &input, const NavTestOptions &options) {
    using namespace roam;

    Array<uint8_t> tri_area_ids = {};
    tri_area_ids.resize(input.indices.length / 3, NULL_AREA);
    input_triangle_mesh::mark_walkable_triangles(input.positions.data, input.indices.data, input.indices.length, tri_area_ids.data,
                                                 options.walkable_slope_angle);

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(input.positions.data, input.indices.data, input.indices.length);
    GridSize grid_size = heightfield::calculate_grid_size(bounds, options.cell_size);

    Heightfield heightfield = heightfield::create(grid_size.width, grid_size.height, bounds, options.cell_size, options.cell_height);
    heightfield::rasterize_triangles(context, heightfield, input.positions.data, input.indices.data, input.indices.length, tri_area_ids.data,
                                     options.walkable_climb);
    heightfield::filter_low_hanging_walkable_obstacles(heightfield, options.walkable_climb);
    heightfield::filter_ledge_spans(heightfield, options.walkable_height, options.walkable_climb);
    heightfield::filter_walkable_low_height_spans(heightfield, options.walkable_height);

    tri_area_ids.release();
    return heightfield;
}
