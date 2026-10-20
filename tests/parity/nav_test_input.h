#pragma once

#include "roam/util/array.h"
#include "tests/harness/fixture.h"

struct NavTestInput {
    roam::Array<float> positions;
    roam::Array<uint32_t> indices;
};

inline NavTestInput load_nav_test_input(const fixture::Fixture &nav_test) {
    NavTestInput input = {};
    fixture::F64 positions = fixture::f64(nav_test, "input.positions");
    fixture::I32 indices = fixture::i32(nav_test, "input.indices");
    for (uint32_t i = 0; i < positions.count; i++) input.positions.push((float)positions.data[i]);
    for (uint32_t i = 0; i < indices.count; i++) input.indices.push((uint32_t)indices.data[i]);
    return input;
}

inline void release(NavTestInput &input) {
    input.positions.release();
    input.indices.release();
}

struct NavTestOptions {
    float cell_size;
    float cell_height;
    float walkable_slope_angle;
    int walkable_radius;
    int walkable_climb;
    int walkable_height;
    int border_size;
    int min_region_area;
    int merge_region_area;
    float max_simplification_error;
    int max_edge_length;
};

inline NavTestOptions load_nav_test_options(const fixture::Fixture &nav_test) {
    NavTestOptions options;
    options.cell_size = (float)fixture::number(nav_test, "options.cell_size");
    options.cell_height = (float)fixture::number(nav_test, "options.cell_height");
    options.walkable_slope_angle = (float)fixture::number(nav_test, "options.walkable_slope_angle_degrees");
    options.walkable_radius = fixture::i32(nav_test, "options.walkable_radius_voxels").data[0];
    options.walkable_climb = fixture::i32(nav_test, "options.walkable_climb_voxels").data[0];
    options.walkable_height = fixture::i32(nav_test, "options.walkable_height_voxels").data[0];
    options.border_size = fixture::i32(nav_test, "options.border_size").data[0];
    options.min_region_area = fixture::i32(nav_test, "options.min_region_area").data[0];
    options.merge_region_area = fixture::i32(nav_test, "options.merge_region_area").data[0];
    options.max_simplification_error = (float)fixture::number(nav_test, "options.max_simplification_error");
    options.max_edge_length = fixture::i32(nav_test, "options.max_edge_length").data[0];
    return options;
}
