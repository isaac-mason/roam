#pragma once

#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/contour_set.h"
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

struct NavTest {
    fixture::Fixture fixture;
    NavTestInput input;
    NavTestOptions options;
    roam::BuildContext context;
    roam::Heightfield heightfield;
};

inline NavTest load_nav_test() {
    NavTest nav_test = {};
    nav_test.fixture = fixture::load("nav_test_solo");
    if (!nav_test.fixture.loaded) return nav_test;
    nav_test.input = load_nav_test_input(nav_test.fixture);
    nav_test.options = load_nav_test_options(nav_test.fixture);
    nav_test.heightfield = build_filtered_heightfield(nav_test.context, nav_test.input, nav_test.options);
    return nav_test;
}

inline void release(NavTest &nav_test) {
    roam::heightfield::release(nav_test.heightfield);
    roam::build_context::release(nav_test.context);
    release(nav_test.input);
    fixture::release(nav_test.fixture);
}

inline roam::CompactHeightfield build_eroded_with_distances(NavTest &nav_test) {
    using namespace roam;
    CompactHeightfield compact_heightfield =
        compact_heightfield::build(nav_test.context, nav_test.heightfield, nav_test.options.walkable_height, nav_test.options.walkable_climb);
    compact_heightfield::erode_walkable_area(compact_heightfield, nav_test.options.walkable_radius);
    compact_heightfield_regions::build_distance_field(compact_heightfield);
    return compact_heightfield;
}

enum RegionMethod {
    REGION_METHOD_WATERSHED,
    REGION_METHOD_MONOTONE,
    REGION_METHOD_LAYERS,
};

inline roam::CompactHeightfield build_with_regions(NavTest &nav_test, RegionMethod method) {
    using namespace roam;
    CompactHeightfield compact_heightfield = build_eroded_with_distances(nav_test);
    const NavTestOptions &options = nav_test.options;
    switch (method) {
        case REGION_METHOD_WATERSHED:
            compact_heightfield_regions::build_regions(nav_test.context, compact_heightfield, options.border_size, options.min_region_area,
                                                       options.merge_region_area);
            break;
        case REGION_METHOD_MONOTONE:
            compact_heightfield_regions::build_regions_monotone(nav_test.context, compact_heightfield, options.border_size, options.min_region_area,
                                                                options.merge_region_area);
            break;
        case REGION_METHOD_LAYERS:
            compact_heightfield_regions::build_layer_regions(nav_test.context, compact_heightfield, options.border_size, options.min_region_area);
            break;
    }
    return compact_heightfield;
}

inline uint32_t contour_flags_for(RegionMethod method) {
    return method == REGION_METHOD_WATERSHED ? roam::CONTOUR_TESS_WALL_EDGES : roam::CONTOUR_TESS_WALL_EDGES | roam::CONTOUR_TESS_AREA_EDGES;
}

inline roam::ContourSet build_contour_set(NavTest &nav_test, const roam::CompactHeightfield &compact_heightfield, RegionMethod method) {
    return roam::contour_set::build_contours(nav_test.context, compact_heightfield, nav_test.options.max_simplification_error,
                                             nav_test.options.max_edge_length, contour_flags_for(method));
}

template <typename T, typename Field>
inline roam::Array<int32_t> widen(const roam::Array<T> &values, Field field) {
    roam::Array<int32_t> wide = {};
    for (const T &value : values) wide.push((int32_t)field(value));
    return wide;
}

inline void check_expected(const char *file, int line, const char *name, roam::Array<int32_t> actual, const fixture::Fixture &expected) {
    fixture::check_i32(file, line, name, actual.data, actual.length, fixture::i32(expected, name));
    actual.release();
}

#define CHECK_EXPECTED(name, actual, expected) check_expected(__FILE__, __LINE__, name, actual, expected)
