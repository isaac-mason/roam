#include "roam/generate/common.h"
#include "roam/generate/heightfield.h"
#include "roam/generate/input_triangle_mesh.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_input.h"

using namespace roam;

struct FlatHeightfield {
    Array<int32_t> column_span_counts;
    Array<int32_t> span_min;
    Array<int32_t> span_max;
    Array<int32_t> span_area;
};

static FlatHeightfield flatten(const Heightfield &heightfield) {
    FlatHeightfield flat = {};
    for (int32_t first : heightfield.spans) {
        int32_t count = 0;
        for (int32_t index = first; index != NULL_SPAN; index = heightfield.span_pool[(uint32_t)index].next) {
            const HeightfieldSpan &span = heightfield.span_pool[(uint32_t)index];
            flat.span_min.push(span.min);
            flat.span_max.push(span.max);
            flat.span_area.push(span.area);
            count++;
        }
        flat.column_span_counts.push(count);
    }
    return flat;
}

static void release(FlatHeightfield &flat) {
    flat.column_span_counts.release();
    flat.span_min.release();
    flat.span_max.release();
    flat.span_area.release();
}

static void check_heightfield_stage(const fixture::Fixture &nav_test, const char *stage, const Heightfield &heightfield) {
    char name[256];
    FlatHeightfield flat = flatten(heightfield);
    CHECK(flat.span_min.length > 1000);

    snprintf(name, sizeof(name), "heightfield.%s.column_span_counts", stage);
    CHECK_I32_ARRAY(name, flat.column_span_counts.data, flat.column_span_counts.length, fixture::i32(nav_test, name));
    snprintf(name, sizeof(name), "heightfield.%s.span_min", stage);
    CHECK_I32_ARRAY(name, flat.span_min.data, flat.span_min.length, fixture::i32(nav_test, name));
    snprintf(name, sizeof(name), "heightfield.%s.span_max", stage);
    CHECK_I32_ARRAY(name, flat.span_max.data, flat.span_max.length, fixture::i32(nav_test, name));
    snprintf(name, sizeof(name), "heightfield.%s.span_area", stage);
    CHECK_I32_ARRAY(name, flat.span_area.data, flat.span_area.length, fixture::i32(nav_test, name));

    release(flat);
}

TEST(heightfield_matches_navcat_on_nav_test) {
    fixture::Fixture nav_test = fixture::load("nav_test_solo");
    REQUIRE(nav_test.loaded);

    NavTestInput input = load_nav_test_input(nav_test);
    float cell_size = (float)fixture::number(nav_test, "options.cell_size");
    float cell_height = (float)fixture::number(nav_test, "options.cell_height");
    float walkable_slope_angle = (float)fixture::number(nav_test, "options.walkable_slope_angle_degrees");
    int walkable_climb = fixture::i32(nav_test, "options.walkable_climb_voxels").data[0];
    int walkable_height = fixture::i32(nav_test, "options.walkable_height_voxels").data[0];

    Array<uint8_t> tri_area_ids = {};
    tri_area_ids.resize(input.indices.length / 3, NULL_AREA);
    input_triangle_mesh::mark_walkable_triangles(tri_area_ids, {input.positions, input.indices}, walkable_slope_angle);

    Array<int32_t> tri_area_ids_wide = {};
    for (uint8_t area : tri_area_ids) tri_area_ids_wide.push(area);
    CHECK_I32_ARRAY("tri_area_ids", tri_area_ids_wide.data, tri_area_ids_wide.length, fixture::i32(nav_test, "tri_area_ids"));

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds({input.positions, input.indices});
    float bounds_values[6] = {bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z};
    CHECK_F64_ARRAY("mesh_bounds", bounds_values, 6, fixture::f64(nav_test, "mesh_bounds"), 1e-5);

    GridSize grid_size = heightfield::calculate_grid_size(bounds, cell_size);
    int32_t grid_size_values[2] = {grid_size.width, grid_size.height};
    CHECK_I32_ARRAY("grid_size", grid_size_values, 2, fixture::i32(nav_test, "grid_size"));

    BuildContext context = {};
    Heightfield heightfield = heightfield::create(grid_size.width, grid_size.height, bounds, cell_size, cell_height);

    heightfield::rasterize_triangles(context, heightfield, {input.positions, input.indices}, tri_area_ids, walkable_climb);
    check_heightfield_stage(nav_test, "rasterized", heightfield);

    heightfield::filter_low_hanging_walkable_obstacles(heightfield, walkable_climb);
    check_heightfield_stage(nav_test, "filtered_low_hanging", heightfield);

    heightfield::filter_ledge_spans(heightfield, walkable_height, walkable_climb);
    check_heightfield_stage(nav_test, "filtered_ledges", heightfield);

    heightfield::filter_walkable_low_height_spans(heightfield, walkable_height);
    check_heightfield_stage(nav_test, "filtered_low_height", heightfield);

    heightfield::release(heightfield);
    build_context::release(context);
    tri_area_ids_wide.release();
    tri_area_ids.release();
    release(input);
    fixture::release(nav_test);
}
