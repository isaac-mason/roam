#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

static void check_regions(const char *prefix, const CompactHeightfield &compact_heightfield, const fixture::Fixture &expected) {
    char name[256];
    snprintf(name, sizeof(name), "%s.span_region", prefix);
    CHECK_EXPECTED(name, widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.region; }), expected);
    snprintf(name, sizeof(name), "%s.max_regions", prefix);
    int32_t max_regions = compact_heightfield.max_regions;
    CHECK_I32_ARRAY(name, &max_regions, 1, fixture::i32(expected, name));
}

TEST(compact_heightfield_matches_navcat_on_nav_test) {
    NavTest nav_test = load_nav_test();
    REQUIRE(nav_test.fixture.loaded);
    const fixture::Fixture &expected = nav_test.fixture;

    CompactHeightfield compact_heightfield =
        compact_heightfield::build(nav_test.context, nav_test.heightfield, nav_test.options.walkable_height, nav_test.options.walkable_climb);
    CHECK(compact_heightfield.span_count > 1000);

    int32_t span_count = (int32_t)compact_heightfield.span_count;
    CHECK_I32_ARRAY("compact_heightfield.built.span_count", &span_count, 1, fixture::i32(expected, "compact_heightfield.built.span_count"));
    const Box3 &bounds = compact_heightfield.bounds;
    float bounds_values[6] = {bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z};
    CHECK_F64_ARRAY("compact_heightfield.built.bounds", bounds_values, 6, fixture::f64(expected, "compact_heightfield.built.bounds"), 1e-5);
    CHECK_EXPECTED("compact_heightfield.built.cell_index", widen(compact_heightfield.cells, [](const CompactHeightfieldCell &cell) { return cell.index; }),
                   expected);
    CHECK_EXPECTED("compact_heightfield.built.cell_count", widen(compact_heightfield.cells, [](const CompactHeightfieldCell &cell) { return cell.count; }),
                   expected);
    CHECK_EXPECTED("compact_heightfield.built.span_y", widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.y; }),
                   expected);
    CHECK_EXPECTED("compact_heightfield.built.span_h", widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.h; }),
                   expected);
    CHECK_EXPECTED("compact_heightfield.built.span_con", widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.con; }),
                   expected);
    CHECK_EXPECTED("compact_heightfield.built.areas", widen(compact_heightfield.areas, [](uint8_t area) { return area; }), expected);

    compact_heightfield::erode_walkable_area(compact_heightfield, nav_test.options.walkable_radius);
    CHECK_EXPECTED("compact_heightfield.eroded.areas", widen(compact_heightfield.areas, [](uint8_t area) { return area; }), expected);

    compact_heightfield_regions::build_distance_field(compact_heightfield);
    CHECK_EXPECTED("compact_heightfield.distances", widen(compact_heightfield.distances, [](uint16_t distance) { return distance; }), expected);
    int32_t max_distance = compact_heightfield.max_distance;
    CHECK_I32_ARRAY("compact_heightfield.max_distance", &max_distance, 1, fixture::i32(expected, "compact_heightfield.max_distance"));

    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

static void check_region_method(RegionMethod method, const char *prefix) {
    NavTest nav_test = load_nav_test();
    REQUIRE(nav_test.fixture.loaded);

    CompactHeightfield compact_heightfield = build_with_regions(nav_test, method);
    check_regions(prefix, compact_heightfield, nav_test.fixture);

    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

TEST(watershed_regions_match_navcat_on_nav_test) { check_region_method(REGION_METHOD_WATERSHED, "regions.watershed"); }

TEST(monotone_regions_match_navcat_on_nav_test) { check_region_method(REGION_METHOD_MONOTONE, "regions.monotone"); }

TEST(layer_regions_match_navcat_on_nav_test) { check_region_method(REGION_METHOD_LAYERS, "regions.layers"); }
