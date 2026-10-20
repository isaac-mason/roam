#include "roam/generate/compact_heightfield.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

template <typename T, typename Field>
static Array<int32_t> widen(const Array<T> &values, Field field) {
    Array<int32_t> wide = {};
    for (const T &value : values) wide.push((int32_t)field(value));
    return wide;
}

static void check_field(const char *name, Array<int32_t> actual, const fixture::Fixture &nav_test) {
    CHECK_I32_ARRAY(name, actual.data, actual.length, fixture::i32(nav_test, name));
    actual.release();
}

static void check_regions(const char *prefix, const CompactHeightfield &compact_heightfield, const fixture::Fixture &nav_test) {
    char name[256];
    snprintf(name, sizeof(name), "%s.span_region", prefix);
    check_field(name, widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.region; }), nav_test);
    snprintf(name, sizeof(name), "%s.max_regions", prefix);
    int32_t max_regions = compact_heightfield.max_regions;
    CHECK_I32_ARRAY(name, &max_regions, 1, fixture::i32(nav_test, name));
}

struct NavTestCompact {
    fixture::Fixture fixture;
    NavTestInput input;
    NavTestOptions options;
    BuildContext context;
    Heightfield heightfield;
};

static NavTestCompact load_nav_test_compact() {
    NavTestCompact nav_test = {};
    nav_test.fixture = fixture::load("nav_test_solo");
    if (!nav_test.fixture.loaded) return nav_test;
    nav_test.input = load_nav_test_input(nav_test.fixture);
    nav_test.options = load_nav_test_options(nav_test.fixture);
    nav_test.heightfield = build_filtered_heightfield(nav_test.context, nav_test.input, nav_test.options);
    return nav_test;
}

static CompactHeightfield build_eroded_with_distances(NavTestCompact &nav_test) {
    CompactHeightfield compact_heightfield =
        compact_heightfield::build(nav_test.context, nav_test.options.walkable_height, nav_test.options.walkable_climb, nav_test.heightfield);
    compact_heightfield::erode_walkable_area(nav_test.options.walkable_radius, compact_heightfield);
    compact_heightfield_regions::build_distance_field(compact_heightfield);
    return compact_heightfield;
}

static void release(NavTestCompact &nav_test) {
    heightfield::release(nav_test.heightfield);
    build_context::release(nav_test.context);
    release(nav_test.input);
    fixture::release(nav_test.fixture);
}

TEST(compact_heightfield_matches_navcat_on_nav_test) {
    NavTestCompact nav_test = load_nav_test_compact();
    REQUIRE(nav_test.fixture.loaded);
    const fixture::Fixture &expected = nav_test.fixture;

    CompactHeightfield compact_heightfield =
        compact_heightfield::build(nav_test.context, nav_test.options.walkable_height, nav_test.options.walkable_climb, nav_test.heightfield);
    CHECK(compact_heightfield.span_count > 1000);

    int32_t span_count = (int32_t)compact_heightfield.span_count;
    CHECK_I32_ARRAY("compact_heightfield.built.span_count", &span_count, 1, fixture::i32(expected, "compact_heightfield.built.span_count"));
    const Box3 &bounds = compact_heightfield.bounds;
    float bounds_values[6] = {bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z};
    CHECK_F64_ARRAY("compact_heightfield.built.bounds", bounds_values, 6, fixture::f64(expected, "compact_heightfield.built.bounds"), 1e-5);
    check_field("compact_heightfield.built.cell_index", widen(compact_heightfield.cells, [](const CompactHeightfieldCell &cell) { return cell.index; }),
                expected);
    check_field("compact_heightfield.built.cell_count", widen(compact_heightfield.cells, [](const CompactHeightfieldCell &cell) { return cell.count; }),
                expected);
    check_field("compact_heightfield.built.span_y", widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.y; }),
                expected);
    check_field("compact_heightfield.built.span_h", widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.h; }),
                expected);
    check_field("compact_heightfield.built.span_con", widen(compact_heightfield.spans, [](const CompactHeightfieldSpan &span) { return span.con; }),
                expected);
    check_field("compact_heightfield.built.areas", widen(compact_heightfield.areas, [](uint8_t area) { return area; }), expected);

    compact_heightfield::erode_walkable_area(nav_test.options.walkable_radius, compact_heightfield);
    check_field("compact_heightfield.eroded.areas", widen(compact_heightfield.areas, [](uint8_t area) { return area; }), expected);

    compact_heightfield_regions::build_distance_field(compact_heightfield);
    check_field("compact_heightfield.distances", widen(compact_heightfield.distances, [](uint16_t distance) { return distance; }), expected);
    int32_t max_distance = compact_heightfield.max_distance;
    CHECK_I32_ARRAY("compact_heightfield.max_distance", &max_distance, 1, fixture::i32(expected, "compact_heightfield.max_distance"));

    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

TEST(watershed_regions_match_navcat_on_nav_test) {
    NavTestCompact nav_test = load_nav_test_compact();
    REQUIRE(nav_test.fixture.loaded);

    CompactHeightfield compact_heightfield = build_eroded_with_distances(nav_test);
    CHECK(compact_heightfield_regions::build_regions(nav_test.context, compact_heightfield, nav_test.options.border_size,
                                                     nav_test.options.min_region_area, nav_test.options.merge_region_area));
    check_regions("regions.watershed", compact_heightfield, nav_test.fixture);

    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

TEST(monotone_regions_match_navcat_on_nav_test) {
    NavTestCompact nav_test = load_nav_test_compact();
    REQUIRE(nav_test.fixture.loaded);

    CompactHeightfield compact_heightfield = build_eroded_with_distances(nav_test);
    CHECK(compact_heightfield_regions::build_regions_monotone(nav_test.context, compact_heightfield, nav_test.options.border_size,
                                                              nav_test.options.min_region_area, nav_test.options.merge_region_area));
    check_regions("regions.monotone", compact_heightfield, nav_test.fixture);

    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

TEST(layer_regions_match_navcat_on_nav_test) {
    NavTestCompact nav_test = load_nav_test_compact();
    REQUIRE(nav_test.fixture.loaded);

    CompactHeightfield compact_heightfield = build_eroded_with_distances(nav_test);
    CHECK(compact_heightfield_regions::build_layer_regions(nav_test.context, compact_heightfield, nav_test.options.border_size,
                                                           nav_test.options.min_region_area));
    check_regions("regions.layers", compact_heightfield, nav_test.fixture);

    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}
