#include "roam/generate/contour_set.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

static void check_contours(RegionMethod method, uint32_t build_flags, const char *prefix) {
    NavTest nav_test = load_nav_test();
    REQUIRE(nav_test.fixture.loaded);
    const fixture::Fixture &expected = nav_test.fixture;

    CompactHeightfield compact_heightfield = build_with_regions(nav_test, method);
    ContourSet contour_set = contour_set::build_contours(nav_test.context, compact_heightfield, nav_test.options.max_simplification_error,
                                                         nav_test.options.max_edge_length, build_flags);
    CHECK(contour_set.contours.length > 10);

    char name[256];
    const Box3 &bounds = contour_set.bounds;
    float bounds_values[6] = {bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z};
    snprintf(name, sizeof(name), "%s.bounds", prefix);
    CHECK_F64_ARRAY(name, bounds_values, 6, fixture::f64(expected, name), 1e-5);

    Array<int32_t> vertices = {};
    Array<int32_t> raw_vertices = {};
    for (const Contour &contour : contour_set.contours) {
        for (int i = 0; i < contour.vertex_count * 4; i++) vertices.push(contour.vertices[(uint32_t)i]);
        for (int i = 0; i < contour.raw_vertex_count * 4; i++) raw_vertices.push(contour.raw_vertices[(uint32_t)i]);
    }

    snprintf(name, sizeof(name), "%s.vertex_counts", prefix);
    CHECK_EXPECTED(name, widen(contour_set.contours, [](const Contour &contour) { return contour.vertex_count; }), expected);
    snprintf(name, sizeof(name), "%s.vertices", prefix);
    CHECK_EXPECTED(name, vertices, expected);
    snprintf(name, sizeof(name), "%s.raw_vertex_counts", prefix);
    CHECK_EXPECTED(name, widen(contour_set.contours, [](const Contour &contour) { return contour.raw_vertex_count; }), expected);
    snprintf(name, sizeof(name), "%s.raw_vertices", prefix);
    CHECK_EXPECTED(name, raw_vertices, expected);
    snprintf(name, sizeof(name), "%s.regions", prefix);
    CHECK_EXPECTED(name, widen(contour_set.contours, [](const Contour &contour) { return contour.reg; }), expected);
    snprintf(name, sizeof(name), "%s.areas", prefix);
    CHECK_EXPECTED(name, widen(contour_set.contours, [](const Contour &contour) { return contour.area; }), expected);

    contour_set::release(contour_set);
    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

TEST(watershed_contours_match_navcat_on_nav_test) { check_contours(REGION_METHOD_WATERSHED, CONTOUR_TESS_WALL_EDGES, "contours.watershed"); }

TEST(monotone_contours_match_navcat_on_nav_test) {
    check_contours(REGION_METHOD_MONOTONE, CONTOUR_TESS_WALL_EDGES | CONTOUR_TESS_AREA_EDGES, "contours.monotone");
}

TEST(layer_contours_match_navcat_on_nav_test) {
    check_contours(REGION_METHOD_LAYERS, CONTOUR_TESS_WALL_EDGES | CONTOUR_TESS_AREA_EDGES, "contours.layers");
}
