#include "roam/generate/poly_mesh.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

static void check_poly_mesh(RegionMethod method, int max_vertices_per_poly, const char *prefix) {
    NavTest nav_test = load_nav_test();
    REQUIRE(nav_test.fixture.loaded);
    const fixture::Fixture &expected = nav_test.fixture;

    CompactHeightfield compact_heightfield = build_with_regions(nav_test, method);
    ContourSet contour_set = build_contour_set(nav_test, compact_heightfield, method);
    PolyMesh poly_mesh = poly_mesh::build_poly_mesh(nav_test.context, contour_set, max_vertices_per_poly);
    CHECK(poly_mesh.poly_count > 10);

    char name[256];
    int32_t vertex_count = poly_mesh.vertex_count;
    int32_t poly_count = poly_mesh.poly_count;
    snprintf(name, sizeof(name), "%s.vertex_count", prefix);
    CHECK_I32_ARRAY(name, &vertex_count, 1, fixture::i32(expected, name));
    snprintf(name, sizeof(name), "%s.poly_count", prefix);
    CHECK_I32_ARRAY(name, &poly_count, 1, fixture::i32(expected, name));
    snprintf(name, sizeof(name), "%s.vertices", prefix);
    CHECK_EXPECTED(name, widen(poly_mesh.vertices, [](int value) { return value; }), expected);
    snprintf(name, sizeof(name), "%s.polys", prefix);
    CHECK_EXPECTED(name, widen(poly_mesh.polys, [](int value) { return value; }), expected);
    snprintf(name, sizeof(name), "%s.regions", prefix);
    CHECK_EXPECTED(name, widen(poly_mesh.regions, [](int value) { return value; }), expected);
    snprintf(name, sizeof(name), "%s.areas", prefix);
    CHECK_EXPECTED(name, widen(poly_mesh.areas, [](uint8_t value) { return value; }), expected);

    poly_mesh::release(poly_mesh);
    contour_set::release(contour_set);
    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}

TEST(watershed_poly_mesh_matches_navcat_on_nav_test) { check_poly_mesh(REGION_METHOD_WATERSHED, 5, "poly_mesh.watershed"); }

TEST(monotone_poly_mesh_matches_navcat_on_nav_test) { check_poly_mesh(REGION_METHOD_MONOTONE, 5, "poly_mesh.monotone"); }

TEST(layer_poly_mesh_matches_navcat_on_nav_test) { check_poly_mesh(REGION_METHOD_LAYERS, 5, "poly_mesh.layers"); }

TEST(triangle_poly_mesh_matches_navcat_on_nav_test) { check_poly_mesh(REGION_METHOD_WATERSHED, 3, "poly_mesh.triangles"); }
