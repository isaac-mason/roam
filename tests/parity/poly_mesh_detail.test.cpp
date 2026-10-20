#include "roam/generate/poly_mesh_detail.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

static int rotate_edge_flags(int flags, int steps) {
    int rotated = flags;
    for (int i = 0; i < steps; i++) rotated = ((rotated >> 2) | ((rotated & 0x3) << 4)) & 0x3f;
    return rotated;
}

static bool same_triangle(const int *actual, const int32_t *expected) {
    for (int steps = 0; steps < 3; steps++) {
        bool vertices_match = actual[0] == expected[steps % 3] && actual[1] == expected[(steps + 1) % 3] && actual[2] == expected[(steps + 2) % 3];
        if (vertices_match && actual[3] == rotate_edge_flags(expected[3], steps)) return true;
    }
    return false;
}

TEST(watershed_poly_mesh_detail_matches_navcat_on_nav_test) {
    NavTest nav_test = load_nav_test();
    REQUIRE(nav_test.fixture.loaded);
    const fixture::Fixture &expected = nav_test.fixture;

    CompactHeightfield compact_heightfield = build_with_regions(nav_test, REGION_METHOD_WATERSHED);
    ContourSet contour_set = build_contour_set(nav_test, compact_heightfield, REGION_METHOD_WATERSHED);
    PolyMesh poly_mesh = poly_mesh::build_poly_mesh(nav_test.context, contour_set, nav_test.options.max_vertices_per_poly);
    PolyMeshDetail detail = poly_mesh_detail::build_poly_mesh_detail(nav_test.context, poly_mesh, compact_heightfield,
                                                                     nav_test.options.detail_sample_distance, nav_test.options.detail_sample_max_error);

    fixture::I32 meshes = fixture::i32(expected, "poly_mesh_detail.watershed.meshes");
    fixture::F64 vertices = fixture::f64(expected, "poly_mesh_detail.watershed.vertices");
    fixture::I32 triangles = fixture::i32(expected, "poly_mesh_detail.watershed.triangles");
    REQUIRE(meshes.count == detail.meshes.length);

    int tied_meshes = 0;
    for (uint32_t m = 0; m < (uint32_t)detail.mesh_count; m++) {
        int vertex_base = detail.meshes[m * 4];
        int vertex_count = detail.meshes[m * 4 + 1];
        int triangle_base = detail.meshes[m * 4 + 2];
        int triangle_count = detail.meshes[m * 4 + 3];
        int expected_vertex_base = meshes.data[m * 4];
        int expected_vertex_count = meshes.data[m * 4 + 1];
        int expected_triangle_base = meshes.data[m * 4 + 2];
        int expected_triangle_count = meshes.data[m * 4 + 3];

        if (vertex_count != expected_vertex_count) {
            tied_meshes++;
            CHECK(scalar::abs(vertex_count - expected_vertex_count) <= 1);
            continue;
        }

        CHECK(triangle_count == expected_triangle_count);
        if (triangle_count != expected_triangle_count) continue;

        for (int v = 0; v < vertex_count * 3; v++) {
            CHECK_NEAR(detail.vertices[(uint32_t)(vertex_base * 3 + v)], vertices.data[expected_vertex_base * 3 + v], 1e-4);
        }
        bool same_triangulation = true;
        for (int t = 0; t < triangle_count; t++) {
            same_triangulation = same_triangulation &&
                                 same_triangle(&detail.triangles[(uint32_t)((triangle_base + t) * 4)], &triangles.data[(expected_triangle_base + t) * 4]);
        }
        if (!same_triangulation) tied_meshes++;
    }

    CHECK(tied_meshes <= detail.mesh_count / 50);

    poly_mesh_detail::release(detail);
    poly_mesh::release(poly_mesh);
    contour_set::release(contour_set);
    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}
