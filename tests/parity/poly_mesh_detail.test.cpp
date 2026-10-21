#include "roam/generate/poly_mesh_detail.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/detail_ties.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

TEST_CASE("watershed poly mesh detail matches navcat on nav test") {
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
    int different_meshes = 0;
    for (uint32_t m = 0; m < (uint32_t)detail.mesh_count; m++) {
        DetailMeshRange actual = {detail.meshes[m * 4], detail.meshes[m * 4 + 1], detail.meshes[m * 4 + 2], detail.meshes[m * 4 + 3]};
        DetailMeshRange wanted = {meshes.data[m * 4], meshes.data[m * 4 + 1], meshes.data[m * 4 + 2], meshes.data[m * 4 + 3]};
        DetailMeshMatch match = compare_detail_mesh(actual, detail.vertices.data, detail.triangles.data, wanted, vertices.data, triangles.data);
        if (match == DETAIL_MESH_TIED) tied_meshes++;
        if (match == DETAIL_MESH_DIFFERENT) different_meshes++;
    }

    CHECK(different_meshes == 0);
    CHECK(tied_meshes <= detail.mesh_count / 50);

    poly_mesh_detail::release(detail);
    poly_mesh::release(poly_mesh);
    contour_set::release(contour_set);
    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}
