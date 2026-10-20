#include "roam/generate/nav_mesh_tile.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"

using namespace roam;

static PolyMesh build_watershed_poly_mesh(NavTest &nav_test, CompactHeightfield &out_compact_heightfield) {
    out_compact_heightfield = build_with_regions(nav_test, REGION_METHOD_WATERSHED);
    ContourSet contour_set = build_contour_set(nav_test, out_compact_heightfield, REGION_METHOD_WATERSHED);
    PolyMesh poly_mesh = poly_mesh::build_poly_mesh(nav_test.context, contour_set, nav_test.options.max_vertices_per_poly);
    contour_set::release(contour_set);
    return poly_mesh;
}

static PolyMeshDetail load_navcat_detail(const fixture::Fixture &expected) {
    PolyMeshDetail detail = {};
    fixture::I32 meshes = fixture::i32(expected, "poly_mesh_detail.watershed.meshes");
    fixture::F64 vertices = fixture::f64(expected, "poly_mesh_detail.watershed.vertices");
    fixture::I32 triangles = fixture::i32(expected, "poly_mesh_detail.watershed.triangles");
    for (uint32_t i = 0; i < meshes.count; i++) detail.meshes.push(meshes.data[i]);
    for (uint32_t i = 0; i < vertices.count; i++) detail.vertices.push((float)vertices.data[i]);
    for (uint32_t i = 0; i < triangles.count; i++) detail.triangles.push(triangles.data[i]);
    detail.mesh_count = (int)meshes.count / 4;
    detail.vertex_count = (int)vertices.count / 3;
    detail.triangle_count = (int)triangles.count / 4;
    return detail;
}

static void check_tile_detail(const char *prefix, const NavMeshTileDetailMesh &detail_mesh, const fixture::Fixture &expected) {
    char name[256];
    Array<int32_t> meshes = {};
    for (const NavMeshPolyDetail &mesh : detail_mesh.detail_meshes) {
        meshes.push((int32_t)mesh.vertices_base);
        meshes.push((int32_t)mesh.triangles_base);
        meshes.push((int32_t)mesh.vertices_count);
        meshes.push((int32_t)mesh.triangles_count);
    }
    snprintf(name, sizeof(name), "%s.detail_meshes", prefix);
    CHECK_EXPECTED(name, meshes, expected);
    snprintf(name, sizeof(name), "%s.detail_vertices", prefix);
    CHECK_F64_ARRAY(name, detail_mesh.detail_vertices.data, detail_mesh.detail_vertices.length, fixture::f64(expected, name), 1e-5);
    snprintf(name, sizeof(name), "%s.detail_triangles", prefix);
    CHECK_EXPECTED(name, widen(detail_mesh.detail_triangles, [](int value) { return value; }), expected);
}

TEST(tile_polys_match_navcat_on_nav_test) {
    NavTest nav_test = load_nav_test();
    REQUIRE(nav_test.fixture.loaded);
    const fixture::Fixture &expected = nav_test.fixture;

    CompactHeightfield compact_heightfield = {};
    PolyMesh poly_mesh = build_watershed_poly_mesh(nav_test, compact_heightfield);
    NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(poly_mesh);


    CHECK_F64_ARRAY("tile.watershed.vertices", tile_polys.vertices.data, tile_polys.vertices.length, fixture::f64(expected, "tile.watershed.vertices"),
                    1e-4);
    CHECK_EXPECTED("tile.watershed.poly_vertex_counts", widen(tile_polys.polys, [](const NavMeshPoly &poly) { return poly.vertex_count; }),
                   expected);
    CHECK_EXPECTED("tile.watershed.poly_vertices", widen(tile_polys.poly_vertices, [](uint32_t value) { return value; }), expected);
    CHECK_EXPECTED("tile.watershed.poly_neis", widen(tile_polys.poly_neis, [](int value) { return value; }), expected);
    uint32_t expected_base = 0;
    for (const NavMeshPoly &poly : tile_polys.polys) {
        CHECK(poly.vertex_base == expected_base);
        expected_base += poly.vertex_count;
    }
    CHECK_EXPECTED("tile.watershed.poly_flags", widen(tile_polys.polys, [](const NavMeshPoly &poly) { return poly.flags; }), expected);
    CHECK_EXPECTED("tile.watershed.poly_areas", widen(tile_polys.polys, [](const NavMeshPoly &poly) { return poly.area; }), expected);

    PolyMeshDetail navcat_detail = load_navcat_detail(expected);
    NavMeshTileDetailMesh detail_mesh = nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, navcat_detail);
    check_tile_detail("tile.watershed", detail_mesh, expected);

    NavMeshTileDetailMesh fan_mesh = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);
    check_tile_detail("tile.watershed.fan", fan_mesh, expected);

    nav_mesh_tile::release(fan_mesh);
    nav_mesh_tile::release(detail_mesh);
    poly_mesh_detail::release(navcat_detail);
    nav_mesh_tile::release(tile_polys);
    poly_mesh::release(poly_mesh);
    compact_heightfield::release(compact_heightfield);
    release(nav_test);
}
