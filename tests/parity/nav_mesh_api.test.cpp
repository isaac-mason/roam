#include <float.h>
#include <math.h>

#include "roam/generate/common.h"
#include "roam/query/nav_mesh_api.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"
#include "tests/parity/nav_test_nav_mesh.h"

using namespace roam;

TEST(nav_mesh_matches_navcat_on_nav_test) {
    fixture::Fixture expected = fixture::load("nav_test_solo");
    REQUIRE(expected.loaded);

    NavMeshTileParams params = load_tile_params(expected);
    NavMeshTile tile = nav_mesh_api::build_tile(params);

    Array<int32_t> bv_bounds = {};
    for (const NavMeshBvNode &node : tile.bv_tree.nodes) {
        for (int axis = 0; axis < 3; axis++) bv_bounds.push(node.bmin[axis]);
        for (int axis = 0; axis < 3; axis++) bv_bounds.push(node.bmax[axis]);
    }
    CHECK_EXPECTED("nav_mesh.bv_bounds", bv_bounds, expected);
    CHECK_EXPECTED("nav_mesh.bv_i", widen(tile.bv_tree.nodes, [](const NavMeshBvNode &node) { return node.i; }), expected);

    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = params.bounds.min;
    nav_mesh.tile_width = params.bounds.max.x - params.bounds.min.x;
    nav_mesh.tile_height = params.bounds.max.z - params.bounds.min.z;
    nav_mesh_api::add_tile(nav_mesh, tile);

    check_nav_mesh("nav_mesh.built", nav_mesh, expected);

    fixture::F64 points = fixture::f64(expected, "queries.points");
    fixture::F64 half_extents_values = fixture::f64(expected, "queries.half_extents");
    fixture::I32 nearest_success = fixture::i32(expected, "queries.nearest_success");
    fixture::F64 nearest_refs = fixture::f64(expected, "queries.nearest_refs");
    fixture::F64 nearest_positions = fixture::f64(expected, "queries.nearest_positions");
    fixture::I32 polygon_counts = fixture::i32(expected, "queries.polygon_counts");
    fixture::F64 polygon_refs = fixture::f64(expected, "queries.polygon_refs");
    Vec3 half_extents = {(float)half_extents_values.data[0], (float)half_extents_values.data[1], (float)half_extents_values.data[2]};

    uint32_t polygon_cursor = 0;
    uint32_t nearest_mismatches = 0;
    uint32_t polygon_mismatches = 0;
    for (uint32_t i = 0; i < nearest_success.count; i++) {
        Vec3 point = {(float)points.data[i * 3], (float)points.data[i * 3 + 1], (float)points.data[i * 3 + 2]};

        FindNearestPolyResult nearest = {};
        nav_mesh_api::find_nearest_poly(nearest, nav_mesh, point, half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER);
        bool same = nearest.success == (nearest_success.data[i] != 0);
        if (same && nearest.success) {
            same = nearest.node_ref == ref_from_f64(nearest_refs.data[i]) && fabs(nearest.position.x - nearest_positions.data[i * 3]) < 1e-3 &&
                   fabs(nearest.position.y - nearest_positions.data[i * 3 + 1]) < 1e-3 && fabs(nearest.position.z - nearest_positions.data[i * 3 + 2]) < 1e-3;
        }
        if (!same) nearest_mismatches++;

        NodeRef refs[256];
        QueryPolygonsResult polygons;
        nav_mesh_api::query_polygons(polygons, nav_mesh, {vec3::subtract(point, half_extents), vec3::add(point, half_extents)},
                                     nav_mesh_api::DEFAULT_QUERY_FILTER, refs);
        bool same_polygons = polygons.poly_total == polygons.poly_count && polygons.poly_count == (uint32_t)polygon_counts.data[i];
        for (uint32_t k = 0; same_polygons && k < polygons.poly_count; k++) same_polygons = refs[k] == ref_from_f64(polygon_refs.data[polygon_cursor + k]);
        if (!same_polygons) polygon_mismatches++;
        polygon_cursor += (uint32_t)polygon_counts.data[i];
    }
    CHECK(nearest_mismatches == 0);
    CHECK(polygon_mismatches == 0);

    uint32_t pruning_mismatches = 0;
    uint32_t seed = 12345;
    for (uint32_t i = 0; i < 400; i++) {
        float u[3];
        for (float &value : u) {
            seed = seed * 1664525u + 1013904223u;
            value = (float)(seed >> 8) / (float)(1u << 24);
        }
        Vec3 point = {params.bounds.min.x + (params.bounds.max.x - params.bounds.min.x) * u[0],
                      params.bounds.min.y + (params.bounds.max.y - params.bounds.min.y) * u[1],
                      params.bounds.min.z + (params.bounds.max.z - params.bounds.min.z) * u[2]};
        Vec3 half_extents = {3, 4, 3};

        FindNearestPolyResult pruned;
        nav_mesh_api::find_nearest_poly(pruned, nav_mesh, point, half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER);

        NodeRef candidates[1024];
        QueryPolygonsResult polygons;
        nav_mesh_api::query_polygons(polygons, nav_mesh, {vec3::subtract(point, half_extents), vec3::add(point, half_extents)},
                                     nav_mesh_api::DEFAULT_QUERY_FILTER, candidates);
        FindNearestPolyResult brute = {false, INVALID_NODE_REF, point};
        float best = FLT_MAX;
        for (uint32_t k = 0; k < polygons.poly_count; k++) {
            GetClosestPointOnPolyResult closest;
            nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, candidates[k], point);
            Vec3 difference = vec3::subtract(point, closest.position);
            float height_difference = fabsf(difference.y) - nav_mesh.tiles[0].walkable_climb;
            float distance_sqr = closest.is_over_poly ? (height_difference > 0 ? height_difference * height_difference : 0) : vec3::squared_length(difference);
            if (distance_sqr < best) {
                best = distance_sqr;
                brute = {true, candidates[k], closest.position};
            }
        }
        bool same = pruned.success == brute.success && pruned.node_ref == brute.node_ref && vec3::exactly_equals(pruned.position, brute.position);
        if (!same) pruning_mismatches++;
    }
    CHECK(pruning_mismatches == 0);

    fixture::F64 start = fixture::f64(expected, "off_mesh.start");
    fixture::F64 end = fixture::f64(expected, "off_mesh.end");
    fixture::F64 attachment = fixture::f64(expected, "off_mesh.attachment");
    uint32_t id = nav_mesh_api::add_off_mesh_connection(nav_mesh, {{(float)start.data[0], (float)start.data[1], (float)start.data[2]},
                                                                   {(float)end.data[0], (float)end.data[1], (float)end.data[2]},
                                                                   0.5f,
                                                                   OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
                                                                   1,
                                                                   0});
    CHECK(nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id) == (fixture::i32(expected, "off_mesh.connected").data[0] != 0));
    const OffMeshConnectionAttachment &actual_attachment = nav_mesh.off_mesh_connection_attachments[id];
    CHECK(actual_attachment.off_mesh_node == ref_from_f64(attachment.data[0]));
    CHECK(actual_attachment.start_poly_node == ref_from_f64(attachment.data[1]));
    CHECK(actual_attachment.end_poly_node == ref_from_f64(attachment.data[2]));

    check_nav_mesh("nav_mesh.with_off_mesh", nav_mesh, expected);

    nav_mesh_api::release_nav_mesh(nav_mesh);
    release(params);
    fixture::release(expected);
}
