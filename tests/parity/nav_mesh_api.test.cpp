#include "roam/generate/common.h"
#include "roam/query/nav_mesh_api.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"
#include "tests/parity/nav_test_pipeline.h"
#include "tests/parity/nav_test_nav_mesh.h"

using namespace roam;

static void check_nav_mesh(const char *prefix, const NavMesh &nav_mesh, const fixture::Fixture &expected) {
    char name[256];
    snprintf(name, sizeof(name), "%s.node_refs", prefix);
    fixture::F64 node_refs = fixture::f64(expected, name);
    snprintf(name, sizeof(name), "%s.node_allocated", prefix);
    fixture::I32 node_allocated = fixture::i32(expected, name);
    snprintf(name, sizeof(name), "%s.link_counts", prefix);
    fixture::I32 link_counts = fixture::i32(expected, name);
    snprintf(name, sizeof(name), "%s.link_to_refs", prefix);
    fixture::F64 link_to_refs = fixture::f64(expected, name);
    snprintf(name, sizeof(name), "%s.link_fields", prefix);
    fixture::I32 link_fields = fixture::i32(expected, name);

    REQUIRE(nav_mesh.nodes.length == node_refs.count);

    uint32_t link_cursor = 0;
    uint32_t mismatched_nodes = 0;
    for (uint32_t i = 0; i < nav_mesh.nodes.length; i++) {
        const NavMeshNode &node = nav_mesh.nodes[i];
        bool same = node.allocated == (node_allocated.data[i] != 0) && (!node.allocated || node.ref == ref_from_f64(node_refs.data[i]));

        int32_t count = 0;
        for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
            const NavMeshLink &navmesh_link = nav_mesh.links[(uint32_t)link];
            uint32_t at = link_cursor + (uint32_t)count;
            if (count < link_counts.data[i]) {
                same = same && navmesh_link.to_node_ref == ref_from_f64(link_to_refs.data[at]) && navmesh_link.edge == link_fields.data[at * 4] &&
                       navmesh_link.side == link_fields.data[at * 4 + 1] && navmesh_link.bmin == link_fields.data[at * 4 + 2] &&
                       navmesh_link.bmax == link_fields.data[at * 4 + 3];
            }
            count++;
        }
        same = same && count == link_counts.data[i];
        link_cursor += (uint32_t)link_counts.data[i];
        if (!same) mismatched_nodes++;
    }
    CHECK(mismatched_nodes == 0);
}

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

        FindNearestPolyResult nearest = nav_mesh_api::find_nearest_poly(nav_mesh, point, half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER);
        bool same = nearest.success == (nearest_success.data[i] != 0);
        if (same && nearest.success) {
            same = nearest.node_ref == ref_from_f64(nearest_refs.data[i]) && fabs(nearest.position.x - nearest_positions.data[i * 3]) < 1e-3 &&
                   fabs(nearest.position.y - nearest_positions.data[i * 3 + 1]) < 1e-3 && fabs(nearest.position.z - nearest_positions.data[i * 3 + 2]) < 1e-3;
        }
        if (!same) nearest_mismatches++;

        Array<NodeRef> refs =
            nav_mesh_api::query_polygons(nav_mesh, {vec3::subtract(point, half_extents), vec3::add(point, half_extents)}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        bool same_polygons = refs.length == (uint32_t)polygon_counts.data[i];
        for (uint32_t k = 0; same_polygons && k < refs.length; k++) same_polygons = refs[k] == ref_from_f64(polygon_refs.data[polygon_cursor + k]);
        if (!same_polygons) polygon_mismatches++;
        polygon_cursor += (uint32_t)polygon_counts.data[i];
        refs.release();
    }
    CHECK(nearest_mismatches == 0);
    CHECK(polygon_mismatches == 0);

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
