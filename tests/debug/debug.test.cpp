#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"
#include "tests/harness/test.h"

using namespace roam;

static bool well_formed(const DebugPrimitives &primitives) {
    if (primitives.length == 0) return false;
    for (const DebugPrimitive &primitive : primitives) {
        if (primitive.positions.length == 0 || primitive.positions.length % 3 != 0) return false;
        if (primitive.colors.length != primitive.positions.length) return false;
        if (primitive.type == DEBUG_PRIMITIVE_TRIANGLES) {
            if (primitive.indices.length % 3 != 0) return false;
            for (uint32_t index : primitive.indices) {
                if (index >= primitive.positions.length / 3) return false;
            }
        }
        if (primitive.type == DEBUG_PRIMITIVE_LINES && primitive.positions.length % 6 != 0) return false;
        if (primitive.type == DEBUG_PRIMITIVE_BOXES && primitive.scales.length != primitive.positions.length) return false;
    }
    return true;
}

static void check_helper(DebugPrimitives primitives, const char *name) {
    if (!well_formed(primitives)) test::fail(__FILE__, __LINE__, name);
    debug::release(primitives);
}

TEST_CASE("debug helpers draw a generated nav mesh") {
    const float positions[] = {0, 0, 0, 10, 0, 0, 10, 0, 10, 0, 0, 10, 4, 0, 4, 6, 0, 4, 6, 2, 6, 4, 2, 6};
    const uint32_t indices[] = {0, 2, 1, 0, 3, 2, 4, 7, 6, 4, 6, 5};

    SoloNavMeshOptions options = {
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .walkable_radius_voxels = 2,
        .walkable_radius_world = 0.4f,
        .walkable_climb_voxels = 2,
        .walkable_climb_world = 0.4f,
        .walkable_height_voxels = 10,
        .walkable_height_world = 2,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 1.2f,
        .detail_sample_max_error = 0.2f,
    };

    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh({positions, indices}, options);
    const SoloNavMeshIntermediates &intermediates = result.intermediates;
    NavMesh &nav_mesh = result.nav_mesh;

    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1, 0, 1}, {9, 0, 9}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});

    check_helper(debug::create_triangle_area_ids_helper({positions, indices}, intermediates.tri_area_ids), "triangle area ids");
    check_helper(debug::create_heightfield_helper(intermediates.heightfield), "heightfield");
    check_helper(debug::create_compact_heightfield_solid_helper(intermediates.compact_heightfield), "compact heightfield solid");
    check_helper(debug::create_compact_heightfield_distances_helper(intermediates.compact_heightfield), "compact heightfield distances");
    check_helper(debug::create_compact_heightfield_regions_helper(intermediates.compact_heightfield), "compact heightfield regions");
    check_helper(debug::create_raw_contours_helper(intermediates.contour_set), "raw contours");
    check_helper(debug::create_simplified_contours_helper(intermediates.contour_set), "simplified contours");
    check_helper(debug::create_poly_mesh_helper(intermediates.poly_mesh), "poly mesh");
    check_helper(debug::create_poly_mesh_detail_helper(intermediates.poly_mesh_detail), "poly mesh detail");
    check_helper(debug::create_nav_mesh_helper(nav_mesh), "nav mesh");
    check_helper(debug::create_nav_mesh_islands_helper(nav_mesh), "nav mesh islands");
    check_helper(debug::create_nav_mesh_tile_helper(nav_mesh.tiles[0]), "nav mesh tile");
    check_helper(debug::create_nav_mesh_bv_tree_helper(nav_mesh), "bv tree");
    check_helper(debug::create_nav_mesh_links_helper(nav_mesh), "links");
    check_helper(debug::create_nav_mesh_off_mesh_connections_helper(nav_mesh), "off mesh connections");

    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, {1, 0, 1}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    REQUIRE(nearest.success);
    check_helper(debug::create_nav_mesh_poly_helper(nav_mesh, nearest.node_ref), "poly");

    FindNearestPolyResult far = {};
    nav_mesh_api::find_nearest_poly(far, nav_mesh, {9, 0, 1}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    REQUIRE(far.success);
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    FindNodePathResult node_path;
    nav_mesh_search::find_node_path(node_path, workspace, nav_mesh, nearest.node_ref, far.node_ref, {1, 0, 1}, {9, 0, 1},
                                    nav_mesh_api::DEFAULT_QUERY_FILTER, {});
    REQUIRE(node_path.success);
    check_helper(debug::create_search_nodes_helper(workspace.nodes), "search nodes");
    nav_mesh_search::release(workspace);

    generate_solo_nav_mesh::release(result.intermediates);
    nav_mesh_api::release(nav_mesh);
}
