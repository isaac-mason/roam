#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"
#include "tests/fuzz/fuzz.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

struct SavedTile {
    int x;
    int y;
    int layer;
    Array<uint8_t> bytes;
};

static Array<uint8_t> save_tile(const NavMeshTile &tile) {
    SaveResult size = {};
    nav_mesh_io::save_tile(size, tile, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_tile(saved, tile, bytes);
    return bytes;
}

static bool has_tile(const NavMesh &nav_mesh, const SavedTile &saved) {
    return nav_mesh_api::get_tile_at(nav_mesh, saved.x, saved.y, saved.layer) != nullptr;
}

static void add_saved_tile(NavMesh &nav_mesh, const SavedTile &saved) {
    NavMeshTile tile = {};
    REQUIRE(nav_mesh_io::load_tile(tile, saved.bytes) == LOAD_STATUS_OK);
    nav_mesh_api::add_tile(nav_mesh, tile);
}

static Vec3 random_point(test::Random &random) { return {test::next_float(random, 0, 24), 0, test::next_float(random, 0, 24)}; }

static void check_paths(test::Random &random, const NavMesh &nav_mesh, QueryWorkspace &workspace) {
    StraightPathPoint points[64];
    NodeRef node_path[128];
    for (int i = 0; i < 3; i++) {
        FindPathResult result = {};
        Vec3 start = random_point(random);
        Vec3 end = random_point(random);
        find_path::find_path(result, workspace, nav_mesh, start, end, {1, 2, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points,
                             {.raycast_distance = 0, .node_path = node_path});
        for (uint32_t k = 0; k < result.node_path_count; k++) CHECK(nav_mesh_api::is_valid_node_ref(nav_mesh, node_path[k]));
    }
}

static void check_reachability(test::Random &random, const NavMesh &nav_mesh, QueryWorkspace &workspace, uint32_t &reachable_pairs,
                               uint32_t &unreachable_pairs) {
    Array<NodeRef> refs = {};
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (node.allocated) refs.push(node.ref);
    }
    for (uint32_t i = 0; i < 20 && refs.length > 0; i++) {
        NodeRef from = refs[test::next_below(random, refs.length)];
        NodeRef to = refs[test::next_below(random, refs.length)];
        bool expected = reachable_by_links(nav_mesh, from, to);
        CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, from, to) == expected);
        (expected ? reachable_pairs : unreachable_pairs)++;
    }
    refs.release();
}

TEST_CASE("random tile and off mesh churn keeps the nav mesh consistent") {
    test::Random random = {0xc0ffeeull};
    NavMesh nav_mesh = create_tiled_terrain();
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, 0);
    Array<LinkKey> original_links = link_keys(nav_mesh);

    Array<SavedTile> saved = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (tile.allocated) saved.push({tile.tile_x, tile.tile_y, tile.tile_layer, save_tile(tile)});
    }
    REQUIRE(saved.length > 2);

    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    Array<uint32_t> connections = {};
    uint32_t fewest_tiles = saved.length;
    uint32_t attached_connections = 0;
    uint32_t reachable_pairs = 0;
    uint32_t unreachable_pairs = 0;

    for (uint32_t step = 0; step < 300; step++) {
        const SavedTile &tile = saved[test::next_below(random, saved.length)];
        uint32_t operation = test::next_below(random, 6);
        INFO("step ", step, " operation ", operation, " tile ", tile.x, ",", tile.y);
        switch (operation) {
            case 0: nav_mesh_api::remove_tile(nav_mesh, tile.x, tile.y, tile.layer); break;
            case 1:
                if (!has_tile(nav_mesh, tile)) add_saved_tile(nav_mesh, tile);
                break;
            case 2: add_saved_tile(nav_mesh, tile); break;
            case 3: {
                OffMeshConnectionDirection direction =
                    test::one_in(random, 2) ? OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL : OFF_MESH_CONNECTION_DIRECTION_START_TO_END;
                connections.push(nav_mesh_api::add_off_mesh_connection(nav_mesh, {random_point(random), random_point(random), 0.6f, direction, 1, 1}));
                break;
            }
            case 5: {
                Vec3 start = random_point(random);
                Vec3 end = random_point(random);
                Vec3 along = {test::next_float(random, -6, 6), 0, test::next_float(random, -6, 6)};
                OffMeshConnectionDirection direction =
                    test::one_in(random, 2) ? OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL : OFF_MESH_CONNECTION_DIRECTION_START_TO_END;
                connections.push(nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, {
                                                                                             .start_a = start,
                                                                                             .start_b = vec3::add(start, along),
                                                                                             .end_a = end,
                                                                                             .end_b = vec3::add(end, along),
                                                                                             .radius = 0.6f,
                                                                                             .inset = 0.1f,
                                                                                             .direction = direction,
                                                                                             .flags = 1,
                                                                                             .area = 1,
                                                                                             .align = false,
                                                                                         }));
                break;
            }
            default:
                if (connections.length > 0) {
                    uint32_t index = test::next_below(random, connections.length);
                    nav_mesh_api::remove_off_mesh_connection(nav_mesh, connections[index]);
                    connections.remove_swap(index);
                }
                break;
        }
        REQUIRE(snapshot_is_valid(nav_mesh));
        REQUIRE(clusters_are_consistent(nav_mesh));
        REQUIRE(segment_pieces_are_consistent(nav_mesh));
        check_paths(random, nav_mesh, workspace);
        check_reachability(random, nav_mesh, workspace, reachable_pairs, unreachable_pairs);

        uint32_t tile_count = 0;
        for (const SavedTile &candidate : saved) tile_count += has_tile(nav_mesh, candidate) ? 1 : 0;
        if (tile_count < fewest_tiles) fewest_tiles = tile_count;
        for (uint32_t id : connections) attached_connections += nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id) ? 1 : 0;
    }
    CHECK(fewest_tiles < saved.length / 2);
    CHECK(attached_connections > 0);
    CHECK(reachable_pairs > 500);
    CHECK(unreachable_pairs > 50);

    for (uint32_t id : connections) nav_mesh_api::remove_off_mesh_connection(nav_mesh, id);
    for (const SavedTile &tile : saved) {
        if (!has_tile(nav_mesh, tile)) add_saved_tile(nav_mesh, tile);
    }
    REQUIRE(snapshot_is_valid(nav_mesh));
    Array<LinkKey> final_links = link_keys(nav_mesh);
    CHECK(same_links(final_links, original_links));

    final_links.release();
    original_links.release();
    connections.release();
    for (SavedTile &tile : saved) tile.bytes.release();
    saved.release();
    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}
