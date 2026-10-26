#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_io.h"
#include "roam/query/nav_mesh_search.h"
#include "tests/fuzz/fuzz.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

static NavMesh create_two_islands() {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 5, 0, 0, 7, 0, 0, 7, 0, 2, 5, 0, 2};
    const uint32_t indices[] = {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6};
    return create_nav_mesh_from_triangles(positions, 8, indices, 12);
}

static NodeRef nearest_poly(const NavMesh &nav_mesh, Vec3 position) {
    FindNearestPolyResult result = {};
    nav_mesh_api::find_nearest_poly(result, nav_mesh, position, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    REQUIRE(result.success);
    return result.node_ref;
}

static Array<NodeRef> poly_refs(const NavMesh &nav_mesh) {
    Array<NodeRef> refs = {};
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (node.allocated) refs.push(node.ref);
    }
    return refs;
}

static uint32_t count_mismatches(QueryWorkspace &workspace, const NavMesh &nav_mesh, test::Random &random, uint32_t pairs) {
    Array<NodeRef> refs = poly_refs(nav_mesh);
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < pairs && refs.length > 0; i++) {
        NodeRef from = refs[test::next_below(random, refs.length)];
        NodeRef to = refs[test::next_below(random, refs.length)];
        if (nav_mesh_search::is_reachable(workspace, nav_mesh, from, to) != reachable_by_links(nav_mesh, from, to)) mismatches++;
    }
    refs.release();
    return mismatches;
}

TEST_CASE("disconnected parts of a tile are separate clusters") {
    NavMesh nav_mesh = create_two_islands();
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    NodeRef a = nearest_poly(nav_mesh, {1, 0, 1});
    NodeRef a_other = nearest_poly(nav_mesh, {0.2f, 0, 1.8f});
    NodeRef b = nearest_poly(nav_mesh, {6, 0, 1});

    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(nav_mesh.tiles[0].clusters.length == 2);
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, a, a_other));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, a, a));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, a, b));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, b, a));

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("one way off mesh connections are reachable in one direction only") {
    NavMesh nav_mesh = create_two_islands();
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    NodeRef a = nearest_poly(nav_mesh, {1, 0, 1});
    NodeRef b = nearest_poly(nav_mesh, {6, 0, 1});

    uint32_t id = nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1, 0, 1}, {6, 0, 1}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});
    NodeRef link = first_piece(nav_mesh, id).off_mesh_node;
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, a, b));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, b, a));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, a, link));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, link, b));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, b, link));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, link, a));

    nav_mesh_api::remove_off_mesh_connection(nav_mesh, id);
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, a, b));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, a, link));

    id = nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1, 0, 1}, {6, 0, 1}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 0});
    link = first_piece(nav_mesh, id).off_mesh_node;
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, a, b));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, b, a));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, link, a));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, b, link));

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("invalid and stale refs are not reachable") {
    NavMesh nav_mesh = create_tiled_terrain();
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    NodeRef a = nearest_poly(nav_mesh, {3, 0, 3});
    NodeRef b = nearest_poly(nav_mesh, {20, 0, 20});
    REQUIRE(nav_mesh_search::is_reachable(workspace, nav_mesh, a, b));

    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, a, INVALID_NODE_REF));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, INVALID_NODE_REF, b));

    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, b);
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
    int x = tile.tile_x;
    int y = tile.tile_y;
    int layer = tile.tile_layer;
    nav_mesh_api::remove_tile(nav_mesh, x, y, layer);
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, a, b));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, b, a));
    CHECK(clusters_are_consistent(nav_mesh));

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("removing a column of tiles splits the terrain and adding it back joins it") {
    NavMesh nav_mesh = create_tiled_terrain();
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, 0);
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    test::Random random = {0x51ull};
    NodeRef left = nearest_poly(nav_mesh, {2, 0, 12});
    NodeRef right = nearest_poly(nav_mesh, {22, 0, 12});
    REQUIRE(clusters_are_consistent(nav_mesh));
    REQUIRE(nav_mesh_search::is_reachable(workspace, nav_mesh, left, right));
    int column = nav_mesh_api::get_node_by_ref(nav_mesh, nearest_poly(nav_mesh, {12, 0, 2})).tile_id;
    int column_x = nav_mesh.tiles[(uint32_t)column].tile_x;

    struct Removed {
        int y;
        int layer;
        Array<uint8_t> bytes;
    };
    Array<Removed> removed = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated || tile.tile_x != column_x) continue;
        SaveResult size = {};
        nav_mesh_io::save_tile(size, tile, {});
        Removed entry = {tile.tile_y, tile.tile_layer, {}};
        entry.bytes.resize(size.byte_total);
        SaveResult saved = {};
        nav_mesh_io::save_tile(saved, tile, entry.bytes);
        removed.push(entry);
    }
    REQUIRE(removed.length > 0);
    for (const Removed &entry : removed) nav_mesh_api::remove_tile(nav_mesh, column_x, entry.y, entry.layer);

    CHECK(clusters_are_consistent(nav_mesh));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, left, right));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, right, left));
    CHECK(count_mismatches(workspace, nav_mesh, random, 400) == 0);

    for (Removed &entry : removed) {
        NavMeshTile tile = {};
        REQUIRE(nav_mesh_io::load_tile(tile, entry.bytes) == LOAD_STATUS_OK);
        nav_mesh_api::add_tile(nav_mesh, tile);
        entry.bytes.release();
    }
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, left, right));
    CHECK(count_mismatches(workspace, nav_mesh, random, 400) == 0);

    removed.release();
    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("loading a nav mesh rebuilds its clusters") {
    NavMesh nav_mesh = create_tiled_terrain();
    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{3, 0, 20}, {20, 0, 3}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();

    SaveResult size = {};
    nav_mesh_io::save_nav_mesh(size, nav_mesh, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);
    NavMesh loaded = {};
    REQUIRE(nav_mesh_io::load_nav_mesh(loaded, bytes) == LOAD_STATUS_OK);

    CHECK(clusters_are_consistent(loaded));
    Array<NodeRef> refs = poly_refs(nav_mesh);
    test::Random random = {0x10adull};
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < 400; i++) {
        NodeRef from = refs[test::next_below(random, refs.length)];
        NodeRef to = refs[test::next_below(random, refs.length)];
        if (nav_mesh_search::is_reachable(workspace, nav_mesh, from, to) != nav_mesh_search::is_reachable(workspace, loaded, from, to)) mismatches++;
    }
    CHECK(mismatches == 0);

    refs.release();
    bytes.release();
    nav_mesh_search::release(workspace);
    nav_mesh_api::release(loaded);
    nav_mesh_api::release(nav_mesh);
}
