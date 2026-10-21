#include <math.h>
#include <string.h>

#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_io.h"
#include "roam/query/nav_mesh_search.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

using namespace roam;

static Array<uint8_t> save_tile_bytes(const NavMeshTile &tile) {
    SaveResult size = {};
    nav_mesh_io::save_tile(size, tile, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_tile(saved, tile, bytes);
    CHECK(saved.success);
    return bytes;
}

static Array<uint8_t> save_nav_mesh_bytes(const NavMesh &nav_mesh) {
    SaveResult size = {};
    nav_mesh_io::save_nav_mesh(size, nav_mesh, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);
    CHECK(saved.success);
    return bytes;
}

static bool same_bytes(const void *a, uint64_t a_size, const void *b, uint64_t b_size) {
    return a_size == b_size && (a_size == 0 || memcmp(a, b, a_size) == 0);
}

#define SAME_ARRAY(a, b) same_bytes((a).data, (uint64_t)(a).length * sizeof(*(a).data), (b).data, (uint64_t)(b).length * sizeof(*(b).data))

static bool same_polys(const Array<NavMeshPoly> &a, const Array<NavMeshPoly> &b) {
    if (a.length != b.length) return false;
    for (uint32_t i = 0; i < a.length; i++) {
        if (a[i].vertex_base != b[i].vertex_base || a[i].vertex_count != b[i].vertex_count || a[i].flags != b[i].flags || a[i].area != b[i].area) {
            return false;
        }
    }
    return true;
}

static bool same_bv_nodes(const Array<NavMeshBvNode> &a, const Array<NavMeshBvNode> &b) {
    if (a.length != b.length) return false;
    for (uint32_t i = 0; i < a.length; i++) {
        for (int k = 0; k < 3; k++) {
            if (a[i].bmin[k] != b[i].bmin[k] || a[i].bmax[k] != b[i].bmax[k]) return false;
        }
        if (a[i].i != b[i].i) return false;
    }
    return true;
}

static uint64_t hash_links(const NavMesh &nav_mesh) {
    uint64_t hash = 1469598103934665603ull;
    auto mix = [&](uint64_t value) { hash = (hash ^ value) * 1099511628211ull; };
    for (uint32_t i = 0; i < nav_mesh.nodes.length; i++) {
        const NavMeshNode &node = nav_mesh.nodes[i];
        if (!node.allocated) continue;
        mix(node.ref);
        for (int32_t l = node.first_link; l != NULL_LINK; l = nav_mesh.links[(uint32_t)l].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)l];
            mix(link.to_node_ref);
            mix(((uint64_t)link.edge << 24) | ((uint64_t)link.side << 16) | ((uint64_t)link.bmin << 8) | link.bmax);
        }
    }
    return hash;
}

static const Vec3 PATH_POINTS[] = {{1, 0, 1}, {23, 0, 23}, {5, 0, 20}, {20, 0, 3}, {11, 0, 4}, {11, 0, 18}, {2, 0, 12}};

static uint64_t hash_paths(const NavMesh &nav_mesh) {
    uint64_t hash = 1469598103934665603ull;
    auto mix = [&](uint64_t value) { hash = (hash ^ value) * 1099511628211ull; };
    SearchWorkspace workspace = nav_mesh_search::create_search_workspace();
    StraightPathPoint points[128];
    NodeRef node_path[256];
    uint32_t count = sizeof(PATH_POINTS) / sizeof(PATH_POINTS[0]);
    for (uint32_t a = 0; a < count; a++) {
        for (uint32_t b = 0; b < count; b++) {
            FindPathResult result = {};
            find_path::find_path(result, workspace, nav_mesh, PATH_POINTS[a], PATH_POINTS[b], {1, 2, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points,
                                 {.raycast_distance = 0, .node_path = node_path});
            mix(result.flags);
            for (uint32_t i = 0; i < result.node_path_count; i++) mix(node_path[i]);
            for (uint32_t i = 0; i < result.point_count; i++) {
                uint32_t bits[3];
                memcpy(bits, &points[i].position, sizeof(bits));
                mix(bits[0]);
                mix(bits[1]);
                mix(bits[2]);
            }
        }
    }
    nav_mesh_search::release(workspace);
    return hash;
}

TEST_CASE("a saved tile loads back with the same data") {
    NavMesh nav_mesh = create_tiled_terrain();
    uint32_t tile_count = 0;
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        tile_count++;
        Array<uint8_t> bytes = save_tile_bytes(tile);
        NavMeshTile loaded = {};
        REQUIRE(nav_mesh_io::load_tile(loaded, bytes) == LOAD_STATUS_OK);
        CHECK(loaded.tile_x == tile.tile_x);
        CHECK(loaded.tile_y == tile.tile_y);
        CHECK(loaded.tile_layer == tile.tile_layer);
        CHECK(SAME_ARRAY(loaded.vertices, tile.vertices));
        CHECK(same_polys(loaded.polys, tile.polys));
        CHECK(SAME_ARRAY(loaded.poly_vertices, tile.poly_vertices));
        CHECK(SAME_ARRAY(loaded.poly_neis, tile.poly_neis));
        CHECK(SAME_ARRAY(loaded.detail_vertices, tile.detail_vertices));
        CHECK(SAME_ARRAY(loaded.detail_triangles, tile.detail_triangles));
        CHECK(loaded.detail_meshes.length == tile.detail_meshes.length);
        CHECK(same_bv_nodes(loaded.bv_tree.nodes, tile.bv_tree.nodes));
        CHECK(loaded.bv_tree.quant_factor == tile.bv_tree.quant_factor);
        CHECK(loaded.walkable_climb == tile.walkable_climb);
        nav_mesh_api::release(loaded);
        bytes.release();
    }
    CHECK(tile_count > 1);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("re adding loaded tiles restores the same links") {
    NavMesh nav_mesh = create_tiled_terrain();
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, 0);
    Array<LinkKey> before = link_keys(nav_mesh);

    Array<Array<uint8_t>> saved = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (tile.allocated) saved.push(save_tile_bytes(tile));
    }
    for (Array<uint8_t> &bytes : saved) {
        NavMeshTile loaded = {};
        REQUIRE(nav_mesh_io::load_tile(loaded, bytes) == LOAD_STATUS_OK);
        nav_mesh_api::remove_tile(nav_mesh, loaded.tile_x, loaded.tile_y, loaded.tile_layer);
        nav_mesh_api::add_tile(nav_mesh, loaded);
        bytes.release();
    }
    saved.release();

    Array<LinkKey> after = link_keys(nav_mesh);
    CHECK(before.length > 0);
    CHECK(same_links(after, before));
    after.release();
    before.release();
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("a saved nav mesh loads back identically after tile and off mesh churn") {
    NavMesh nav_mesh = create_tiled_terrain();
    const NavMeshTile *first = nullptr;
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (tile.allocated) {
            first = &tile;
            break;
        }
    }
    REQUIRE(first != nullptr);
    Array<uint8_t> tile_bytes = save_tile_bytes(*first);
    int x = first->tile_x;
    int y = first->tile_y;
    nav_mesh_api::remove_tile(nav_mesh, x, y, 0);
    uint32_t extra = nav_mesh_api::add_off_mesh_connection(nav_mesh, {{3, 0, 21}, {21, 0, 3}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 1});
    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{6, 0, 6}, {18, 0, 18}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 1});
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, extra);
    NavMeshTile readded = {};
    REQUIRE(nav_mesh_io::load_tile(readded, tile_bytes) == LOAD_STATUS_OK);
    nav_mesh_api::add_tile(nav_mesh, readded);
    tile_bytes.release();

    Array<NodeRef> refs = {};
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (node.allocated) refs.push(node.ref);
    }
    uint64_t paths = hash_paths(nav_mesh);
    uint64_t links = hash_links(nav_mesh);
    Array<uint8_t> bytes = save_nav_mesh_bytes(nav_mesh);

    NavMesh loaded = {};
    REQUIRE(nav_mesh_io::load_nav_mesh(loaded, bytes) == LOAD_STATUS_OK);
    for (NodeRef ref : refs) CHECK(nav_mesh_api::is_valid_node_ref(loaded, ref));
    CHECK(hash_links(loaded) == links);
    CHECK(hash_paths(loaded) == paths);

    Array<uint8_t> again = save_nav_mesh_bytes(loaded);
    CHECK(SAME_ARRAY(again, bytes));

    const OffMeshConnectionParams next_connection = {{4, 0, 4}, {8, 0, 8}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 1};
    uint32_t original_next = nav_mesh_api::add_off_mesh_connection(nav_mesh, next_connection);
    uint32_t loaded_next = nav_mesh_api::add_off_mesh_connection(loaded, next_connection);
    CHECK(original_next == loaded_next);
    CHECK(hash_links(loaded) == hash_links(nav_mesh));

    Array<uint8_t> moved = save_tile_bytes(nav_mesh.tiles[nav_mesh.nodes[0].allocated ? (uint32_t)nav_mesh.nodes[0].tile_id : 0]);
    NavMeshTile original_tile = {};
    NavMeshTile loaded_tile = {};
    REQUIRE(nav_mesh_io::load_tile(original_tile, moved) == LOAD_STATUS_OK);
    REQUIRE(nav_mesh_io::load_tile(loaded_tile, moved) == LOAD_STATUS_OK);
    nav_mesh_api::add_tile(nav_mesh, original_tile);
    nav_mesh_api::add_tile(loaded, loaded_tile);
    CHECK(hash_links(loaded) == hash_links(nav_mesh));
    CHECK(hash_paths(loaded) == hash_paths(nav_mesh));
    moved.release();

    again.release();
    bytes.release();
    refs.release();
    nav_mesh_api::release(loaded);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("saving reports the size it needs") {
    NavMesh nav_mesh = create_tiled_terrain();
    SaveResult measured = {};
    nav_mesh_io::save_nav_mesh(measured, nav_mesh, {});
    CHECK_FALSE(measured.success);
    CHECK(measured.truncated);
    CHECK(measured.byte_count == 0);
    CHECK(measured.byte_total > 0);

    Array<uint8_t> bytes = {};
    bytes.resize(measured.byte_total - 1);
    SaveResult short_save = {};
    nav_mesh_io::save_nav_mesh(short_save, nav_mesh, bytes);
    CHECK(short_save.truncated);

    bytes.resize(measured.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);
    CHECK(saved.success);
    CHECK_FALSE(saved.truncated);
    CHECK(saved.byte_count == measured.byte_total);
    bytes.release();
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("loading rejects wrong magic and versions") {
    NavMesh nav_mesh = create_tiled_terrain();
    Array<uint8_t> bytes = save_nav_mesh_bytes(nav_mesh);
    NavMesh loaded = {};

    bytes[0] = 'X';
    CHECK(nav_mesh_io::load_nav_mesh(loaded, bytes) == LOAD_STATUS_BAD_MAGIC);
    bytes[0] = 'R';
    bytes[4] = 99;
    CHECK(nav_mesh_io::load_nav_mesh(loaded, bytes) == LOAD_STATUS_UNSUPPORTED_VERSION);
    bytes[4] = (uint8_t)NAV_MESH_FORMAT_VERSION;

    NavMeshTile tile = {};
    CHECK(nav_mesh_io::load_tile(tile, bytes) == LOAD_STATUS_BAD_MAGIC);
    CHECK(nav_mesh_io::load_nav_mesh(loaded, Span<const uint8_t>{}) == LOAD_STATUS_TRUNCATED);

    bytes.release();
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("loading every truncation of a saved nav mesh fails cleanly") {
    NavMesh nav_mesh = create_tiled_terrain();
    Array<uint8_t> bytes = save_nav_mesh_bytes(nav_mesh);
    for (uint32_t length = 0; length < bytes.length; length += length < 512 ? 1 : 61) {
        NavMesh loaded = {};
        LoadStatus status = nav_mesh_io::load_nav_mesh(loaded, {bytes.data, length});
        CHECK(status != LOAD_STATUS_OK);
        CHECK(loaded.nodes.length == 0);
    }
    bytes.release();
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("loading corrupted nav meshes never yields an unsafe mesh") {
    NavMesh nav_mesh = create_tiled_terrain();
    Array<uint8_t> bytes = save_nav_mesh_bytes(nav_mesh);
    Array<uint8_t> corrupted = {};
    corrupted.resize(bytes.length);
    uint32_t state = 12345;
    auto next = [&]() {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    };

    uint32_t accepted = 0;
    for (uint32_t trial = 0; trial < 3000; trial++) {
        memcpy(corrupted.data, bytes.data, bytes.length);
        uint32_t flips = 1 + next() % 4;
        for (uint32_t f = 0; f < flips; f++) corrupted[next() % corrupted.length] ^= (uint8_t)(1u << (next() % 8));

        NavMesh loaded = {};
        if (nav_mesh_io::load_nav_mesh(loaded, corrupted) != LOAD_STATUS_OK) continue;
        accepted++;
        hash_paths(loaded);
        hash_links(loaded);
        nav_mesh_api::release(loaded);
    }
    CHECK(accepted > 0);

    corrupted.release();
    bytes.release();
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("loading corrupted tiles never yields an unsafe tile") {
    NavMesh nav_mesh = create_tiled_terrain();
    const NavMeshTile *source = nullptr;
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (tile.allocated) source = &tile;
    }
    REQUIRE(source != nullptr);
    Array<uint8_t> bytes = save_tile_bytes(*source);
    Array<uint8_t> corrupted = {};
    corrupted.resize(bytes.length);
    uint32_t state = 777;
    auto next = [&]() {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    };

    for (uint32_t trial = 0; trial < 2000; trial++) {
        memcpy(corrupted.data, bytes.data, bytes.length);
        corrupted[next() % corrupted.length] ^= (uint8_t)(1u << (next() % 8));
        NavMeshTile loaded = {};
        if (nav_mesh_io::load_tile(loaded, corrupted) != LOAD_STATUS_OK) continue;
        nav_mesh_api::remove_tile(nav_mesh, loaded.tile_x, loaded.tile_y, loaded.tile_layer);
        nav_mesh_api::add_tile(nav_mesh, loaded);
        hash_paths(nav_mesh);
    }

    corrupted.release();
    bytes.release();
    nav_mesh_api::release(nav_mesh);
}
