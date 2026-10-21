#include "roam/query/nav_mesh_api.h"
#include "tests/harness/test.h"

using namespace roam;

static NavMeshTile mock_tile(int tile_x, int tile_y, int tile_layer) {
    NavMeshTile tile = {};
    tile.tile_x = tile_x;
    tile.tile_y = tile_y;
    tile.tile_layer = tile_layer;
    tile.bounds = {{0, 0, 0}, {10, 10, 10}};
    tile.cell_size = 0.3f;
    tile.cell_height = 0.2f;
    tile.walkable_height = 2;
    tile.walkable_radius = 0.6f;
    tile.walkable_climb = 0.9f;
    return tile;
}

TEST_CASE("get tile at returns tile at position") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(0, 0, 0));

    const NavMeshTile *tile = nav_mesh_api::get_tile_at(nav_mesh, 0, 0, 0);
    REQUIRE(tile != nullptr);
    CHECK((tile->tile_x == 0 && tile->tile_y == 0 && tile->tile_layer == 0));

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile at returns null for missing position") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    CHECK(nav_mesh_api::get_tile_at(nav_mesh, 5, 5, 0) == nullptr);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile at differentiates layers") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(1, 1, 0));
    nav_mesh_api::add_tile(nav_mesh, mock_tile(1, 1, 1));

    const NavMeshTile *layer_0 = nav_mesh_api::get_tile_at(nav_mesh, 1, 1, 0);
    const NavMeshTile *layer_1 = nav_mesh_api::get_tile_at(nav_mesh, 1, 1, 1);
    REQUIRE((layer_0 != nullptr && layer_1 != nullptr));
    CHECK(layer_0->tile_layer == 0);
    CHECK(layer_1->tile_layer == 1);
    CHECK(layer_0->id != layer_1->id);

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile at handles negative coordinates") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(-5, -3, 0));

    const NavMeshTile *tile = nav_mesh_api::get_tile_at(nav_mesh, -5, -3, 0);
    REQUIRE(tile != nullptr);
    CHECK((tile->tile_x == -5 && tile->tile_y == -3));

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile ids at returns all layers") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    for (int layer = 0; layer < 3; layer++) nav_mesh_api::add_tile(nav_mesh, mock_tile(2, 3, layer));

    Span<const uint32_t> tile_ids = nav_mesh_api::get_tile_ids_at(nav_mesh, 2, 3);
    REQUIRE(tile_ids.capacity == 3);
    int layer_mask = 0;
    for (uint32_t tile_id : tile_ids) layer_mask |= 1 << nav_mesh.tiles[tile_id].tile_layer;
    CHECK(layer_mask == 0x7);

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile ids at returns empty for missing position") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    CHECK(nav_mesh_api::get_tile_ids_at(nav_mesh, 10, 10).capacity == 0);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile ids at returns single tile when only one layer exists") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(0, 0, 0));

    Span<const uint32_t> tile_ids = nav_mesh_api::get_tile_ids_at(nav_mesh, 0, 0);
    REQUIRE(tile_ids.capacity == 1);
    CHECK(nav_mesh.tiles[tile_ids[0]].tile_layer == 0);

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile ids at does not return tiles from different x y positions") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(0, 0, 0));
    nav_mesh_api::add_tile(nav_mesh, mock_tile(1, 0, 0));

    Span<const uint32_t> tile_ids = nav_mesh_api::get_tile_ids_at(nav_mesh, 0, 0);
    REQUIRE(tile_ids.capacity == 1);
    CHECK(nav_mesh.tiles[tile_ids[0]].tile_x == 0);

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("get tile ids at handles negative coordinates and ignores other columns") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(-10, -5, 0));
    nav_mesh_api::add_tile(nav_mesh, mock_tile(-9, -5, 0));

    Span<const uint32_t> tile_ids = nav_mesh_api::get_tile_ids_at(nav_mesh, -10, -5);
    REQUIRE(tile_ids.capacity == 1);
    CHECK((nav_mesh.tiles[tile_ids[0]].tile_x == -10 && nav_mesh.tiles[tile_ids[0]].tile_y == -5));

    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("add tile replaces tile at same position") {
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh_api::add_tile(nav_mesh, mock_tile(0, 0, 0));
    uint32_t first_sequence = nav_mesh_api::get_tile_at(nav_mesh, 0, 0, 0)->sequence;
    nav_mesh_api::add_tile(nav_mesh, mock_tile(0, 0, 0));

    CHECK(nav_mesh_api::get_tile_ids_at(nav_mesh, 0, 0).capacity == 1);
    CHECK(nav_mesh_api::get_tile_at(nav_mesh, 0, 0, 0)->sequence == first_sequence + 1);

    nav_mesh_api::release(nav_mesh);
}
