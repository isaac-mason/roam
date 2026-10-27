#include <stdio.h>

#include "snippets.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_io.h"

namespace roam {

static void node_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, {-8, 0, 0}, {1, 1, 1}, scene.filter);
    const NodeRef node_ref = nearest.node_ref;

    /* SNIPPET_START: nodes_and_refs */
    // a ref packs the node's type, its index in nav_mesh.nodes and its tile's sequence. removing
    // a tile and adding another bumps the sequence, so a ref kept from before no longer resolves
    if (nav_mesh_api::is_valid_node_ref(nav_mesh, node_ref)) {
        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, node_ref);
        const char *type = node.type == NODE_TYPE_POLY ? "a poly" : "an off-mesh connection";
        printf("%s, area %u, flags %u\n", type, node.area, node.flags);
    }

    // a poly's tile and record, and its vertices in world space
    TileAndPoly found = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, node_ref);
    if (found.success) {
        for (uint32_t i = 0; i < found.poly->vertex_count; i++) {
            const uint32_t index = found.tile->poly_vertices[found.poly->vertex_base + i];
            const Vec3 vertex = vec3::load(found.tile->vertices.data, index * 3);
            printf("  %.2f %.2f %.2f\n", vertex.x, vertex.y, vertex.z);
        }
    }

    // every node and the links leaving it: the graph that searches walk
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (!node.allocated) continue;
        for (int32_t link_index = node.first_link; link_index != NULL_LINK;) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];
            const NodeRef neighbour = link.to_node_ref;
            (void)neighbour;
            link_index = link.next;
        }
    }
    /* SNIPPET_END: nodes_and_refs */
}

static void tile_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;

    /* SNIPPET_START: tiles */
    // the tile at a grid position, and the grid position of a point
    const Vec2 cell = nav_mesh_api::world_to_tile_position(nav_mesh, {0, 0, 0});
    const int x = (int)cell.x;
    const int y = (int)cell.y;
    const NavMeshTile *tile = nav_mesh_api::get_tile_at(nav_mesh, x, y, 0);

    // a copy of it, as bytes, before taking it out: its polys' refs go with it
    SaveResult size = {};
    nav_mesh_io::save_tile(size, *tile, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_tile(saved, *tile, bytes);
    nav_mesh_api::remove_tile(nav_mesh, x, y, 0);

    // and back: add_tile takes the tile, links it to its neighbours and returns its id
    NavMeshTile copy = {};
    if (nav_mesh_io::load_tile(copy, bytes) == LOAD_STATUS_OK) nav_mesh_api::add_tile(nav_mesh, copy);
    bytes.release();
    /* SNIPPET_END: tiles */
}

static void poly_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    const QueryFilter &filter = scene.filter;
    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);
    const NodeRef node_ref = nearest.node_ref;

    /* SNIPPET_START: polys */
    // every poly whose bounds overlap a box
    const Box3 box = {{-10, -1, -10}, {0, 1, 0}};
    NodeRef found[64];
    QueryPolygonsResult query = {};
    nav_mesh_api::query_polygons(query, nav_mesh, box, filter, found);

    // the nearest point on a poly to a point, and whether the point is over the poly
    GetClosestPointOnPolyResult closest = {};
    nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, node_ref, {-8, 3, 0});

    // the shared edge between two neighbouring polys, as the funnel sees it
    GetPortalPointsResult portal = {};
    if (query.poly_count > 1) nav_mesh_search::get_portal_points(portal, nav_mesh, found[0], found[1]);
    /* SNIPPET_END: polys */
}

static void external_snippets() {
    /* SNIPPET_START: external_polygons */
    // two squares side by side, sharing an edge: indexed vertices, so shared edges become links
    const float vertices[] = {0, 0, 0, 4, 0, 0, 8, 0, 0, 0, 0, 4, 4, 0, 4, 8, 0, 4};
    const uint32_t left[] = {0, 3, 4, 1};
    const uint32_t right[] = {1, 4, 5, 2};
    const ExternalPolygon polygons[] = {
        {.vertices = left, .flags = 1, .area = 0},
        {.vertices = right, .flags = 1, .area = 0},
    };
    const Box3 bounds = {{0, 0, 0}, {8, 1, 4}};

    NavMeshTilePolys tile_polys =
        nav_mesh_tile::polygons_to_nav_mesh_tile_polys(polygons, vertices, 0, bounds);
    // no detail mesh of its own: one built from the polygons themselves
    NavMeshTileDetailMesh tile_detail = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);

    const NavMeshTileParams params = {
        .bounds = bounds,
        .vertices = tile_polys.vertices,
        .polys = tile_polys.polys,
        .poly_vertices = tile_polys.poly_vertices,
        .poly_neis = tile_polys.poly_neis,
        .detail_meshes = tile_detail.detail_meshes,
        .detail_vertices = tile_detail.detail_vertices,
        .detail_triangles = tile_detail.detail_triangles,
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .walkable_height = 1.8f,
        .walkable_radius = 0.3f,
        .walkable_climb = 0.4f,
    };

    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = bounds.min;
    nav_mesh.tile_width = 8;
    nav_mesh.tile_height = 4;
    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(tile_detail);
    /* SNIPPET_END: external_polygons */

    nav_mesh_api::release(nav_mesh);
}

static void io_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;

    /* SNIPPET_START: save_load */
    // an empty buffer first, for the size; then the bytes
    SaveResult size = {};
    nav_mesh_io::save_nav_mesh(size, nav_mesh, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);

    // the same nav mesh back: the same refs, the same query results
    NavMesh loaded = {};
    const LoadStatus status = nav_mesh_io::load_nav_mesh(loaded, bytes);
    if (status != LOAD_STATUS_OK) printf("not a nav mesh: %u\n", status);
    /* SNIPPET_END: save_load */

    bytes.release();
    nav_mesh_api::release(loaded);
}

void run_advanced_snippets(Scene &scene) {
    node_snippets(scene);
    tile_snippets(scene);
    poly_snippets(scene);
    external_snippets();
    io_snippets(scene);
}

}
