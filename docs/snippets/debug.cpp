#include <stdio.h>

#include "snippets.h"
#include "roam/debug/debug.h"

namespace roam {

void run_debug_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const InputTriangleMesh level = {scene.level.positions, scene.level.indices};
    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(level, scene_options());
    SoloNavMeshIntermediates &intermediates = result.intermediates;
    const InputTriangleMesh &mesh = intermediates.input;
    Span<const uint8_t> tri_area_ids = intermediates.tri_area_ids;
    const Heightfield &solid = intermediates.heightfield;
    const CompactHeightfield &compact = intermediates.compact_heightfield;
    const ContourSet &contours = intermediates.contour_set;
    const PolyMesh &polys = intermediates.poly_mesh;
    const PolyMeshDetail &detail = intermediates.poly_mesh_detail;
    const NavMeshTile &tile = nav_mesh.tiles[0];
    const NodeRef node_ref = nav_mesh.nodes[0].ref;

    DebugPrimitives all[21] = {};
    uint32_t count = 0;
    /* SNIPPET_START: debug_helpers */
    // each stage of generation
    DebugPrimitives triangle_area_ids = debug::create_triangle_area_ids_helper(mesh, tri_area_ids);
    DebugPrimitives heightfield = debug::create_heightfield_helper(solid);
    DebugPrimitives compact_solid = debug::create_compact_heightfield_solid_helper(compact);
    DebugPrimitives distances = debug::create_compact_heightfield_distances_helper(compact);
    DebugPrimitives regions = debug::create_compact_heightfield_regions_helper(compact);
    DebugPrimitives raw_contours = debug::create_raw_contours_helper(contours);
    DebugPrimitives simplified_contours = debug::create_simplified_contours_helper(contours);
    DebugPrimitives poly_mesh = debug::create_poly_mesh_helper(polys);
    DebugPrimitives poly_mesh_detail = debug::create_poly_mesh_detail_helper(detail);

    // the nav mesh
    DebugPrimitives nav_mesh_triangles = debug::create_nav_mesh_helper(nav_mesh);
    DebugPrimitives nav_mesh_tile = debug::create_nav_mesh_tile_helper(tile);
    DebugPrimitives nav_mesh_poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
    DebugPrimitives islands = debug::create_nav_mesh_islands_helper(nav_mesh);
    DebugPrimitives tile_bv_tree = debug::create_nav_mesh_tile_bv_tree_helper(tile);
    DebugPrimitives bv_tree = debug::create_nav_mesh_bv_tree_helper(nav_mesh);
    DebugPrimitives links = debug::create_nav_mesh_links_helper(nav_mesh);
    DebugPrimitives tile_portals = debug::create_nav_mesh_tile_portals_helper(tile);
    DebugPrimitives portals = debug::create_nav_mesh_portals_helper(nav_mesh);
    DebugPrimitives off_mesh = debug::create_nav_mesh_off_mesh_connections_helper(nav_mesh);

    // the nodes the last search through this workspace visited
    DebugPrimitives search_nodes = debug::create_search_nodes_helper(workspace);
    /* SNIPPET_END: debug_helpers */

    const DebugPrimitives listed[] = {
        triangle_area_ids, heightfield, compact_solid, distances, regions, raw_contours,
        simplified_contours, poly_mesh, poly_mesh_detail, nav_mesh_triangles, nav_mesh_tile,
        nav_mesh_poly, islands, tile_bv_tree, bv_tree, links, tile_portals, portals, off_mesh,
        search_nodes,
    };
    for (const DebugPrimitives &primitives : listed) all[count++] = primitives;

    /* SNIPPET_START: debug_primitives */
    DebugPrimitives primitives = debug::create_nav_mesh_helper(nav_mesh);
    for (const DebugPrimitive &primitive : primitives) {
        if (primitive.type == DEBUG_PRIMITIVE_TRIANGLES) {
            printf("%u triangles\n", primitive.indices.length / 3);
        } else if (primitive.type == DEBUG_PRIMITIVE_LINES) {
            printf("%u lines\n", primitive.positions.length / 6);
        }
    }
    debug::release(primitives);
    /* SNIPPET_END: debug_primitives */

    for (uint32_t i = 0; i < count; i++) debug::release(all[i]);
    generate_solo_nav_mesh::release(result.intermediates);
    nav_mesh_api::release(result.nav_mesh);
}

}
