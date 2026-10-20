#include "roam/blocks/search/flood_fill_nav_mesh.h"

#include "roam/query/nav_mesh_api.h"
#include "roam/util/map.h"

namespace roam {

namespace flood_fill_nav_mesh {

FloodFillNavMeshResult flood_fill_nav_mesh(const NavMesh &nav_mesh, const NodeRef *start_node_refs, uint32_t start_node_ref_count) {
    FloodFillNavMeshResult result = {};

    Map<NodeRef, uint8_t> visited = {};
    Array<NodeRef> queue = {};
    for (uint32_t i = 0; i < start_node_ref_count; i++) queue.push(start_node_refs[i]);

    for (uint32_t head = 0; head < queue.length; head++) {
        NodeRef current = queue[head];
        if (visited.has(current)) continue;

        visited.set(current, 1);
        result.reachable.push(current);

        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, current);
        for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
            NodeRef to = nav_mesh.links[(uint32_t)link].to_node_ref;
            if (!visited.has(to)) queue.push(to);
        }
    }

    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
            const NavMeshNode &node = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, tile, (int)poly_index);
            if (!visited.has(node.ref)) result.unreachable.push(node.ref);
        }
    }

    visited.release();
    queue.release();
    return result;
}

void release(FloodFillNavMeshResult &result) {
    result.reachable.release();
    result.unreachable.release();
}

}

}
