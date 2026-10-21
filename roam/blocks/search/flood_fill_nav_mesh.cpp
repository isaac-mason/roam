#include "roam/blocks/search/flood_fill_nav_mesh.h"

#include "roam/query/nav_mesh_api.h"
#include "roam/util/map.h"

namespace roam {

namespace flood_fill_nav_mesh {

static void write_ref(NodeRef *refs, uint32_t capacity, uint32_t &count, NodeRef ref) {
    if (count < capacity) refs[count] = ref;
    count++;
}

void flood_fill_nav_mesh(FloodFillNavMeshResult &result, const NavMesh &nav_mesh, const NodeRef *start_node_refs, uint32_t start_node_ref_count,
                         NodeRef *reachable, uint32_t max_reachable, NodeRef *unreachable, uint32_t max_unreachable) {
    result = {0, 0, 0, 0};

    Map<NodeRef, uint8_t> visited = {};
    Array<NodeRef> queue = {};
    for (uint32_t i = 0; i < start_node_ref_count; i++) queue.push(start_node_refs[i]);

    for (uint32_t head = 0; head < queue.length; head++) {
        NodeRef current = queue[head];
        if (visited.has(current)) continue;

        visited.set(current, 1);
        write_ref(reachable, max_reachable, result.full_reachable_count, current);

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
            if (!visited.has(node.ref)) write_ref(unreachable, max_unreachable, result.full_unreachable_count, node.ref);
        }
    }

    result.reachable_count = result.full_reachable_count < max_reachable ? result.full_reachable_count : max_reachable;
    result.unreachable_count = result.full_unreachable_count < max_unreachable ? result.full_unreachable_count : max_unreachable;

    visited.release();
    queue.release();
}

}

}
