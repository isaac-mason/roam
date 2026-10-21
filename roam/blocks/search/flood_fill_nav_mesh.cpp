#include "roam/blocks/search/flood_fill_nav_mesh.h"

#include "roam/query/nav_mesh_api.h"
#include "roam/util/map.h"

namespace roam {

namespace flood_fill_nav_mesh {

static void write_ref(Span<NodeRef> refs, uint32_t &count, NodeRef ref) {
    if (count < refs.capacity) refs.data[count] = ref;
    count++;
}

void flood_fill_nav_mesh(FloodFillNavMeshResult &result, const NavMesh &nav_mesh, Span<const NodeRef> seeds, Span<NodeRef> reachable,
                         Span<NodeRef> unreachable) {
    result = {0, 0, 0, 0, false};

    Map<NodeRef, uint8_t> visited = {};
    Array<NodeRef> queue = {};
    for (NodeRef seed : seeds) queue.push(seed);

    for (uint32_t head = 0; head < queue.length; head++) {
        NodeRef current = queue[head];
        if (visited.has(current)) continue;

        visited.set(current, 1);
        write_ref(reachable, result.reachable_total, current);

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
            if (!visited.has(node.ref)) write_ref(unreachable, result.unreachable_total, node.ref);
        }
    }

    result.reachable_count = result.reachable_total < reachable.capacity ? result.reachable_total : reachable.capacity;
    result.unreachable_count = result.unreachable_total < unreachable.capacity ? result.unreachable_total : unreachable.capacity;
    result.truncated = result.reachable_count < result.reachable_total || result.unreachable_count < result.unreachable_total;

    visited.release();
    queue.release();
}

}

}
