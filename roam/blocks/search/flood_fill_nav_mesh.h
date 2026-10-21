#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/util/array.h"

namespace roam {

struct FloodFillNavMeshResult {
    uint32_t reachable_count;
    uint32_t full_reachable_count;
    uint32_t unreachable_count;
    uint32_t full_unreachable_count;
};

namespace flood_fill_nav_mesh {

void flood_fill_nav_mesh(FloodFillNavMeshResult &out, const NavMesh &nav_mesh, const NodeRef *start_node_refs, uint32_t start_node_ref_count,
                         NodeRef *reachable, uint32_t max_reachable, NodeRef *unreachable, uint32_t max_unreachable);

}

}
