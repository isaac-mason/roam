#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/util/array.h"
#include "roam/util/span.h"

namespace roam {

struct FloodFillNavMeshResult {
    uint32_t reachable_count;
    uint32_t reachable_total;
    uint32_t unreachable_count;
    uint32_t unreachable_total;
    bool truncated;
};

namespace flood_fill_nav_mesh {

void flood_fill_nav_mesh(FloodFillNavMeshResult &out, const NavMesh &nav_mesh, const NodeRef *start_node_refs, uint32_t start_node_ref_count,
                         Span<NodeRef> reachable = {}, Span<NodeRef> unreachable = {});

}

}
