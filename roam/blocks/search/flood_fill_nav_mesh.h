#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/util/array.h"

namespace roam {

struct FloodFillNavMeshResult {
    Array<NodeRef> reachable;
    Array<NodeRef> unreachable;
};

namespace flood_fill_nav_mesh {

FloodFillNavMeshResult flood_fill_nav_mesh(const NavMesh &nav_mesh, const NodeRef *start_node_refs, uint32_t start_node_ref_count);

void release(FloodFillNavMeshResult &result);

}

}
