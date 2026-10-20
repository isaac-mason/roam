#pragma once

#include "roam/query/nav_mesh.h"

namespace roam {

namespace bv_tree {

NavMeshTileBvTree build_nav_mesh_bv_tree(const NavMeshTileParams &params);

}

}
