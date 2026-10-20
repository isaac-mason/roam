#pragma once

#include <stdint.h>

#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

enum FindPathResultFlags : uint32_t {
    FIND_PATH_RESULT_NONE = 0,
    FIND_PATH_RESULT_SUCCESS = 1 << 0,
    FIND_PATH_RESULT_COMPLETE_PATH = 1 << 1,
    FIND_PATH_RESULT_PARTIAL_PATH = 1 << 2,
    FIND_PATH_RESULT_MAX_POINTS_REACHED = 1 << 3,
    FIND_PATH_RESULT_INVALID_INPUT = 1 << 4,
    FIND_PATH_RESULT_FIND_NODE_PATH_FAILED = 1 << 5,
    FIND_PATH_RESULT_FIND_STRAIGHT_PATH_FAILED = 1 << 6,
};

struct FindPathResult {
    bool success;
    uint32_t flags;
    Array<StraightPathPoint> path;
    uint32_t straight_path_flags;
    NodeRef start_node_ref;
    Vec3 start_position;
    NodeRef end_node_ref;
    Vec3 end_position;
    FindNodePathResult node_path;
    uint32_t node_path_flags;
};

struct FindPathOptions {
    float raycast_distance;
};

namespace find_path {

FindPathResult find_path(const NavMesh &nav_mesh, Vec3 start, Vec3 end, Vec3 half_extents, const QueryFilter &filter, FindPathOptions options = {});

void release(FindPathResult &result);

}

}
