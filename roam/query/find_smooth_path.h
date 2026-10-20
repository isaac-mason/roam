#pragma once

#include <stdint.h>

#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

enum FindSmoothPathResultFlags : uint32_t {
    FIND_SMOOTH_PATH_RESULT_NONE = 0,
    FIND_SMOOTH_PATH_RESULT_SUCCESS = 1 << 0,
    FIND_SMOOTH_PATH_RESULT_COMPLETE_PATH = 1 << 1,
    FIND_SMOOTH_PATH_RESULT_PARTIAL_PATH = 1 << 2,
    FIND_SMOOTH_PATH_RESULT_INVALID_INPUT = 1 << 3,
    FIND_SMOOTH_PATH_RESULT_FIND_NODE_PATH_FAILED = 1 << 4,
    FIND_SMOOTH_PATH_RESULT_FIND_STRAIGHT_PATH_FAILED = 1 << 5,
};

using SmoothPathPoint = StraightPathPoint;

struct FindSmoothPathResult {
    bool success;
    uint32_t flags;
    Array<SmoothPathPoint> path;
    NodeRef start_node_ref;
    Vec3 start_position;
    NodeRef end_node_ref;
    Vec3 end_position;
    FindNodePathResult node_path;
};

struct FindSmoothPathOptions {
    float step_size;
    float slop;
    uint32_t max_points;
    float raycast_distance;
};

namespace find_smooth_path {

FindSmoothPathResult find_smooth_path(const NavMesh &nav_mesh, Vec3 start, Vec3 end, Vec3 half_extents, const QueryFilter &filter,
                                      const FindSmoothPathOptions &options, SearchWorkspace *workspace = nullptr);

void release(FindSmoothPathResult &result);

}

}
