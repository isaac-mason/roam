#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/util/array.h"

namespace roam {

enum FindStraightPathOptions : uint32_t {
    FIND_STRAIGHT_PATH_ALL_CROSSINGS = 1,
    FIND_STRAIGHT_PATH_AREA_CROSSINGS = 2,
};

enum StraightPathPointFlags : uint32_t {
    STRAIGHT_PATH_POINT_START = 0,
    STRAIGHT_PATH_POINT_END = 1,
    STRAIGHT_PATH_POINT_OFFMESH = 2,
};

struct StraightPathPoint {
    Vec3 position;
    NodeType type;
    NodeRef node_ref;
    uint32_t flags;
};

enum FindStraightPathResultFlags : uint32_t {
    FIND_STRAIGHT_PATH_RESULT_NONE = 0,
    FIND_STRAIGHT_PATH_RESULT_SUCCESS = 1 << 0,
    FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH = 1 << 2,
    FIND_STRAIGHT_PATH_RESULT_MAX_POINTS_REACHED = 1 << 3,
    FIND_STRAIGHT_PATH_RESULT_INVALID_INPUT = 1 << 4,
};

struct FindStraightPathResult {
    uint32_t flags;
    bool success;
    uint32_t point_count;
};

namespace find_straight_path {

void find_straight_path(FindStraightPathResult &out, const NavMesh &nav_mesh, Vec3 start, Vec3 end, const NodeRef *path_node_refs,
                        uint32_t path_node_count, StraightPathPoint *points, uint32_t max_points, uint32_t straight_path_options = 0);

}

}
