#include "roam/query/find_path.h"

namespace roam {

namespace find_path {

FindPathResult find_path(const NavMesh &nav_mesh, Vec3 start, Vec3 end, Vec3 half_extents, const QueryFilter &filter, FindPathOptions options,
                         SearchWorkspace *workspace) {
    FindPathResult result = {};
    result.flags = FIND_PATH_RESULT_NONE | FIND_PATH_RESULT_INVALID_INPUT;
    result.node_path_flags = FIND_NODE_PATH_RESULT_NONE;
    result.straight_path_flags = FIND_STRAIGHT_PATH_RESULT_NONE;
    result.start_node_ref = INVALID_NODE_REF;
    result.end_node_ref = INVALID_NODE_REF;

    FindNearestPolyResult start_nearest = nav_mesh_api::find_nearest_poly(nav_mesh, start, half_extents, filter);
    if (!start_nearest.success) return result;
    result.start_position = start_nearest.position;
    result.start_node_ref = start_nearest.node_ref;

    FindNearestPolyResult end_nearest = nav_mesh_api::find_nearest_poly(nav_mesh, end, half_extents, filter);
    if (!end_nearest.success) return result;
    result.end_position = end_nearest.position;
    result.end_node_ref = end_nearest.node_ref;

    result.node_path = nav_mesh_search::find_node_path(nav_mesh, result.start_node_ref, result.end_node_ref, result.start_position,
                                                       result.end_position, filter, {options.raycast_distance}, workspace);
    result.node_path_flags = result.node_path.flags;

    if (!result.node_path.success) {
        result.flags = FIND_PATH_RESULT_FIND_NODE_PATH_FAILED;
        return result;
    }

    FindStraightPathResult straight_path = find_straight_path::find_straight_path(nav_mesh, result.start_position, result.end_position,
                                                                                  result.node_path.path.data, result.node_path.path.length);

    if (!straight_path.success) {
        find_straight_path::release(straight_path);
        result.flags = FIND_PATH_RESULT_FIND_STRAIGHT_PATH_FAILED;
        return result;
    }

    result.success = true;
    result.path = straight_path.path;
    result.straight_path_flags = straight_path.flags;

    uint32_t flags = FIND_PATH_RESULT_SUCCESS;
    if (result.node_path.flags & FIND_NODE_PATH_RESULT_COMPLETE_PATH && (straight_path.flags & FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH) == 0) {
        flags |= FIND_PATH_RESULT_COMPLETE_PATH;
    } else {
        flags |= FIND_PATH_RESULT_PARTIAL_PATH;
    }
    if (straight_path.flags & FIND_STRAIGHT_PATH_RESULT_MAX_POINTS_REACHED) flags |= FIND_PATH_RESULT_MAX_POINTS_REACHED;

    result.flags = flags;
    return result;
}

void release(FindPathResult &result) {
    result.path.release();
    nav_mesh_search::release(result.node_path);
}

}

}
