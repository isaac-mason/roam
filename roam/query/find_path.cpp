#include "roam/query/find_path.h"

namespace roam {

namespace find_path {

void find_path(FindPathResult &out, SearchWorkspace &workspace, const NavMesh &nav_mesh, Vec3 start, Vec3 end, Vec3 half_extents,
               const QueryFilter &filter, StraightPathPoint *points, uint32_t max_points, FindPathOptions options) {
    out = {};
    out.flags = FIND_PATH_RESULT_NONE | FIND_PATH_RESULT_INVALID_INPUT;
    out.straight_path_flags = FIND_STRAIGHT_PATH_RESULT_NONE;
    out.start_node_ref = INVALID_NODE_REF;
    out.end_node_ref = INVALID_NODE_REF;
    out.node_path_flags = FIND_NODE_PATH_RESULT_NONE;

    FindNearestPolyResult start_nearest;
    nav_mesh_api::find_nearest_poly(start_nearest, nav_mesh, start, half_extents, filter);
    if (!start_nearest.success) return;
    out.start_position = start_nearest.position;
    out.start_node_ref = start_nearest.node_ref;

    FindNearestPolyResult end_nearest;
    nav_mesh_api::find_nearest_poly(end_nearest, nav_mesh, end, half_extents, filter);
    if (!end_nearest.success) return;
    out.end_position = end_nearest.position;
    out.end_node_ref = end_nearest.node_ref;

    FindNodePathResult node_path;
    nav_mesh_search::find_node_path_to_workspace(node_path, workspace, nav_mesh, out.start_node_ref, out.end_node_ref, out.start_position, out.end_position,
                                         filter, {options.raycast_distance});
    const Array<NodeRef> &node_refs = workspace.node_path;
    out.node_path_flags = node_path.flags;
    out.full_node_path_count = node_refs.length;
    out.node_path_count = node_refs.length < options.max_node_path ? node_refs.length : options.max_node_path;
    for (uint32_t i = 0; i < out.node_path_count; i++) options.node_path[i] = node_refs[i];

    if (!node_path.success) {
        out.flags = FIND_PATH_RESULT_FIND_NODE_PATH_FAILED;
        return;
    }

    FindStraightPathResult straight_path;
    find_straight_path::find_straight_path(straight_path, nav_mesh, out.start_position, out.end_position, node_refs.data, node_refs.length, points,
                                           max_points);

    if (!straight_path.success) {
        out.flags = FIND_PATH_RESULT_FIND_STRAIGHT_PATH_FAILED;
        return;
    }

    out.success = true;
    out.point_count = straight_path.point_count;
    out.straight_path_flags = straight_path.flags;

    uint32_t flags = FIND_PATH_RESULT_SUCCESS;
    if (node_path.flags & FIND_NODE_PATH_RESULT_COMPLETE_PATH && (straight_path.flags & FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH) == 0) {
        flags |= FIND_PATH_RESULT_COMPLETE_PATH;
    } else {
        flags |= FIND_PATH_RESULT_PARTIAL_PATH;
    }
    if (straight_path.flags & FIND_STRAIGHT_PATH_RESULT_MAX_POINTS_REACHED) flags |= FIND_PATH_RESULT_MAX_POINTS_REACHED;

    out.flags = flags;
}

}

}
