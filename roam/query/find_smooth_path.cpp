#include "roam/query/find_smooth_path.h"

#include <math.h>

namespace roam {

namespace find_smooth_path {

struct SteerTarget {
    bool success;
    Vec3 steer_position;
    NodeRef steer_position_ref;
    uint32_t steer_position_flags;
};

static bool in_range(Vec3 a, Vec3 b, float r, float h) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    return dx * dx + dz * dz < r * r && fabsf(dy) < h;
}

static SteerTarget get_steer_target(const NavMesh &nav_mesh, Vec3 start, Vec3 end, float min_target_distance, const Array<NodeRef> &path_polys) {
    SteerTarget result = {false, {0, 0, 0}, INVALID_NODE_REF, 0};

    const uint32_t max_straight_path_points = 3;
    StraightPathPoint points[max_straight_path_points];
    FindStraightPathResult straight_path;
    find_straight_path::find_straight_path(straight_path, nav_mesh, start, end, path_polys, points);

    if (straight_path.success && straight_path.point_count > 0) {
        uint32_t ns = 0;
        while (ns < straight_path.point_count) {
            const StraightPathPoint &point = points[ns];
            if (point.type == NODE_TYPE_OFFMESH) break;
            if (!in_range(point.position, start, min_target_distance, 1000.0f)) break;
            ns++;
        }

        if (ns < straight_path.point_count) {
            const StraightPathPoint &steer_point = points[ns];
            result = {true, steer_point.position, steer_point.node_ref, steer_point.flags};
        }
    }

    return result;
}

static void merge_corridor_start_moved(Array<NodeRef> &current_path, const Array<NodeRef> &visited, uint32_t max_path) {
    if (visited.length == 0) return;

    int furthest_path = -1;
    int furthest_visited = -1;

    for (int i = (int)current_path.length - 1; i >= 0; i--) {
        for (int j = (int)visited.length - 1; j >= 0; j--) {
            if (current_path[(uint32_t)i] == visited[(uint32_t)j]) {
                furthest_path = i;
                furthest_visited = j;
                break;
            }
        }
        if (furthest_path != -1) break;
    }

    if (furthest_path == -1 || furthest_visited == -1) return;

    int request = (int)visited.length - furthest_visited;
    int original = furthest_path + 1 < (int)current_path.length ? furthest_path + 1 : (int)current_path.length;
    int size = (int)current_path.length - original;
    if (size < 0) size = 0;

    if (request + size > (int)max_path) size = (int)max_path - request;

    Array<NodeRef> new_path = {};
    int count = request + size;
    new_path.resize((uint32_t)(count > 0 ? count : 0), INVALID_NODE_REF);

    int stored = request < (int)max_path ? request : (int)max_path;
    for (int i = 0; i < stored && i < count; i++) new_path[(uint32_t)i] = visited[visited.length - 1 - (uint32_t)i];

    for (int i = 0; i < size; i++) new_path[(uint32_t)(request + i)] = current_path[(uint32_t)(original + i)];

    current_path.release();
    current_path = new_path;
}

static void fixup_shortcuts(Array<NodeRef> &path_polys, const NavMesh &nav_mesh) {
    if (path_polys.length < 3) return;

    const uint32_t max_neighbours = 16;
    NodeRef neighbours[max_neighbours];
    uint32_t neighbour_count = 0;

    const NavMeshNode &first_node = nav_mesh_api::get_node_by_ref(nav_mesh, path_polys[0]);
    for (int32_t link = first_node.first_link; link != NULL_LINK && neighbour_count < max_neighbours; link = nav_mesh.links[(uint32_t)link].next) {
        neighbours[neighbour_count++] = nav_mesh.links[(uint32_t)link].to_node_ref;
    }

    const uint32_t max_look_ahead = 6;
    uint32_t cut = 0;
    for (int i = (int)(max_look_ahead < path_polys.length ? max_look_ahead : path_polys.length) - 1; i > 1 && cut == 0; i--) {
        for (uint32_t j = 0; j < neighbour_count; j++) {
            if (path_polys[(uint32_t)i] == neighbours[j]) {
                cut = (uint32_t)i;
                break;
            }
        }
    }

    if (cut > 1) {
        uint32_t removed = cut - 1;
        for (uint32_t i = 1; i + removed < path_polys.length; i++) path_polys[i] = path_polys[i + removed];
        path_polys.resize(path_polys.length - removed);
    }
}

static void remove_front(Array<NodeRef> &values, uint32_t count) {
    if (count > values.length) count = values.length;
    for (uint32_t i = 0; i + count < values.length; i++) values[i] = values[i + count];
    values.resize(values.length - count);
}

static void move_with_visited(MoveAlongSurfaceResult &moved, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                              Vec3 start_position, Vec3 end_position, const QueryFilter &filter) {
    Array<NodeRef> &visited = workspace.visited;
    nav_mesh_search::move_along_surface(moved, workspace, nav_mesh, start_node_ref, start_position, end_position, filter, visited);
    if (moved.truncated) {
        visited.reserve(moved.visited_total);
        nav_mesh_search::move_along_surface(moved, workspace, nav_mesh, start_node_ref, start_position, end_position, filter, visited);
    }
    visited.resize(moved.visited_count);
}

void find_smooth_path(FindSmoothPathResult &result, QueryWorkspace &workspace, const NavMesh &nav_mesh, Vec3 start, Vec3 end, Vec3 half_extents,
                      const QueryFilter &filter, Span<SmoothPathPoint> point_buffer, const FindSmoothPathOptions &options) {
    SmoothPathPoint *points = point_buffer.data;
    uint32_t max_points = point_buffer.capacity;
    result = {};
    result.flags = FIND_SMOOTH_PATH_RESULT_NONE | FIND_SMOOTH_PATH_RESULT_INVALID_INPUT;
    result.start_node_ref = INVALID_NODE_REF;
    result.end_node_ref = INVALID_NODE_REF;
    result.node_path_flags = FIND_NODE_PATH_RESULT_NONE;
    if (max_points == 0) return;
    uint32_t &point_count = result.point_count;

    FindNearestPolyResult start_nearest;
    nav_mesh_api::find_nearest_poly(start_nearest, nav_mesh, start, half_extents, filter);
    if (!start_nearest.success) return;
    result.start_position = start_nearest.position;
    result.start_node_ref = start_nearest.node_ref;

    FindNearestPolyResult end_nearest;
    nav_mesh_api::find_nearest_poly(end_nearest, nav_mesh, end, half_extents, filter);
    if (!end_nearest.success) return;
    result.end_position = end_nearest.position;
    result.end_node_ref = end_nearest.node_ref;

    FindNodePathResult node_path;
    Array<NodeRef> &polys = workspace.node_path;
    nav_mesh_search::find_node_path(node_path, workspace, nav_mesh, result.start_node_ref, result.end_node_ref, result.start_position,
                                    result.end_position, filter, polys, {options.raycast_distance});
    if (node_path.truncated) {
        polys.reserve(node_path.path_total);
        nav_mesh_search::copy_last_node_path(node_path, workspace, polys);
    }
    polys.resize(node_path.path_count);
    result.node_path_flags = node_path.flags;
    result.node_path_total = polys.length;
    result.node_path_count = polys.length < options.node_path.capacity ? polys.length : options.node_path.capacity;
    for (uint32_t i = 0; i < result.node_path_count; i++) options.node_path.data[i] = polys[i];

    if (!node_path.success || polys.length == 0) {
        result.flags = FIND_SMOOTH_PATH_RESULT_FIND_NODE_PATH_FAILED;
        return;
    }

    Vec3 iteration_position = result.start_position;
    Vec3 target_position = result.end_position;

    points[point_count++] = {iteration_position, NODE_TYPE_POLY, result.start_node_ref, STRAIGHT_PATH_POINT_START};

    while (polys.length > 0 && point_count < max_points) {
        SteerTarget steer_target = get_steer_target(nav_mesh, iteration_position, target_position, options.slop, polys);
        if (!steer_target.success) break;

        bool is_end_of_path = (steer_target.steer_position_flags & STRAIGHT_PATH_POINT_END) != 0;
        bool is_off_mesh_connection = (steer_target.steer_position_flags & STRAIGHT_PATH_POINT_OFFMESH) != 0;

        Vec3 delta = vec3::subtract(steer_target.steer_position, iteration_position);
        float length = vec3::length(delta);
        length = length > 0 ? fminf(options.step_size, length) / length : 0;

        Vec3 move_target = vec3::scale_and_add(iteration_position, delta, length);

        MoveAlongSurfaceResult moved;
        move_with_visited(moved, workspace, nav_mesh, polys[0], iteration_position, move_target, filter);
        if (!moved.success) break;

        merge_corridor_start_moved(polys, workspace.visited, 256);
        fixup_shortcuts(polys, nav_mesh);

        iteration_position = moved.position;

        if (is_end_of_path && in_range(iteration_position, steer_target.steer_position, options.slop, 1.0f)) {
            iteration_position = target_position;
            if (point_count < max_points) {
                points[point_count++] = {iteration_position, NODE_TYPE_POLY, result.end_node_ref, STRAIGHT_PATH_POINT_END};
            }
            break;
        } else if (is_off_mesh_connection && in_range(iteration_position, steer_target.steer_position, options.slop, 1.0f)) {
            NodeRef off_mesh_connection_ref = steer_target.steer_position_ref;

            NodeRef previous_node_ref = INVALID_NODE_REF;
            NodeRef node_ref = polys[0];
            uint32_t npos = 0;

            while (npos < polys.length && node_ref != off_mesh_connection_ref) {
                previous_node_ref = node_ref;
                node_ref = polys[npos];
                npos++;
            }

            remove_front(polys, npos);

            const NavMeshNode &off_mesh_node = nav_mesh_api::get_node_by_ref(nav_mesh, off_mesh_connection_ref);
            int32_t connection_id = off_mesh_node.off_mesh_connection_id;
            bool has_connection = connection_id >= 0 && (uint32_t)connection_id < nav_mesh.off_mesh_connections.length &&
                                  nav_mesh.off_mesh_connections[(uint32_t)connection_id].allocated;

            if (has_connection && previous_node_ref != INVALID_NODE_REF) {
                OffMeshCrossing crossing;
                nav_mesh_api::get_off_mesh_crossing(crossing, nav_mesh, previous_node_ref, off_mesh_connection_ref, iteration_position);

                if (point_count < max_points && crossing.success) {
                    points[point_count++] = {iteration_position, NODE_TYPE_OFFMESH, off_mesh_connection_ref, STRAIGHT_PATH_POINT_OFFMESH};
                    iteration_position = crossing.end;
                }
            }
        }

        if (point_count < max_points) {
            NodeRef current_node_ref = polys.length > 0 ? polys[0] : result.end_node_ref;
            points[point_count++] = {iteration_position, NODE_TYPE_POLY, current_node_ref, 0};
        }
    }

    uint32_t flags = FIND_SMOOTH_PATH_RESULT_SUCCESS;
    if (node_path.flags & FIND_NODE_PATH_RESULT_COMPLETE_PATH) {
        flags |= FIND_SMOOTH_PATH_RESULT_COMPLETE_PATH;
    } else if (node_path.flags & FIND_NODE_PATH_RESULT_PARTIAL_PATH) {
        flags |= FIND_SMOOTH_PATH_RESULT_PARTIAL_PATH;
    }

    result.success = true;
    result.flags = flags;
    bool ended = point_count > 0 && (points[point_count - 1].flags & STRAIGHT_PATH_POINT_END) != 0;
    result.truncated = (point_count == max_points && !ended) || result.node_path_count < result.node_path_total;
}


}

}
