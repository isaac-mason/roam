#include "roam/blocks/agents/path_corridor.h"

#include <math.h>

#include "roam/query/nav_mesh_search.h"

namespace roam {

namespace path_corridor {

static const float MIN_TARGET_DISTANCE = 0.01f;

PathCorridor create() { return {}; }

void release(PathCorridor &corridor) { corridor.path.release(); }

void reset(PathCorridor &corridor, NodeRef ref, Vec3 position) {
    corridor.position = position;
    corridor.target = position;
    corridor.path.clear();
    corridor.path.push(ref);
}

void set_path(PathCorridor &corridor, Vec3 target, const NodeRef *path, uint32_t path_count) {
    corridor.target = target;
    corridor.path.clear();
    for (uint32_t i = 0; i < path_count; i++) corridor.path.push(path[i]);
}

static bool find_furthest_common(const Array<NodeRef> &current_path, const Array<NodeRef> &visited, int &furthest_path, int &furthest_visited) {
    for (int i = (int)current_path.length - 1; i >= 0; i--) {
        for (int j = (int)visited.length - 1; j >= 0; j--) {
            if (current_path[(uint32_t)i] == visited[(uint32_t)j]) {
                furthest_path = i;
                furthest_visited = j;
                return true;
            }
        }
    }
    return false;
}

static void replace(Array<NodeRef> &current_path, Array<NodeRef> &merged) {
    current_path.release();
    current_path = merged;
}

void merge_start_moved(Array<NodeRef> &current_path, const Array<NodeRef> &visited) {
    if (visited.length == 0) return;

    int furthest_path = -1;
    int furthest_visited = -1;
    if (!find_furthest_common(current_path, visited, furthest_path, furthest_visited)) return;

    uint32_t required = visited.length - (uint32_t)furthest_visited;
    uint32_t original = (uint32_t)furthest_path + 1 < current_path.length ? (uint32_t)furthest_path + 1 : current_path.length;

    Array<NodeRef> merged = {};
    for (uint32_t i = 0; i < required; i++) merged.push(visited[visited.length - 1 - i]);
    for (uint32_t i = original; i < current_path.length; i++) merged.push(current_path[i]);
    replace(current_path, merged);
}

void merge_start_shortcut(Array<NodeRef> &current_path, const Array<NodeRef> &visited) {
    if (visited.length == 0) return;

    int furthest_path = -1;
    int furthest_visited = -1;
    if (!find_furthest_common(current_path, visited, furthest_path, furthest_visited)) return;
    if (furthest_visited <= 0) return;

    Array<NodeRef> merged = {};
    for (uint32_t i = 0; i < (uint32_t)furthest_visited; i++) merged.push(visited[i]);
    for (uint32_t i = (uint32_t)furthest_path; i < current_path.length; i++) merged.push(current_path[i]);
    replace(current_path, merged);
}

bool move_position(PathCorridor &corridor, Vec3 new_position, const NavMesh &nav_mesh, const QueryFilter &filter, SearchWorkspace *workspace) {
    if (corridor.path.length == 0) return false;

    MoveAlongSurfaceResult result = nav_mesh_search::move_along_surface(nav_mesh, corridor.path[0], corridor.position, new_position, filter, workspace);
    bool moved = result.success;
    if (moved) {
        merge_start_moved(corridor.path, result.visited);
        corridor.position = result.position;
    }

    nav_mesh_search::release(result);
    return moved;
}

FindCornersResult find_corners(const PathCorridor &corridor, const NavMesh &nav_mesh, int max_corners) {
    FindCornersResult result = {};
    if (corridor.path.length == 0) return result;

    FindStraightPathResult straight_path =
        find_straight_path::find_straight_path(nav_mesh, corridor.position, corridor.target, corridor.path.data, corridor.path.length, max_corners);

    if (!straight_path.success || straight_path.path.length == 0) {
        find_straight_path::release(straight_path);
        return result;
    }

    Array<StraightPathPoint> &points = straight_path.path;

    uint32_t first = 0;
    while (first < points.length) {
        const StraightPathPoint &corner = points[first];
        if (corner.type == NODE_TYPE_OFFMESH || vec3::distance(corridor.position, corner.position) > MIN_TARGET_DISTANCE) break;
        first++;
    }

    uint32_t end = points.length;
    for (uint32_t i = first; i < points.length; i++) {
        if (points[i].type == NODE_TYPE_OFFMESH) {
            end = i + 1;
            break;
        }
    }

    result.success = true;
    for (uint32_t i = first; i < end; i++) result.corners.push(points[i]);
    find_straight_path::release(straight_path);
    return result;
}

void release(FindCornersResult &result) { result.corners.release(); }

bool corridor_is_valid(const PathCorridor &corridor, uint32_t max_look_ahead, const NavMesh &nav_mesh, const QueryFilter &filter) {
    uint32_t count = corridor.path.length < max_look_ahead ? corridor.path.length : max_look_ahead;
    for (uint32_t i = 0; i < count; i++) {
        NodeRef node_ref = corridor.path[i];
        if (!nav_mesh_api::is_valid_node_ref(nav_mesh, node_ref) || !filter.pass_filter(filter, node_ref, nav_mesh)) return false;
    }
    return true;
}

bool fix_path_start(PathCorridor &corridor, NodeRef safe_ref, Vec3 safe_position) {
    corridor.position = safe_position;

    if (corridor.path.length < 3 && corridor.path.length > 0) {
        NodeRef last = corridor.path.back();
        corridor.path.resize(3);
        corridor.path[2] = last;
    } else if (corridor.path.length == 0) {
        corridor.path.resize(2);
    }

    corridor.path[0] = safe_ref;
    corridor.path[1] = INVALID_NODE_REF;
    return true;
}

MoveOverOffMeshConnectionResult move_over_off_mesh_connection(PathCorridor &corridor, NodeRef off_mesh_node_ref, const NavMesh &nav_mesh) {
    MoveOverOffMeshConnectionResult result = {};
    if (corridor.path.length == 0) return result;

    bool has_previous = false;
    NodeRef previous = INVALID_NODE_REF;
    NodeRef node_ref = corridor.path[0];
    uint32_t i = 0;

    while (i < corridor.path.length && node_ref != off_mesh_node_ref) {
        previous = node_ref;
        has_previous = true;
        i++;
        if (i < corridor.path.length) node_ref = corridor.path[i];
    }

    if (i == corridor.path.length) return result;

    uint32_t remaining = corridor.path.length - (i + 1);
    for (uint32_t k = 0; k < remaining; k++) corridor.path[k] = corridor.path[i + 1 + k];
    corridor.path.resize(remaining);

    if (!has_previous) return result;

    int32_t connection_id = nav_mesh_api::get_node_by_ref(nav_mesh, off_mesh_node_ref).off_mesh_connection_id;
    if (connection_id < 0 || (uint32_t)connection_id >= nav_mesh.off_mesh_connections.length) return result;

    const OffMeshConnection &connection = nav_mesh.off_mesh_connections[(uint32_t)connection_id];
    const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[(uint32_t)connection_id];
    if (!connection.allocated || !attachment.attached) return result;

    bool on_start = attachment.start_poly_node == previous;
    Vec3 end_position = on_start ? connection.end : connection.start;

    corridor.position = end_position;

    result.success = true;
    result.start_position = on_start ? connection.start : connection.end;
    result.end_position = end_position;
    result.end_node_ref = on_start ? attachment.end_poly_node : attachment.start_poly_node;
    result.prev_node_ref = previous;
    result.off_mesh_node_ref = off_mesh_node_ref;
    return result;
}

bool optimize_path_topology(PathCorridor &corridor, const NavMesh &nav_mesh, const QueryFilter &filter) {
    if (corridor.path.length < 3) return false;

    const int MAX_ITERATIONS = 32;

    SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();
    nav_mesh_search::init_sliced_find_node_path(nav_mesh, query, corridor.path[0], corridor.path.back(), corridor.position, corridor.target, filter);
    nav_mesh_search::update_sliced_find_node_path(nav_mesh, query, MAX_ITERATIONS);
    SlicedFindNodePathResult result =
        nav_mesh_search::finalize_sliced_find_node_path_partial(nav_mesh, query, corridor.path.data, corridor.path.length);

    bool optimized = (result.status & SLICED_FIND_NODE_PATH_SUCCESS) != 0 && result.path.length > 0;
    if (optimized) merge_start_shortcut(corridor.path, result.path);

    nav_mesh_search::release(result);
    nav_mesh_search::release(query);
    return optimized;
}

void optimize_path_visibility(PathCorridor &corridor, Vec3 next, float path_optimization_range, const NavMesh &nav_mesh, const QueryFilter &filter,
                              SearchWorkspace *workspace) {
    if (corridor.path.length == 0) return;

    float dx = next.x - corridor.position.x;
    float dz = next.z - corridor.position.z;
    float distance = sqrtf(dx * dx + dz * dz);
    if (distance < 0.01f) return;

    distance = fminf(distance + 0.01f, path_optimization_range);

    Vec3 delta = vec3::subtract(next, corridor.position);
    Vec3 goal = vec3::scale_and_add(corridor.position, delta, path_optimization_range / distance);

    RaycastResult result = nav_mesh_search::raycast(nav_mesh, corridor.path[0], corridor.position, goal, filter, workspace);
    if (result.path.length > 1 && result.t > 0.99f) merge_start_shortcut(corridor.path, result.path);
    nav_mesh_search::release(result);
}

}

}
