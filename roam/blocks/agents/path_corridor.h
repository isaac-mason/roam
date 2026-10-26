#pragma once

#include <stdint.h>

#include "roam/math/vec3.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"
#include "roam/util/array.h"

namespace roam {

struct PathCorridor {
    Vec3 position;
    Vec3 target;
    Array<NodeRef> path;
    Array<NodeRef> scratch;
};

struct FindCornersResult {
    bool success;
    uint32_t corner_count;
    bool truncated;
};

struct MoveOverOffMeshConnectionResult {
    bool success;
    Vec3 start_position;
    Vec3 end_position;
    NodeRef end_node_ref;
    NodeRef prev_node_ref;
    NodeRef off_mesh_node_ref;
    float t;
};

namespace path_corridor {

PathCorridor create();
void release(PathCorridor &corridor);

void reset(PathCorridor &corridor, NodeRef ref, Vec3 position);
void set_path(PathCorridor &corridor, Vec3 target, Span<const NodeRef> path);

void merge_start_moved(Array<NodeRef> &current_path, const Array<NodeRef> &visited);
void merge_start_shortcut(Array<NodeRef> &current_path, const Array<NodeRef> &visited);

bool move_position(PathCorridor &corridor, QueryWorkspace &workspace, Vec3 new_position, const NavMesh &nav_mesh, const QueryFilter &filter);

void find_corners(FindCornersResult &out, const PathCorridor &corridor, const NavMesh &nav_mesh, Span<StraightPathPoint> corners);

bool corridor_is_valid(const PathCorridor &corridor, uint32_t max_look_ahead, const NavMesh &nav_mesh, const QueryFilter &filter);

bool fix_path_start(PathCorridor &corridor, NodeRef safe_ref, Vec3 safe_position);

void move_over_off_mesh_connection(MoveOverOffMeshConnectionResult &out, PathCorridor &corridor, NodeRef off_mesh_node_ref, const NavMesh &nav_mesh);

bool optimize_path_topology(PathCorridor &corridor, SlicedNodePathQuery &query, const NavMesh &nav_mesh, const QueryFilter &filter);

void optimize_path_visibility(PathCorridor &corridor, Vec3 next, float path_optimization_range, const NavMesh &nav_mesh, const QueryFilter &filter);

}

}
