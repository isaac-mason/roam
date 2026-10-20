#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/util/array.h"
#include "roam/util/map.h"

namespace roam {

constexpr uint32_t NODE_FLAG_OPEN = 0x01;
constexpr uint32_t NODE_FLAG_CLOSED = 0x02;
constexpr uint32_t NODE_FLAG_PARENT_DETACHED = 0x04;

constexpr int NO_PARENT_STATE = -1;
constexpr int32_t NULL_SEARCH_NODE = -1;

struct SearchNode {
    Vec3 position;
    float cost;
    float total;
    NodeRef parent_node_ref;
    int parent_state;
    int state;
    uint32_t flags;
    NodeRef node_ref;
    int32_t next_in_bucket;
};

struct SearchNodePool {
    Array<SearchNode> nodes;
    Array<int32_t> buckets;
};

struct SearchNodeQueue {
    Array<uint32_t> items;
};

struct SearchWorkspace {
    SearchNodePool nodes;
    SearchNodeQueue open_list;
    SearchNodePool local_nodes;
    Array<uint32_t> local_queue;
    Array<float> vertices;
    Array<float> other_vertices;
    Array<float> raycast_vertices;
    Array<NodeRef> neighbours;
};

enum FindNodePathResultFlags : uint32_t {
    FIND_NODE_PATH_RESULT_NONE = 0,
    FIND_NODE_PATH_RESULT_SUCCESS = 1 << 0,
    FIND_NODE_PATH_RESULT_COMPLETE_PATH = 1 << 1,
    FIND_NODE_PATH_RESULT_PARTIAL_PATH = 1 << 2,
    FIND_NODE_PATH_RESULT_INVALID_INPUT = 1 << 3,
};

struct FindNodePathResult {
    bool success;
    uint32_t flags;
    Array<NodeRef> path;
};

struct FindNodePathOptions {
    float raycast_distance;
};

enum SlicedFindNodePathStatusFlags : uint32_t {
    SLICED_FIND_NODE_PATH_NOT_INITIALIZED = 0,
    SLICED_FIND_NODE_PATH_IN_PROGRESS = 1,
    SLICED_FIND_NODE_PATH_SUCCESS = 2,
    SLICED_FIND_NODE_PATH_PARTIAL_RESULT = 4,
    SLICED_FIND_NODE_PATH_FAILURE = 8,
    SLICED_FIND_NODE_PATH_INVALID_PARAM = 16,
};

enum SlicedFindNodePathInitFlags : uint32_t {
    SLICED_FIND_NODE_PATH_ANY_ANGLE = 1,
};

struct SlicedNodePathQuery {
    uint32_t status;
    NodeRef start_node_ref;
    NodeRef end_node_ref;
    Vec3 start_position;
    Vec3 end_position;
    QueryFilter filter;
    SearchNodePool nodes;
    SearchNodeQueue open_list;
    int32_t last_best_node;
    float last_best_node_cost;
    bool has_raycast_limit;
    float raycast_limit_sqr;
    Array<float> raycast_vertices;
};

struct SlicedFindNodePathResult {
    uint32_t status;
    Array<NodeRef> path;
};

struct MoveAlongSurfaceResult {
    bool success;
    Vec3 position;
    NodeRef node_ref;
    Array<NodeRef> visited;
};

struct RaycastResult {
    float t;
    Vec3 hit_normal;
    int hit_edge_index;
    Array<NodeRef> path;
    float path_cost;
};

struct FindRandomPointResult {
    bool success;
    NodeRef node_ref;
    Vec3 position;
};

using RandomFunction = float (*)(void *user_data);

namespace nav_mesh_search {

SearchWorkspace create_search_workspace();
void release(SearchWorkspace &workspace);

int32_t get_search_node(const SearchNodePool &pool, NodeRef node_ref, int state);
int32_t get_first_search_node(const SearchNodePool &pool, NodeRef node_ref);
uint32_t add_search_node(SearchNodePool &pool, const SearchNode &node);
void clear(SearchNodePool &pool);
void release(SearchNodePool &pool);

void push_node_to_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t node);
int32_t pop_node_from_queue(SearchNodeQueue &queue, const SearchNodePool &pool);
void reindex_node_in_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t node);
void release(SearchNodeQueue &queue);

bool get_portal_points(const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref, Vec3 &out_left, Vec3 &out_right);
bool get_edge_mid_point(const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref, Vec3 &out_mid_point);

FindNodePathResult find_node_path(const NavMesh &nav_mesh, NodeRef start_node_ref, NodeRef end_node_ref, Vec3 start_position, Vec3 end_position,
                                  const QueryFilter &filter, FindNodePathOptions options = {}, SearchWorkspace *workspace = nullptr);
void release(FindNodePathResult &result);

SlicedNodePathQuery create_sliced_node_path_query();
uint32_t init_sliced_find_node_path(const NavMesh &nav_mesh, SlicedNodePathQuery &query, NodeRef start_node_ref, NodeRef end_node_ref,
                                    Vec3 start_position, Vec3 end_position, const QueryFilter &filter, uint32_t flags = 0);
int update_sliced_find_node_path(const NavMesh &nav_mesh, SlicedNodePathQuery &query, int max_iterations);
SlicedFindNodePathResult finalize_sliced_find_node_path(const NavMesh &nav_mesh, SlicedNodePathQuery &query);
SlicedFindNodePathResult finalize_sliced_find_node_path_partial(const NavMesh &nav_mesh, SlicedNodePathQuery &query, const NodeRef *existing_path,
                                                                uint32_t existing_path_count);
void release(SlicedNodePathQuery &query);
void release(SlicedFindNodePathResult &result);

MoveAlongSurfaceResult move_along_surface(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position,
                                          const QueryFilter &filter, SearchWorkspace *workspace = nullptr);
void release(MoveAlongSurfaceResult &result);

RaycastResult raycast(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
                      SearchWorkspace *workspace = nullptr);
RaycastResult raycast_with_costs(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
                                 NodeRef previous_ref, SearchWorkspace *workspace = nullptr);
void release(RaycastResult &result);

FindRandomPointResult find_random_point(const NavMesh &nav_mesh, const QueryFilter &filter, RandomFunction random, void *random_user_data);
FindRandomPointResult find_random_point_around_circle(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 position, float max_radius,
                                                      const QueryFilter &filter, RandomFunction random, void *random_user_data,
                                                      SearchWorkspace *workspace = nullptr);

}

}
