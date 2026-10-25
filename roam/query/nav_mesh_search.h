#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/util/array.h"
#include "roam/util/map.h"
#include "roam/util/span.h"

namespace roam {

constexpr uint32_t NODE_FLAG_OPEN = 0x01;
constexpr uint32_t NODE_FLAG_CLOSED = 0x02;
constexpr uint32_t NODE_FLAG_PARENT_DETACHED = 0x04;

constexpr int NO_PARENT_STATE = -1;
constexpr int32_t NULL_SEARCH_NODE = -1;

struct SearchNode {
    NodeRef node_ref;
    NodeRef parent_node_ref;
    Vec3 position;
    float cost;
    float total;
    int32_t next_with_same_ref;
    uint8_t flags;
    int8_t state;
    int8_t parent_state;
};

static_assert(sizeof(SearchNode) == 48);

struct SearchNodePool {
    Array<SearchNode> nodes;
    Array<int32_t> first_by_index;
    Array<uint32_t> stamp_by_index;
    uint32_t stamp;
};

struct SearchNodeQueue {
    Array<uint32_t> items;
};

enum FindNodePathResultFlags : uint32_t {
    FIND_NODE_PATH_RESULT_NONE = 0,
    FIND_NODE_PATH_RESULT_SUCCESS = 1 << 0,
    FIND_NODE_PATH_RESULT_COMPLETE_PATH = 1 << 1,
    FIND_NODE_PATH_RESULT_PARTIAL_PATH = 1 << 2,
    FIND_NODE_PATH_RESULT_INVALID_INPUT = 1 << 3,
};

constexpr uint32_t INITIAL_PATH_CAPACITY = 256;

struct FindNodePathResult {
    bool success;
    uint32_t flags;
    uint32_t path_count;
    uint32_t path_total;
    bool truncated;
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

struct RaycastResult {
    float t;
    Vec3 hit_normal;
    int hit_edge_index;
    float path_cost;
    uint32_t path_count;
    uint32_t path_total;
    bool truncated;
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
    Array<uint32_t> path_nodes;
    uint32_t finalized_status;
};

struct SlicedFindNodePathResult {
    uint32_t status;
    uint32_t path_count;
    uint32_t path_total;
    bool truncated;
};

struct MoveAlongSurfaceResult {
    bool success;
    Vec3 position;
    NodeRef node_ref;
    uint32_t visited_count;
    uint32_t visited_total;
    bool truncated;
};

struct GetPortalPointsResult {
    bool success;
    Vec3 left;
    Vec3 right;
};

struct GetEdgeMidPointResult {
    bool success;
    Vec3 position;
};

struct QueryWorkspace {
    SearchNodePool nodes;
    SearchNodeQueue open_list;
    SearchNodePool local_nodes;
    Array<uint32_t> local_queue;
    Array<float> vertices;
    Array<float> other_vertices;
    Array<NodeRef> neighbours;
    Array<NodeRef> node_path;
    Array<NodeRef> visited;
    Array<NodeRef> local_refs;
    Array<uint32_t> cluster_stamps;
    Array<uint32_t> cluster_queue;
    uint32_t cluster_stamp;
    int32_t last_path_node = NULL_SEARCH_NODE;
};

struct FindRandomPointResult {
    bool success;
    NodeRef node_ref;
    Vec3 position;
};

using RandomFunction = float (*)(void *user_data);

namespace nav_mesh_search {

QueryWorkspace create_query_workspace();
void release(QueryWorkspace &workspace);
uint32_t get_search_node_count(const QueryWorkspace &workspace);

int32_t get_search_node(const SearchNodePool &pool, NodeRef node_ref, int state);
int32_t get_first_search_node(const SearchNodePool &pool, NodeRef node_ref);
uint32_t add_search_node(SearchNodePool &pool, const SearchNode &node);
void clear(SearchNodePool &pool);
void release(SearchNodePool &pool);

void push_node_to_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t node);
int32_t pop_node_from_queue(SearchNodeQueue &queue, const SearchNodePool &pool);
void reindex_node_in_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t node);
void release(SearchNodeQueue &queue);

void get_portal_points(GetPortalPointsResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref);
void get_portal_points_unchecked(GetPortalPointsResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref);
void get_edge_mid_point(GetEdgeMidPointResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref);

void copy_last_node_path(FindNodePathResult &out, const QueryWorkspace &workspace, Span<NodeRef> path);
void find_node_path(FindNodePathResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref, NodeRef end_node_ref,
                    Vec3 start_position, Vec3 end_position, const QueryFilter &filter, Span<NodeRef> path, FindNodePathOptions options = {});

SlicedNodePathQuery create_sliced_node_path_query();
void init_sliced_find_node_path(SlicedNodePathQuery &query, const NavMesh &nav_mesh, NodeRef start_node_ref, NodeRef end_node_ref,
                                    Vec3 start_position, Vec3 end_position, const QueryFilter &filter, uint32_t flags = 0);
int update_sliced_find_node_path(SlicedNodePathQuery &query, const NavMesh &nav_mesh, int max_iterations);
void finalize_sliced_find_node_path(SlicedFindNodePathResult &out, const NavMesh &nav_mesh, SlicedNodePathQuery &query, Span<NodeRef> path);
void finalize_sliced_find_node_path_partial(SlicedFindNodePathResult &out, const NavMesh &nav_mesh, SlicedNodePathQuery &query,
                                            Span<const NodeRef> existing_path, Span<NodeRef> path);
void release(SlicedNodePathQuery &query);

void move_along_surface(MoveAlongSurfaceResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                        Vec3 start_position, Vec3 end_position, const QueryFilter &filter, Span<NodeRef> visited);

void raycast(RaycastResult &out, const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
             Span<NodeRef> path);
void raycast_with_costs(RaycastResult &out, const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position,
                        const QueryFilter &filter, NodeRef previous_ref, Span<NodeRef> path);

bool is_reachable(QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref);

void find_random_point(FindRandomPointResult &out, const NavMesh &nav_mesh, const QueryFilter &filter, RandomFunction random, void *random_user_data);
void find_random_point_around_circle(FindRandomPointResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                                     Vec3 position, float max_radius, const QueryFilter &filter, RandomFunction random, void *random_user_data);

}

}
