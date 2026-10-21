#pragma once

#include <stdint.h>

#include "roam/blocks/agents/local_boundary.h"
#include "roam/blocks/agents/obstacle_avoidance.h"
#include "roam/blocks/agents/path_corridor.h"
#include "roam/math/vec3.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"
#include "roam/util/array.h"
#include "roam/util/index_pool.h"

namespace roam {

enum AgentState : uint8_t {
    AGENT_STATE_INVALID,
    AGENT_STATE_WALKING,
    AGENT_STATE_OFFMESH,
};

enum AgentTargetState : uint8_t {
    AGENT_TARGET_STATE_NONE,
    AGENT_TARGET_STATE_FAILED,
    AGENT_TARGET_STATE_VALID,
    AGENT_TARGET_STATE_REQUESTING,
    AGENT_TARGET_STATE_WAITING_FOR_QUEUE,
    AGENT_TARGET_STATE_WAITING_FOR_PATH,
    AGENT_TARGET_STATE_VELOCITY,
};

enum CrowdUpdateFlags : uint32_t {
    CROWD_UPDATE_ANTICIPATE_TURNS = 1,
    CROWD_UPDATE_OBSTACLE_AVOIDANCE = 2,
    CROWD_UPDATE_SEPARATION = 4,
    CROWD_UPDATE_OPTIMIZE_VIS = 8,
    CROWD_UPDATE_OPTIMIZE_TOPO = 16,
};

struct AgentParams {
    float radius;
    float height;
    float max_acceleration;
    float max_speed;
    float collision_query_range;
    float path_optimization_range = 0;
    float separation_weight;
    uint32_t update_flags;
    QueryFilter query_filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    ObstacleAvoidanceParams obstacle_avoidance = DEFAULT_OBSTACLE_AVOIDANCE_PARAMS;
    bool auto_traverse_off_mesh_connections = true;
    bool debug_obstacle_avoidance = false;
};

struct AgentNeighbour {
    uint32_t agent_id;
    float distance_sqr;
};

struct OffMeshAnimation {
    bool active;
    float t;
    Vec3 start_position;
    Vec3 end_position;
    NodeRef node_ref;
    float duration;
};

struct Agent {
    bool allocated;

    float radius;
    float height;
    float max_acceleration;
    float max_speed;
    float collision_query_range;
    float path_optimization_range;
    float separation_weight;
    uint32_t update_flags;
    QueryFilter query_filter;
    ObstacleAvoidanceParams obstacle_avoidance;
    bool auto_traverse_off_mesh_connections;
    bool debug_obstacle_avoidance;

    AgentState state;
    PathCorridor corridor;
    LocalBoundary boundary;
    SlicedNodePathQuery sliced_query;
    ObstacleAvoidanceQuery obstacle_avoidance_query;
    ObstacleAvoidanceDebugData obstacle_avoidance_debug_data;

    Array<AgentNeighbour> neis;
    Array<StraightPathPoint> corners;

    Vec3 position;
    float desired_speed;
    Vec3 desired_velocity;
    Vec3 new_velocity;
    Vec3 velocity;
    Vec3 displacement;

    AgentTargetState target_state;
    NodeRef target_ref;
    Vec3 target_position;
    bool target_replan;
    float target_pathfinding_time;
    bool target_path_is_partial;

    float topology_opt_time;

    OffMeshAnimation off_mesh_animation;
};

struct Crowd {
    Array<Agent> agents;
    IndexPool agent_ids;
    float max_agent_radius;
    Vec3 agent_placement_half_extents;
    int max_iterations_per_update;
    int max_iterations_per_agent;
    int quick_search_iterations;
    uint32_t max_neighbours;

    SearchWorkspace workspace;
    Array<uint32_t> scratch_agents;
    Array<uint32_t> scratch_cells;
    Array<uint32_t> scratch_cell_starts;
    Array<uint32_t> scratch_cell_agents;
    SlicedNodePathQuery topology_query;
    Array<NodeRef> scratch_merged_path;
};

namespace crowd {

Crowd create(float max_agent_radius);
void release(Crowd &crowd);

uint32_t add_agent(Crowd &crowd, const NavMesh &nav_mesh, Vec3 position, const AgentParams &params);
bool remove_agent(Crowd &crowd, uint32_t agent_id);
bool is_agent(const Crowd &crowd, uint32_t agent_id);

bool request_move_target(Crowd &crowd, uint32_t agent_id, NodeRef target_ref, Vec3 target_position);
bool request_move_velocity(Crowd &crowd, uint32_t agent_id, Vec3 velocity);
bool reset_move_target(Crowd &crowd, uint32_t agent_id);

bool complete_off_mesh_connection(Crowd &crowd, uint32_t agent_id);

void update(Crowd &crowd, const NavMesh &nav_mesh, float delta_time);

bool is_agent_at_target(const Crowd &crowd, uint32_t agent_id, float threshold);

}

}
