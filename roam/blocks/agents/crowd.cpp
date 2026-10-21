#include "roam/blocks/agents/crowd.h"

#include <math.h>

#include "roam/math/vec2.h"
#include "roam/util/sort.h"

namespace roam {

namespace crowd {

static const uint32_t CHECK_LOOKAHEAD = 10;
static const float TARGET_REPLAN_DELAY_SECONDS = 1.0f;
static const uint32_t MAX_CORNERS = 3;

Crowd create(float max_agent_radius) {
    Crowd crowd = {};
    crowd.max_agent_radius = max_agent_radius;
    crowd.agent_placement_half_extents = {max_agent_radius, max_agent_radius, max_agent_radius};
    crowd.max_iterations_per_update = 600;
    crowd.max_iterations_per_agent = 200;
    crowd.quick_search_iterations = 20;
    crowd.workspace = nav_mesh_search::create_search_workspace();
    crowd.topology_query = nav_mesh_search::create_sliced_node_path_query();
    return crowd;
}

static void release_agent(Agent &agent) {
    path_corridor::release(agent.corridor);
    nav_mesh_search::release(agent.sliced_query);
    obstacle_avoidance::release(agent.obstacle_avoidance_query);
    obstacle_avoidance::release(agent.obstacle_avoidance_debug_data);
    agent.neis.release();
    agent.corners.release();
    agent.allocated = false;
}

void release(Crowd &crowd) {
    for (Agent &agent : crowd.agents) {
        if (agent.allocated) release_agent(agent);
    }
    crowd.agents.release();
    index_pool::release(crowd.agent_ids);
    nav_mesh_search::release(crowd.workspace);
    crowd.scratch_agents.release();
    crowd.scratch_cells.release();
    crowd.scratch_cell_starts.release();
    crowd.scratch_cell_agents.release();
    nav_mesh_search::release(crowd.topology_query);
    crowd.scratch_merged_path.release();
}

static Agent *get_agent(Crowd &crowd, uint32_t agent_id) {
    if (agent_id >= crowd.agents.length || !crowd.agents[agent_id].allocated) return nullptr;
    return &crowd.agents[agent_id];
}

bool is_agent(const Crowd &crowd, uint32_t agent_id) { return agent_id < crowd.agents.length && crowd.agents[agent_id].allocated; }

uint32_t add_agent(Crowd &crowd, const NavMesh &nav_mesh, Vec3 position, const AgentParams &params) {
    uint32_t agent_id = index_pool::request(crowd.agent_ids);
    while (crowd.agents.length <= agent_id) crowd.agents.push({});

    Agent &agent = crowd.agents[agent_id];
    agent = {};
    agent.allocated = true;

    agent.radius = params.radius;
    agent.height = params.height;
    agent.max_acceleration = params.max_acceleration;
    agent.max_speed = params.max_speed;
    agent.collision_query_range = params.collision_query_range;
    agent.path_optimization_range = params.path_optimization_range > 0 ? params.path_optimization_range : params.radius * 30.0f;
    agent.separation_weight = params.separation_weight;
    agent.update_flags = params.update_flags;
    agent.query_filter = params.query_filter;
    agent.obstacle_avoidance = params.obstacle_avoidance;
    agent.auto_traverse_off_mesh_connections = params.auto_traverse_off_mesh_connections;
    agent.debug_obstacle_avoidance = params.debug_obstacle_avoidance;

    agent.state = AGENT_STATE_WALKING;
    agent.corridor = path_corridor::create();
    agent.sliced_query = nav_mesh_search::create_sliced_node_path_query();
    agent.boundary = local_boundary::create();
    agent.obstacle_avoidance_query = obstacle_avoidance::create_obstacle_avoidance_query(32, 32);

    agent.target_state = AGENT_TARGET_STATE_NONE;
    agent.target_ref = INVALID_NODE_REF;

    FindNearestPolyResult nearest = {};
    nav_mesh_api::find_nearest_poly(nearest, nav_mesh, position, crowd.agent_placement_half_extents, agent.query_filter);

    Vec3 nearest_position = position;
    NodeRef nearest_ref = INVALID_NODE_REF;
    if (nearest.success) {
        nearest_position = nearest.position;
        nearest_ref = nearest.node_ref;
    }

    agent.position = nearest_position;
    path_corridor::reset(agent.corridor, nearest_ref, nearest_position);
    agent.state = nearest_ref != INVALID_NODE_REF ? AGENT_STATE_WALKING : AGENT_STATE_INVALID;

    return agent_id;
}

bool remove_agent(Crowd &crowd, uint32_t agent_id) {
    Agent *agent = get_agent(crowd, agent_id);
    if (!agent) return false;
    release_agent(*agent);
    index_pool::release_index(crowd.agent_ids, agent_id);
    return true;
}

bool request_move_target(Crowd &crowd, uint32_t agent_id, NodeRef target_ref, Vec3 target_position) {
    Agent *agent = get_agent(crowd, agent_id);
    if (!agent) return false;

    agent->target_ref = target_ref;
    agent->target_position = target_position;
    agent->target_replan = false;
    agent->target_state = AGENT_TARGET_STATE_REQUESTING;
    agent->target_path_is_partial = false;
    agent->target_pathfinding_time = 0;
    return true;
}

static void request_move_target_replan(Agent &agent, NodeRef target_ref, Vec3 target_position) {
    agent.target_ref = target_ref;
    agent.target_position = target_position;
    agent.target_replan = true;
    agent.target_state = AGENT_TARGET_STATE_REQUESTING;
}

bool request_move_velocity(Crowd &crowd, uint32_t agent_id, Vec3 velocity) {
    Agent *agent = get_agent(crowd, agent_id);
    if (!agent) return false;

    agent->target_position = velocity;
    agent->target_state = AGENT_TARGET_STATE_VELOCITY;
    agent->target_replan = false;
    agent->target_ref = INVALID_NODE_REF;
    return true;
}

bool reset_move_target(Crowd &crowd, uint32_t agent_id) {
    Agent *agent = get_agent(crowd, agent_id);
    if (!agent) return false;

    agent->target_ref = INVALID_NODE_REF;
    agent->target_position = {0, 0, 0};
    agent->desired_velocity = {0, 0, 0};
    agent->target_replan = false;
    agent->target_state = AGENT_TARGET_STATE_NONE;
    agent->target_path_is_partial = false;
    return true;
}

static NodeRef first_corridor_ref(const Agent &agent) { return agent.corridor.path.length > 0 ? agent.corridor.path[0] : INVALID_NODE_REF; }

static void check_path_validity(Crowd &crowd, const NavMesh &nav_mesh, float delta_time) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;

        agent.target_pathfinding_time += delta_time;

        bool replan = false;

        if (!nav_mesh_api::is_valid_node_ref(nav_mesh, first_corridor_ref(agent))) {
            FindNearestPolyResult nearest = {};
            nav_mesh_api::find_nearest_poly(nearest, nav_mesh, agent.position, crowd.agent_placement_half_extents, agent.query_filter);

            if (!nearest.success) {
                agent.state = AGENT_STATE_INVALID;
                path_corridor::reset(agent.corridor, INVALID_NODE_REF, agent.position);
                local_boundary::reset_local_boundary(agent.boundary);
                continue;
            }

            path_corridor::fix_path_start(agent.corridor, nearest.node_ref, nearest.position);
            local_boundary::reset_local_boundary(agent.boundary);
            agent.position = nearest.position;

            replan = true;
        }

        if (agent.target_state == AGENT_TARGET_STATE_NONE || agent.target_state == AGENT_TARGET_STATE_VELOCITY) continue;

        if (agent.target_state != AGENT_TARGET_STATE_FAILED) {
            if (!nav_mesh_api::is_valid_node_ref(nav_mesh, agent.target_ref) ||
                !agent.query_filter.pass_filter(agent.query_filter, agent.target_ref, nav_mesh)) {
                FindNearestPolyResult nearest = {};
                nav_mesh_api::find_nearest_poly(nearest, nav_mesh, agent.target_position, crowd.agent_placement_half_extents, agent.query_filter);

                if (!nearest.success) {
                    agent.target_state = AGENT_TARGET_STATE_NONE;
                    agent.target_ref = INVALID_NODE_REF;
                    path_corridor::reset(agent.corridor, INVALID_NODE_REF, agent.position);
                } else {
                    agent.target_ref = nearest.node_ref;
                    agent.target_position = nearest.position;
                    replan = true;
                }
            }
        }

        if (!path_corridor::corridor_is_valid(agent.corridor, CHECK_LOOKAHEAD, nav_mesh, agent.query_filter)) replan = true;

        if (agent.target_state == AGENT_TARGET_STATE_VALID) {
            if (agent.target_pathfinding_time > TARGET_REPLAN_DELAY_SECONDS && agent.corridor.path.length < CHECK_LOOKAHEAD &&
                agent.corridor.path.back() != agent.target_ref) {
                replan = true;
            }
        }

        if (replan && agent.target_state != AGENT_TARGET_STATE_NONE) request_move_target_replan(agent, agent.target_ref, agent.target_position);
    }
}

static void finalize_into(Array<NodeRef> &path, const NavMesh &nav_mesh, SlicedNodePathQuery &query) {
    SlicedFindNodePathResult result;
    nav_mesh_search::finalize_sliced_find_node_path(result, nav_mesh, query, path);
    if (result.truncated) {
        path.reserve(result.path_total);
        nav_mesh_search::finalize_sliced_find_node_path(result, nav_mesh, query, path);
    }
    path.resize(result.path_count);
}

static void finalize_partial_into(Array<NodeRef> &path, const NavMesh &nav_mesh, SlicedNodePathQuery &query, const Array<NodeRef> &existing) {
    SlicedFindNodePathResult result;
    nav_mesh_search::finalize_sliced_find_node_path_partial(result, nav_mesh, query, existing, path);
    if (result.truncated) {
        path.reserve(result.path_total);
        nav_mesh_search::finalize_sliced_find_node_path_partial(result, nav_mesh, query, existing, path);
    }
    path.resize(result.path_count);
}

static void request_path(Crowd &crowd, const NavMesh &nav_mesh, Agent &agent) {
    nav_mesh_search::init_sliced_find_node_path(agent.sliced_query, nav_mesh, first_corridor_ref(agent), agent.target_ref, agent.position,
                                                agent.target_position, agent.query_filter);
    nav_mesh_search::update_sliced_find_node_path(agent.sliced_query, nav_mesh, crowd.quick_search_iterations);

    Array<NodeRef> &request_path = crowd.workspace.node_path;
    if (agent.target_replan) {
        finalize_partial_into(request_path, nav_mesh, agent.sliced_query, agent.corridor.path);
    } else {
        finalize_into(request_path, nav_mesh, agent.sliced_query);
    }

    uint32_t request_path_count = request_path.length;
    Vec3 request_position = agent.target_position;

    if (request_path_count > 0) {
        if (request_path[request_path_count - 1] != agent.target_ref) {
            GetClosestPointOnPolyResult closest = {};
            nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, request_path[request_path_count - 1], agent.target_position);

            if (closest.success) {
                request_position = closest.position;
            } else {
                request_position = agent.position;
                request_path.resize(1);
                request_path[0] = first_corridor_ref(agent);
            }
        }
    } else {
        request_position = agent.position;
        request_path.push(first_corridor_ref(agent));
    }

    path_corridor::set_path(agent.corridor, request_position, request_path);
    local_boundary::reset_local_boundary(agent.boundary);
    agent.target_path_is_partial = false;

    bool reached_target = request_path_count > 0 && request_path_count - 1 < request_path.length &&
                          request_path[request_path_count - 1] == agent.target_ref;
    if (reached_target) {
        agent.target_state = AGENT_TARGET_STATE_VALID;
        agent.target_pathfinding_time = 0;
    } else {
        agent.target_state = AGENT_TARGET_STATE_WAITING_FOR_QUEUE;
    }
}

static void retry_or_fail(Agent &agent) {
    if (agent.target_ref != INVALID_NODE_REF) {
        agent.target_state = AGENT_TARGET_STATE_REQUESTING;
    } else {
        agent.target_state = AGENT_TARGET_STATE_FAILED;
        agent.target_pathfinding_time = 0;
    }
}

static void remove_trackbacks(Array<NodeRef> &path) {
    for (int j = 0; j < (int)path.length; j++) {
        if (j - 1 >= 0 && j + 1 < (int)path.length && path[(uint32_t)(j - 1)] == path[(uint32_t)(j + 1)]) {
            for (uint32_t k = (uint32_t)(j - 1); k + 2 < path.length; k++) path[k] = path[k + 2];
            path.resize(path.length - 2);
            j -= 2;
        }
    }
}

static void complete_path(Crowd &crowd, const NavMesh &nav_mesh, Agent &agent) {
    agent.target_path_is_partial = (agent.sliced_query.status & SLICED_FIND_NODE_PATH_PARTIAL_RESULT) != 0;

    finalize_into(crowd.workspace.node_path, nav_mesh, agent.sliced_query);
    const Array<NodeRef> &new_path = crowd.workspace.node_path;
    const Array<NodeRef> &current_path = agent.corridor.path;

    bool valid = !(current_path.length > 0 && new_path.length > 0 && current_path.back() != new_path[0]);
    if (!valid) {
        retry_or_fail(agent);
        return;
    }

    Array<NodeRef> &merged = crowd.scratch_merged_path;
    merged.clear();
    if (current_path.length > 1) {
        for (uint32_t i = 0; i + 1 < current_path.length; i++) merged.push(current_path[i]);
        for (NodeRef ref : new_path) merged.push(ref);
        remove_trackbacks(merged);
    } else {
        for (NodeRef ref : new_path) merged.push(ref);
    }

    Vec3 target_position = agent.target_position;
    if (merged.length > 0 && merged.back() != agent.target_ref) {
        GetClosestPointOnPolyResult closest = {};
        nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, merged.back(), agent.target_position);
        if (closest.success) {
            target_position = closest.position;
        } else {
            valid = false;
        }
    }

    if (valid) {
        path_corridor::set_path(agent.corridor, target_position, merged);
        local_boundary::reset_local_boundary(agent.boundary);
        agent.target_state = AGENT_TARGET_STATE_VALID;
        agent.target_pathfinding_time = 0;
    } else {
        retry_or_fail(agent);
    }
}

static void update_move_requests(Crowd &crowd, const NavMesh &nav_mesh, float delta_time) {
    for (Agent &agent : crowd.agents) {
        if (agent.allocated && agent.target_state == AGENT_TARGET_STATE_WAITING_FOR_PATH) agent.target_pathfinding_time += delta_time;
    }

    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state == AGENT_STATE_INVALID || agent.target_state == AGENT_TARGET_STATE_NONE ||
            agent.target_state == AGENT_TARGET_STATE_VELOCITY || agent.target_ref == INVALID_NODE_REF) {
            continue;
        }
        if (agent.target_state == AGENT_TARGET_STATE_REQUESTING) request_path(crowd, nav_mesh, agent);
    }

    Array<uint32_t> &pathfinding_agents = crowd.scratch_agents;
    pathfinding_agents.clear();

    for (uint32_t agent_id = 0; agent_id < crowd.agents.length; agent_id++) {
        Agent &agent = crowd.agents[agent_id];
        if (!agent.allocated) continue;

        if (agent.target_state == AGENT_TARGET_STATE_WAITING_FOR_QUEUE) {
            nav_mesh_search::init_sliced_find_node_path(agent.sliced_query, nav_mesh, agent.corridor.path.back(), agent.target_ref, agent.corridor.target,
                                                        agent.target_position, agent.query_filter);
            agent.target_state = AGENT_TARGET_STATE_WAITING_FOR_PATH;
            pathfinding_agents.push(agent_id);
        } else if (agent.target_state == AGENT_TARGET_STATE_WAITING_FOR_PATH) {
            pathfinding_agents.push(agent_id);
        }
    }

    const Array<Agent> &agents = crowd.agents;
    sort::stable(pathfinding_agents.data, pathfinding_agents.length, [&agents](uint32_t a, uint32_t b) {
        return agents[a].target_pathfinding_time > agents[b].target_pathfinding_time;
    });

    int remaining_iterations = crowd.max_iterations_per_update;

    for (uint32_t agent_id : pathfinding_agents) {
        Agent &agent = crowd.agents[agent_id];

        if ((agent.sliced_query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS) != 0 && remaining_iterations > 0) {
            int iterations = crowd.max_iterations_per_agent < remaining_iterations ? crowd.max_iterations_per_agent : remaining_iterations;
            remaining_iterations -= nav_mesh_search::update_sliced_find_node_path(agent.sliced_query, nav_mesh, iterations);
        }

        if ((agent.sliced_query.status & SLICED_FIND_NODE_PATH_FAILURE) != 0) {
            agent.target_state = AGENT_TARGET_STATE_FAILED;
            agent.target_pathfinding_time = 0;
            agent.target_path_is_partial = false;
        } else if ((agent.sliced_query.status & SLICED_FIND_NODE_PATH_SUCCESS) != 0) {
            complete_path(crowd, nav_mesh, agent);
        }
    }
}

static void add_neighbour(Array<AgentNeighbour> &neis, AgentNeighbour neighbour, uint32_t max_neighbours) {
    if (max_neighbours == 0) {
        neis.push(neighbour);
        return;
    }

    uint32_t insert_index = neis.length;
    while (insert_index > 0 && neighbour.distance_sqr < neis[insert_index - 1].distance_sqr) insert_index--;
    if (insert_index >= max_neighbours) return;

    if (neis.length < max_neighbours) neis.push(neighbour);
    for (uint32_t i = neis.length - 1; i > insert_index; i--) neis[i] = neis[i - 1];
    neis[insert_index] = neighbour;
}

static void update_neighbours(Crowd &crowd) {
    float min_x = INFINITY;
    float min_z = INFINITY;
    float max_x = -INFINITY;
    float max_z = -INFINITY;
    float max_query_range = 0;

    Array<uint32_t> &walking = crowd.scratch_agents;
    walking.clear();

    for (uint32_t agent_id = 0; agent_id < crowd.agents.length; agent_id++) {
        Agent &agent = crowd.agents[agent_id];
        if (!agent.allocated) continue;
        agent.neis.clear();
        if (agent.state != AGENT_STATE_WALKING) continue;

        walking.push(agent_id);
        min_x = fminf(min_x, agent.position.x);
        max_x = fmaxf(max_x, agent.position.x);
        min_z = fminf(min_z, agent.position.z);
        max_z = fmaxf(max_z, agent.position.z);
        max_query_range = fmaxf(max_query_range, agent.collision_query_range);
    }

    if (walking.length == 0) return;

    float cell_size = max_query_range;
    if (cell_size < 0.01f) return;

    int grid_width = (int)ceilf((max_x - min_x) / cell_size) + 1;
    int grid_height = (int)ceilf((max_z - min_z) / cell_size) + 1;
    uint32_t grid_size = (uint32_t)(grid_width * grid_height);

    Array<uint32_t> &cells = crowd.scratch_cells;
    Array<uint32_t> &cell_starts = crowd.scratch_cell_starts;
    Array<uint32_t> &cell_agents = crowd.scratch_cell_agents;
    cells.clear();
    cell_starts.clear();
    cell_starts.resize(grid_size + 1, 0);
    cell_agents.resize(walking.length);

    for (uint32_t agent_id : walking) {
        const Agent &agent = crowd.agents[agent_id];
        int ix = (int)floorf((agent.position.x - min_x) / cell_size);
        int iz = (int)floorf((agent.position.z - min_z) / cell_size);
        uint32_t cell = (uint32_t)(iz * grid_width + ix);
        cells.push(cell);
        cell_starts[cell + 1]++;
    }

    for (uint32_t i = 0; i < grid_size; i++) cell_starts[i + 1] += cell_starts[i];

    for (uint32_t i = 0; i < walking.length; i++) {
        uint32_t cell = cells[i];
        cell_agents[cell_starts[cell]++] = walking[i];
    }
    for (uint32_t i = grid_size; i > 0; i--) cell_starts[i] = cell_starts[i - 1];
    cell_starts[0] = 0;

    for (uint32_t agent_id : walking) {
        Agent &agent = crowd.agents[agent_id];
        float query_range_sqr = agent.collision_query_range * agent.collision_query_range;

        int ix = (int)floorf((agent.position.x - min_x) / cell_size);
        int iz = (int)floorf((agent.position.z - min_z) / cell_size);

        for (int dz = -1; dz <= 1; dz++) {
            for (int dx = -1; dx <= 1; dx++) {
                int check_z = iz + dz;
                int check_x = ix + dx;
                if (check_x < 0 || check_x >= grid_width || check_z < 0 || check_z >= grid_height) continue;

                uint32_t cell = (uint32_t)(check_z * grid_width + check_x);
                for (uint32_t k = cell_starts[cell]; k < cell_starts[cell + 1]; k++) {
                    uint32_t other_id = cell_agents[k];
                    if (other_id == agent_id) continue;

                    float distance_sqr = vec3::squared_distance(agent.position, crowd.agents[other_id].position);
                    if (distance_sqr < query_range_sqr) add_neighbour(agent.neis, {other_id, distance_sqr}, crowd.max_neighbours);
                }
            }
        }
    }
}

static void update_local_boundaries(Crowd &crowd, const NavMesh &nav_mesh) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING || agent.corridor.path.length == 0) continue;

        float update_threshold = agent.collision_query_range * 0.25f;
        float moved_distance = vec3::distance(agent.position, agent.boundary.center);

        if (moved_distance > update_threshold || !local_boundary::is_local_boundary_valid(agent.boundary, nav_mesh, agent.query_filter)) {
            local_boundary::update_local_boundary(agent.boundary, agent.corridor.path[0], agent.position, agent.collision_query_range, nav_mesh,
                                                  agent.query_filter, crowd.workspace);
        }
    }
}

static void update_corners(Crowd &crowd, const NavMesh &nav_mesh) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated) continue;

        if (agent.state != AGENT_STATE_WALKING || agent.target_state != AGENT_TARGET_STATE_VALID) {
            agent.desired_velocity = {0, 0, 0};
            continue;
        }

        StraightPathPoint corners[MAX_CORNERS];
        FindCornersResult found;
        path_corridor::find_corners(found, agent.corridor, nav_mesh, corners);
        if (!found.success) {
            agent.desired_velocity = {0, 0, 0};
            continue;
        }

        agent.corners.clear();
        for (uint32_t i = 0; i < found.corner_count; i++) agent.corners.push(corners[i]);

        if ((agent.update_flags & CROWD_UPDATE_OPTIMIZE_VIS) != 0 && agent.corners.length > 0) {
            uint32_t target_index = agent.corners.length > 1 ? 1 : 0;
            path_corridor::optimize_path_visibility(agent.corridor, crowd.workspace, agent.corners[target_index].position, agent.path_optimization_range,
                                                    nav_mesh, agent.query_filter);
        }
    }
}

static float distance_2d_sqr(Vec3 a, Vec3 b) {
    float dx = b.x - a.x;
    float dz = b.z - a.z;
    return dx * dx + dz * dz;
}

static bool agent_is_over_off_mesh_connection(const Agent &agent, float radius) {
    if (agent.corners.length == 0) return false;
    const StraightPathPoint &last = agent.corners.back();
    if (last.type != NODE_TYPE_OFFMESH) return false;
    return distance_2d_sqr(agent.position, last.position) < radius * radius;
}

static void update_off_mesh_connection_triggers(Crowd &crowd, const NavMesh &nav_mesh) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING || agent.target_state == AGENT_TARGET_STATE_NONE ||
            agent.target_state == AGENT_TARGET_STATE_VELOCITY) {
            continue;
        }

        float trigger_radius = agent.radius * 2.25f;
        if (!agent_is_over_off_mesh_connection(agent, trigger_radius)) continue;

        NodeRef off_mesh_node_ref = agent.corners.back().node_ref;
        if (off_mesh_node_ref == INVALID_NODE_REF) continue;

        MoveOverOffMeshConnectionResult result = {};
        path_corridor::move_over_off_mesh_connection(result, agent.corridor, off_mesh_node_ref, nav_mesh);
        if (!result.success) continue;

        agent.state = AGENT_STATE_OFFMESH;
        agent.off_mesh_animation = {
            .active = true,
            .t = 0,
            .start_position = agent.position,
            .end_position = result.end_position,
            .node_ref = result.off_mesh_node_ref,
            .duration = agent.auto_traverse_off_mesh_connections ? 0.5f : -1.0f,
        };
    }
}

bool complete_off_mesh_connection(Crowd &crowd, uint32_t agent_id) {
    Agent *agent = get_agent(crowd, agent_id);
    if (!agent || agent->state != AGENT_STATE_OFFMESH || !agent->off_mesh_animation.active) return false;

    agent->position = agent->off_mesh_animation.end_position;
    agent->velocity = {0, 0, 0};
    agent->desired_velocity = {0, 0, 0};
    agent->off_mesh_animation.active = false;
    agent->state = AGENT_STATE_WALKING;
    return true;
}

static void calc_straight_steer_direction(Agent &agent) {
    if (agent.corners.length == 0) {
        agent.desired_velocity = {0, 0, 0};
        return;
    }

    Vec3 direction = vec3::subtract(agent.corners[0].position, agent.position);
    direction.y = 0;
    agent.desired_velocity = vec3::scale(vec3::normalize(direction), agent.max_speed);
}

static void calc_smooth_steer_direction(Agent &agent) {
    if (agent.corners.length == 0) {
        agent.desired_velocity = {0, 0, 0};
        return;
    }

    Vec3 p0 = agent.corners[0].position;
    Vec3 p1 = agent.corners[agent.corners.length > 1 ? 1 : 0].position;

    Vec3 dir0 = vec3::subtract(p0, agent.position);
    Vec3 dir1 = vec3::subtract(p1, agent.position);
    dir0.y = 0;
    dir1.y = 0;

    float length0 = vec3::length(dir0);
    float length1 = vec3::length(dir1);
    if (length1 > 0.001f) dir1 = vec3::scale(dir1, 1.0f / length1);

    Vec3 direction = {dir0.x - dir1.x * length0 * 0.5f, 0, dir0.z - dir1.z * length0 * 0.5f};
    agent.desired_velocity = vec3::scale(vec3::normalize(direction), agent.max_speed);
}

static float get_distance_to_goal(const Agent &agent, float range) {
    if (agent.corners.length == 0) return range;

    const StraightPathPoint &end = agent.corners.back();
    if ((end.flags & STRAIGHT_PATH_POINT_END) == 0) return range;

    float distance = vec2::distance({end.position.x, end.position.z}, {agent.position.x, agent.position.z});
    return fminf(range, distance);
}

static void update_steering(Crowd &crowd) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated) continue;

        if (agent.target_state == AGENT_TARGET_STATE_VELOCITY) {
            agent.desired_velocity = agent.target_position;
            continue;
        }

        if ((agent.update_flags & CROWD_UPDATE_ANTICIPATE_TURNS) != 0) {
            calc_smooth_steer_direction(agent);
        } else {
            calc_straight_steer_direction(agent);
        }

        float slow_down_radius = agent.radius * 2;
        float speed_scale = get_distance_to_goal(agent, slow_down_radius) / slow_down_radius;

        agent.desired_speed = agent.max_speed;
        agent.desired_velocity = vec3::scale(agent.desired_velocity, speed_scale);

        if ((agent.update_flags & CROWD_UPDATE_SEPARATION) == 0) continue;

        float separation_distance = agent.collision_query_range;
        float inverse_separation_distance = 1.0f / separation_distance;

        float w = 0;
        Vec3 displacement = {0, 0, 0};

        for (const AgentNeighbour &neighbour : agent.neis) {
            if (!is_agent(crowd, neighbour.agent_id)) continue;
            const Agent &other = crowd.agents[neighbour.agent_id];

            Vec3 diff = vec3::subtract(agent.position, other.position);
            diff.y = 0;

            float distance_sqr = vec3::squared_length(diff);
            if (distance_sqr < 0.00001f) continue;
            if (distance_sqr > separation_distance * separation_distance) continue;

            float distance = sqrtf(distance_sqr);
            float weight = agent.separation_weight * (1.0f - distance * inverse_separation_distance * (distance * inverse_separation_distance));

            displacement = vec3::scale_and_add(displacement, diff, weight / distance);
            w += 1.0f;
        }

        if (w > 0.0001f) {
            agent.desired_velocity = vec3::scale_and_add(agent.desired_velocity, displacement, 1.0f / w);

            float speed_sqr = vec3::squared_length(agent.desired_velocity);
            float desired_sqr = agent.desired_speed * agent.desired_speed;
            if (speed_sqr > desired_sqr && speed_sqr > 0) {
                agent.desired_velocity = vec3::scale(agent.desired_velocity, sqrtf(desired_sqr / speed_sqr));
            }
        }
    }
}

static void update_velocity_planning(Crowd &crowd) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;

        if ((agent.update_flags & CROWD_UPDATE_OBSTACLE_AVOIDANCE) == 0) {
            agent.new_velocity = agent.desired_velocity;
            continue;
        }

        obstacle_avoidance::reset_obstacle_avoidance_query(agent.obstacle_avoidance_query);

        for (const AgentNeighbour &neighbour : agent.neis) {
            if (!is_agent(crowd, neighbour.agent_id)) continue;
            const Agent &other = crowd.agents[neighbour.agent_id];
            obstacle_avoidance::add_circle_obstacle(agent.obstacle_avoidance_query, other.position, other.radius, other.velocity,
                                                    other.desired_velocity);
        }

        for (uint32_t i = 0; i < agent.boundary.segment_count; i++) {
            Vec3 p1 = agent.boundary.segments[i].start;
            Vec3 p2 = agent.boundary.segments[i].end;

            float tri_area = (agent.position.x - p1.x) * (p2.z - p1.z) - (agent.position.z - p1.z) * (p2.x - p1.x);
            if (tri_area < 0.0f) continue;

            obstacle_avoidance::add_segment_obstacle(agent.obstacle_avoidance_query, p1, p2);
        }

        ObstacleAvoidanceDebugData *debug = agent.debug_obstacle_avoidance ? &agent.obstacle_avoidance_debug_data : nullptr;
        SampleVelocityResult sampled = {};
        obstacle_avoidance::sample_velocity_adaptive(sampled, agent.obstacle_avoidance_query, agent.position, agent.radius, agent.max_speed, agent.velocity, agent.desired_velocity, agent.obstacle_avoidance, debug);
        agent.new_velocity = sampled.velocity;
    }
}

static void integrate(Crowd &crowd, float delta_time) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;

        float max_delta = agent.max_acceleration * delta_time;
        Vec3 dv = vec3::subtract(agent.new_velocity, agent.velocity);
        float ds = vec3::length(dv);
        if (ds > max_delta) dv = vec3::scale(dv, max_delta / ds);

        agent.velocity = vec3::add(agent.velocity, dv);

        if (vec3::length(agent.velocity) > 0.0001f) {
            agent.position = vec3::scale_and_add(agent.position, agent.velocity, delta_time);
        } else {
            agent.velocity = {0, 0, 0};
        }
    }
}

static void handle_collisions(Crowd &crowd) {
    const float COLLISION_RESOLVE_FACTOR = 0.7f;

    for (int iteration = 0; iteration < 4; iteration++) {
        for (uint32_t agent_id = 0; agent_id < crowd.agents.length; agent_id++) {
            Agent &agent = crowd.agents[agent_id];
            if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;

            agent.displacement = {0, 0, 0};
            float w = 0;

            for (const AgentNeighbour &neighbour : agent.neis) {
                if (!is_agent(crowd, neighbour.agent_id)) continue;
                const Agent &other = crowd.agents[neighbour.agent_id];

                Vec3 diff = vec3::subtract(agent.position, other.position);
                diff.y = 0;

                float distance_sqr = vec3::squared_length(diff);
                float combined_radius = agent.radius + other.radius;
                if (distance_sqr > combined_radius * combined_radius) continue;

                float distance = sqrtf(distance_sqr);
                float penetration = combined_radius - distance;

                if (distance < 0.0001f) {
                    if (agent_id > neighbour.agent_id) {
                        diff = {-agent.desired_velocity.z, 0, agent.desired_velocity.x};
                    } else {
                        diff = {agent.desired_velocity.z, 0, -agent.desired_velocity.x};
                    }
                    penetration = 0.01f;
                } else {
                    penetration = (1.0f / distance) * (penetration * 0.5f) * COLLISION_RESOLVE_FACTOR;
                }

                agent.displacement = vec3::scale_and_add(agent.displacement, diff, penetration);
                w += 1.0f;
            }

            if (w > 0.0001f) agent.displacement = vec3::scale(agent.displacement, 1.0f / w);
        }

        for (Agent &agent : crowd.agents) {
            if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;
            agent.position = vec3::add(agent.position, agent.displacement);
        }
    }
}

static void update_corridors(Crowd &crowd, const NavMesh &nav_mesh) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;

        path_corridor::move_position(agent.corridor, crowd.workspace, agent.position, nav_mesh, agent.query_filter);
        agent.position = agent.corridor.position;

        if (agent.target_state == AGENT_TARGET_STATE_NONE || agent.target_state == AGENT_TARGET_STATE_VELOCITY) {
            path_corridor::reset(agent.corridor, first_corridor_ref(agent), agent.position);
        }
    }
}

static void off_mesh_connection_update(Crowd &crowd, float delta_time) {
    for (Agent &agent : crowd.agents) {
        if (!agent.allocated || !agent.off_mesh_animation.active || !agent.auto_traverse_off_mesh_connections) continue;

        OffMeshAnimation &animation = agent.off_mesh_animation;
        animation.t += delta_time;

        if (animation.t >= animation.duration) {
            animation.active = false;
            agent.state = AGENT_STATE_WALKING;
            continue;
        }

        float progress = animation.t / animation.duration;
        agent.position = vec3::lerp(animation.start_position, animation.end_position, progress);
        agent.velocity = {0, 0, 0};
        agent.desired_velocity = {0, 0, 0};
    }
}

struct TopologyOptimizationCandidate {
    uint32_t agent_id;
    float time;
};

static void update_topology_optimization(Crowd &crowd, const NavMesh &nav_mesh, float delta_time) {
    const float OPT_TIME_THRESHOLD = 0.5f;
    const uint32_t OPT_MAX_AGENTS = 1;

    TopologyOptimizationCandidate queue[OPT_MAX_AGENTS + 1];
    uint32_t queue_length = 0;

    for (uint32_t agent_id = 0; agent_id < crowd.agents.length; agent_id++) {
        Agent &agent = crowd.agents[agent_id];
        if (!agent.allocated || agent.state != AGENT_STATE_WALKING) continue;
        if (agent.target_state == AGENT_TARGET_STATE_NONE || agent.target_state == AGENT_TARGET_STATE_VELOCITY) continue;
        if ((agent.update_flags & CROWD_UPDATE_OPTIMIZE_TOPO) == 0) continue;

        agent.topology_opt_time += delta_time;
        if (agent.topology_opt_time < OPT_TIME_THRESHOLD) continue;

        TopologyOptimizationCandidate candidate = {agent_id, agent.topology_opt_time};

        if (queue_length == 0) {
            queue[queue_length++] = candidate;
        } else if (agent.topology_opt_time <= queue[queue_length - 1].time) {
            if (queue_length < OPT_MAX_AGENTS) queue[queue_length++] = candidate;
        } else {
            uint32_t insert_index = 0;
            for (uint32_t i = 0; i < queue_length; i++) {
                if (agent.topology_opt_time >= queue[i].time) {
                    insert_index = i;
                    break;
                }
            }
            for (uint32_t i = queue_length; i > insert_index; i--) queue[i] = queue[i - 1];
            queue[insert_index] = candidate;
            queue_length++;
            if (queue_length > OPT_MAX_AGENTS) queue_length = OPT_MAX_AGENTS;
        }
    }

    for (uint32_t i = 0; i < queue_length; i++) {
        Agent &agent = crowd.agents[queue[i].agent_id];
        path_corridor::optimize_path_topology(agent.corridor, crowd.topology_query, crowd.workspace, nav_mesh, agent.query_filter);
        agent.topology_opt_time = 0;
    }
}

void update(Crowd &crowd, const NavMesh &nav_mesh, float delta_time) {
    check_path_validity(crowd, nav_mesh, delta_time);
    update_topology_optimization(crowd, nav_mesh, delta_time);
    update_move_requests(crowd, nav_mesh, delta_time);
    update_neighbours(crowd);
    update_local_boundaries(crowd, nav_mesh);
    update_corners(crowd, nav_mesh);
    update_off_mesh_connection_triggers(crowd, nav_mesh);
    update_steering(crowd);
    update_velocity_planning(crowd);
    integrate(crowd, delta_time);
    handle_collisions(crowd);
    update_corridors(crowd, nav_mesh);
    off_mesh_connection_update(crowd, delta_time);
}

bool is_agent_at_target(const Crowd &crowd, uint32_t agent_id, float threshold) {
    if (!is_agent(crowd, agent_id)) return false;
    const Agent &agent = crowd.agents[agent_id];

    if (agent.target_state != AGENT_TARGET_STATE_VALID || agent.corners.length == 0) return false;

    const StraightPathPoint &end = agent.corners.back();
    if ((end.flags & STRAIGHT_PATH_POINT_END) == 0) return false;

    return vec3::distance(agent.position, end.position) <= threshold;
}

}

}
