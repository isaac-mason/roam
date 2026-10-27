#include <math.h>
#include <stdio.h>

#include "snippets.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/blocks/agents/path_corridor.h"

namespace roam {

static AgentParams default_agent_params() {
    /* SNIPPET_START: crowd */
    const AgentParams params = {
        .radius = 0.3f,
        .height = 1.8f,
        .max_acceleration = 15,
        .max_speed = 3.5f,
        .collision_query_range = 2,
        .separation_weight = 0.5f,
        .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_OBSTACLE_AVOIDANCE |
                        CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OPTIMIZE_VIS |
                        CROWD_UPDATE_OPTIMIZE_TOPO,
    };
    /* SNIPPET_END: crowd */
    return params;
}

static void crowd_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    const QueryFilter &filter = scene.filter;

    const AgentParams params = default_agent_params();

    /* SNIPPET_START: crowd */

    // a crowd for agents up to 0.6 m in radius
    Crowd crowd = crowd::create(0.6f);
    uint32_t agents[4];
    for (uint32_t i = 0; i < 4; i++) {
        agents[i] = crowd::add_agent(crowd, nav_mesh, {-8, 0, -6 + 4 * (float)i}, params);
    }

    // everyone to the far side of the block: the target is a poly and a point on it
    FindNearestPolyResult target = {};
    nav_mesh_api::find_nearest_poly(target, nav_mesh, {8, 0, 2}, {1, 1, 1}, filter);
    for (uint32_t agent : agents) {
        crowd::request_move_target(crowd, agent, target.node_ref, target.position);
    }

    // each frame: plan, steer, avoid each other and move
    for (int frame = 0; frame < 600; frame++) crowd::update(crowd, nav_mesh, 1.0f / 60.0f);

    for (uint32_t agent : agents) {
        const Vec3 at = crowd.agents[agent].position;
        const bool arrived = crowd::is_agent_at_target(crowd, agent, 1.0f);
        printf("agent %u at %.2f %.2f %.2f%s\n", agent, at.x, at.y, at.z, arrived ? ", arrived" : "");
    }
    /* SNIPPET_END: crowd */

    /* SNIPPET_START: crowd_steer */
    // steered by hand, as a player's input would: the agent keeps to the nav mesh and others
    // avoid it
    crowd::request_move_velocity(crowd, agents[0], {0, 0, 2});
    // and stopped
    crowd::reset_move_target(crowd, agents[0]);
    crowd::remove_agent(crowd, agents[3]);
    /* SNIPPET_END: crowd_steer */

    /* SNIPPET_START: crowd_release */
    crowd::release(crowd);
    /* SNIPPET_END: crowd_release */
}

static void crowd_off_mesh_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    const QueryFilter &filter = scene.filter;
    Crowd crowd = crowd::create(0.6f);
    const AgentParams params = default_agent_params();

    /* SNIPPET_START: crowd_off_mesh */
    // an agent that animates its own jumps: when it reaches an off-mesh connection, its state is
    // AGENT_STATE_OFFMESH and its off_mesh_animation says from where to where. move it there,
    // then hand it back to the crowd
    AgentParams jumper_params = params;
    jumper_params.auto_traverse_off_mesh_connections = false;
    const uint32_t jumper = crowd::add_agent(crowd, nav_mesh, {6.5f, 0, -1}, jumper_params);

    FindNearestPolyResult up = {};
    nav_mesh_api::find_nearest_poly(up, nav_mesh, {7, 1, -7}, {1, 1, 1}, filter);
    crowd::request_move_target(crowd, jumper, up.node_ref, up.position);

    const float jump_seconds = 0.4f;
    for (int frame = 0; frame < 600; frame++) {
        crowd::update(crowd, nav_mesh, 1.0f / 60.0f);

        Agent &agent = crowd.agents[jumper];
        if (agent.state != AGENT_STATE_OFFMESH || !agent.off_mesh_animation.active) continue;

        OffMeshAnimation &animation = agent.off_mesh_animation;
        animation.t += 1.0f / 60.0f;
        const float progress = animation.t / jump_seconds;
        if (progress >= 1) {
            crowd::complete_off_mesh_connection(crowd, jumper);
            continue;
        }
        const float arc = 4 * progress * (1 - progress);
        agent.position = vec3::lerp(animation.start_position, animation.end_position, progress);
        agent.position.y += arc;
    }
    /* SNIPPET_END: crowd_off_mesh */

    crowd::release(crowd);
}

static void path_corridor_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: path_corridor */
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);
    FindNearestPolyResult end = {};
    nav_mesh_api::find_nearest_poly(end, nav_mesh, {8, 0, 0}, {1, 1, 1}, filter);

    NodeRef node_path[256];
    FindNodePathResult path = {};
    nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref,
                                    start.position, end.position, filter, node_path);

    PathCorridor corridor = path_corridor::create();
    path_corridor::reset(corridor, start.node_ref, start.position);
    path_corridor::set_path(corridor, end.position, {node_path, path.path_count});

    const float speed = 3;
    for (int frame = 0; frame < 600; frame++) {
        // the next corners, from wherever the agent now is
        StraightPathPoint corners[3];
        FindCornersResult found = {};
        path_corridor::find_corners(found, corridor, nav_mesh, corners);
        if (found.corner_count == 0) break;

        // a step towards the first corner: the corridor moves along the surface and trims what
        // is behind it
        const Vec3 to_corner = vec3::subtract(corners[0].position, corridor.position);
        const float distance = vec3::length(to_corner);
        if (distance < 0.01f) break;
        const Vec3 step = vec3::scale(to_corner, fminf(speed / 60.0f, distance) / distance);
        const Vec3 next = vec3::add(corridor.position, step);
        path_corridor::move_position(corridor, workspace, next, nav_mesh, filter);
    }

    const Vec3 at = corridor.position;
    printf("corridor at %.2f %.2f %.2f, %u polys left\n", at.x, at.y, at.z, corridor.path.length);
    path_corridor::release(corridor);
    /* SNIPPET_END: path_corridor */
}

void run_agent_snippets(Scene &scene) {
    crowd_snippets(scene);
    crowd_off_mesh_snippets(scene);
    path_corridor_snippets(scene);
}

}
