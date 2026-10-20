#pragma once

#include <stdint.h>

#include "roam/math/vec3.h"
#include "roam/util/array.h"

namespace roam {

constexpr int MAX_PATTERN_DIVS = 32;
constexpr int MAX_PATTERN_RINGS = 4;

struct ObstacleCircle {
    Vec3 position;
    Vec3 velocity;
    Vec3 desired_velocity;
    float radius;
    Vec3 side_direction;
    Vec3 side_normal;
};

struct ObstacleSegment {
    Vec3 p;
    Vec3 q;
    bool touch;
};

struct ObstacleAvoidanceParams {
    float vel_bias;
    float weight_des_vel;
    float weight_cur_vel;
    float weight_side;
    float weight_toi;
    float horiz_time;
    int grid_size;
    int adaptive_divs;
    int adaptive_rings;
    int adaptive_depth;
};

constexpr ObstacleAvoidanceParams DEFAULT_OBSTACLE_AVOIDANCE_PARAMS = {
    .vel_bias = 0.4f,
    .weight_des_vel = 2.0f,
    .weight_cur_vel = 0.75f,
    .weight_side = 0.75f,
    .weight_toi = 2.5f,
    .horiz_time = 2.5f,
    .grid_size = 33,
    .adaptive_divs = 7,
    .adaptive_rings = 2,
    .adaptive_depth = 5,
};

struct ObstacleAvoidanceSample {
    Vec3 velocity;
    float sample_size;
    float penalty;
    float desired_velocity_penalty;
    float current_velocity_penalty;
    float side_penalty;
    float time_of_impact_penalty;
};

struct ObstacleAvoidanceDebugData {
    Array<ObstacleAvoidanceSample> samples;
};

struct ObstacleAvoidanceQuery {
    Array<ObstacleCircle> circles;
    Array<ObstacleSegment> segments;
    uint32_t max_circles;
    uint32_t max_segments;
    ObstacleAvoidanceParams params;
    float inv_horiz_time;
    float vmax;
    float inv_vmax;
    float pattern[(MAX_PATTERN_DIVS * MAX_PATTERN_RINGS + 1) * 2];
};

struct SampleVelocityResult {
    Vec3 velocity;
    int samples;
};

namespace obstacle_avoidance {

ObstacleAvoidanceQuery create_obstacle_avoidance_query(uint32_t max_circles, uint32_t max_segments);
void release(ObstacleAvoidanceQuery &query);
void reset_obstacle_avoidance_query(ObstacleAvoidanceQuery &query);

void release(ObstacleAvoidanceDebugData &debug);
void reset_obstacle_avoidance_debug_data(ObstacleAvoidanceDebugData &debug);

void add_circle_obstacle(ObstacleAvoidanceQuery &query, Vec3 position, float radius, Vec3 velocity, Vec3 desired_velocity);
void add_segment_obstacle(ObstacleAvoidanceQuery &query, Vec3 p, Vec3 q);

SampleVelocityResult sample_velocity_grid(ObstacleAvoidanceQuery &query, Vec3 position, float radius, float vmax, Vec3 velocity, Vec3 desired_velocity,
                                          const ObstacleAvoidanceParams &params, ObstacleAvoidanceDebugData *debug = nullptr);

SampleVelocityResult sample_velocity_adaptive(ObstacleAvoidanceQuery &query, Vec3 position, float radius, float vmax, Vec3 velocity,
                                              Vec3 desired_velocity, const ObstacleAvoidanceParams &params,
                                              ObstacleAvoidanceDebugData *debug = nullptr);

}

}
