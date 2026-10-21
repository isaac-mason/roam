#include "roam/blocks/agents/obstacle_avoidance.h"

#include <float.h>
#include <math.h>

#include "roam/math/scalar.h"

namespace roam {

namespace obstacle_avoidance {

ObstacleAvoidanceQuery create_obstacle_avoidance_query(uint32_t max_circles, uint32_t max_segments) {
    ObstacleAvoidanceQuery query = {};
    query.circles.reserve(max_circles);
    query.segments.reserve(max_segments);
    query.max_circles = max_circles;
    query.max_segments = max_segments;
    query.params = DEFAULT_OBSTACLE_AVOIDANCE_PARAMS;
    return query;
}

void release(ObstacleAvoidanceQuery &query) {
    query.circles.release();
    query.segments.release();
}

void reset_obstacle_avoidance_query(ObstacleAvoidanceQuery &query) {
    query.circles.clear();
    query.segments.clear();
}

void release(ObstacleAvoidanceDebugData &debug) { debug.samples.release(); }

void reset_obstacle_avoidance_debug_data(ObstacleAvoidanceDebugData &debug) { debug.samples.clear(); }

void add_circle_obstacle(ObstacleAvoidanceQuery &query, Vec3 position, float radius, Vec3 velocity, Vec3 desired_velocity) {
    if (query.circles.length >= query.max_circles) return;
    query.circles.push({position, velocity, desired_velocity, radius, {0, 0, 0}, {0, 0, 0}, 0, 0, 0});
}

void add_segment_obstacle(ObstacleAvoidanceQuery &query, Vec3 p, Vec3 q) {
    if (query.segments.length >= query.max_segments) return;
    query.segments.push({p, q, false, {0, 0, 0}, {0, 0, 0}, 0});
}

static float tri_area_2d(Vec3 a, Vec3 b, Vec3 c) {
    float abx = b.x - a.x;
    float abz = b.z - a.z;
    float acx = c.x - a.x;
    float acz = c.z - a.z;
    return acx * abz - abx * acz;
}

static float dot_2d(Vec3 a, Vec3 b) { return a.x * b.x + a.z * b.z; }

static float perp_2d(Vec3 a, Vec3 b) { return a.x * b.z - a.z * b.x; }

static float distance_2d(Vec3 a, Vec3 b) {
    float dx = b.x - a.x;
    float dz = b.z - a.z;
    return sqrtf(dx * dx + dz * dz);
}

static float distance_pt_seg_sqr_2d(Vec3 point, Vec3 p, Vec3 q) {
    float pqx = q.x - p.x;
    float pqz = q.z - p.z;
    float dx = point.x - p.x;
    float dz = point.z - p.z;

    float d = pqx * pqx + pqz * pqz;
    float t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    float distance_x = point.x - (p.x + t * pqx);
    float distance_z = point.z - (p.z + t * pqz);
    return distance_x * distance_x + distance_z * distance_z;
}

struct SweepResult {
    bool hit;
    float t_min;
    float t_max;
};

static SweepResult sweep_circle_circle(const ObstacleCircle &circle, Vec3 v) {
    const float EPS = 0.0001f;

    float sx = circle.sweep_x;
    float sz = circle.sweep_z;
    float c = circle.sweep_c;
    float a = v.x * v.x + v.z * v.z;
    if (a < EPS) return {false, 0, 0};

    float b = v.x * sx + v.z * sz;
    float d = b * b - a * c;
    if (d < 0.0f) return {false, 0, 0};

    float inverse_a = 1.0f / a;
    float rd = sqrtf(d);
    return {true, (b - rd) * inverse_a, (b + rd) * inverse_a};
}

struct RaySegmentResult {
    bool hit;
    float t;
};

static RaySegmentResult intersect_ray_segment(const ObstacleSegment &segment, Vec3 u) {
    Vec3 v = segment.direction;
    Vec3 w = segment.to_agent;

    float d = perp_2d(u, v);
    if (fabsf(d) < 1e-6f) return {false, 0};

    float inverse_d = 1.0f / d;
    float t = segment.direction_perp_to_agent * inverse_d;
    if (t < 0 || t > 1) return {false, 0};

    float s = perp_2d(u, w) * inverse_d;
    if (s < 0 || s > 1) return {false, 0};

    return {true, t};
}

static void prepare_obstacles(ObstacleAvoidanceQuery &query, Vec3 position, float radius, Vec3 desired_velocity) {
    for (ObstacleCircle &circle : query.circles) {
        circle.side_direction = vec3::normalize(vec3::subtract(circle.position, position));
        Vec3 dv = vec3::subtract(circle.desired_velocity, desired_velocity);

        float a = tri_area_2d({0, 0, 0}, circle.side_direction, dv);
        if (a < 0.01f) {
            circle.side_normal = {-circle.side_direction.z, 0, circle.side_direction.x};
        } else {
            circle.side_normal = {circle.side_direction.z, 0, -circle.side_direction.x};
        }

        circle.sweep_x = circle.position.x - position.x;
        circle.sweep_z = circle.position.z - position.z;
        float r = radius + circle.radius;
        circle.sweep_c = circle.sweep_x * circle.sweep_x + circle.sweep_z * circle.sweep_z - r * r;
    }

    for (ObstacleSegment &segment : query.segments) {
        const float r = 0.01f;
        segment.touch = distance_pt_seg_sqr_2d(position, segment.p, segment.q) < r * r;
        segment.direction = vec3::subtract(segment.q, segment.p);
        segment.to_agent = vec3::subtract(position, segment.p);
        segment.direction_perp_to_agent = perp_2d(segment.direction, segment.to_agent);
    }
}

static float process_sample(const ObstacleAvoidanceQuery &query, Vec3 candidate, float sample_size, Vec3 velocity, Vec3 desired_velocity,
                            float min_penalty, ObstacleAvoidanceDebugData *debug) {
    const ObstacleAvoidanceParams &params = query.params;

    float desired_velocity_penalty = params.weight_des_vel * (distance_2d(candidate, desired_velocity) * query.inv_vmax);
    float current_velocity_penalty = params.weight_cur_vel * (distance_2d(candidate, velocity) * query.inv_vmax);

    float min_pen = min_penalty - desired_velocity_penalty - current_velocity_penalty;
    float t_threshold = (params.weight_toi / min_pen - 0.1f) * params.horiz_time;
    if (t_threshold - params.horiz_time > -(float)DBL_EPSILON) return min_penalty;

    float t_min = params.horiz_time;
    float side = 0;
    int side_count = 0;

    for (const ObstacleCircle &circle : query.circles) {
        Vec3 vab = {
            candidate.x * 2 - velocity.x - circle.velocity.x,
            candidate.y * 2 - velocity.y - circle.velocity.y,
            candidate.z * 2 - velocity.z - circle.velocity.z,
        };

        side += fmaxf(0, fminf(1, fminf(dot_2d(circle.side_direction, vab) * 0.5f + 0.5f, dot_2d(circle.side_normal, vab) * 2)));
        side_count++;

        SweepResult sweep = sweep_circle_circle(circle, vab);
        if (!sweep.hit) continue;

        float hit_t_min = sweep.t_min;
        if (hit_t_min < 0.0f && sweep.t_max > 0.0f) hit_t_min = -hit_t_min * 0.5f;

        if (hit_t_min >= 0.0f && hit_t_min < t_min) {
            t_min = hit_t_min;
            if (t_min < t_threshold) return min_penalty;
        }
    }

    for (const ObstacleSegment &segment : query.segments) {
        float hit_t_min = 0;

        if (segment.touch) {
            Vec3 direction = segment.direction;
            Vec3 normal = {-direction.z, direction.y, direction.x};
            if (dot_2d(normal, candidate) < 0.0f) continue;
            hit_t_min = 0.0f;
        } else {
            RaySegmentResult hit = intersect_ray_segment(segment, candidate);
            if (!hit.hit) continue;
            hit_t_min = hit.t;
        }

        hit_t_min *= 2.0f;

        if (hit_t_min < t_min) {
            t_min = hit_t_min;
            if (t_min < t_threshold) return min_penalty;
        }
    }

    if (side_count > 0) side /= (float)side_count;

    float side_penalty = params.weight_side * side;
    float time_of_impact_penalty = params.weight_toi * (1.0f / (0.1f + t_min * query.inv_horiz_time));
    float penalty = desired_velocity_penalty + current_velocity_penalty + side_penalty + time_of_impact_penalty;

    if (debug) {
        debug->samples.push(
            {candidate, sample_size, penalty, desired_velocity_penalty, current_velocity_penalty, side_penalty, time_of_impact_penalty});
    }

    return penalty;
}

static void prepare_sampling(ObstacleAvoidanceQuery &query, Vec3 position, float radius, float vmax, Vec3 desired_velocity,
                             const ObstacleAvoidanceParams &params, ObstacleAvoidanceDebugData *debug) {
    prepare_obstacles(query, position, radius, desired_velocity);

    query.params = params;
    query.inv_horiz_time = 1.0f / query.params.horiz_time;
    query.vmax = vmax;
    query.inv_vmax = vmax > 0 ? 1.0f / vmax : FLT_MAX;

    if (debug) reset_obstacle_avoidance_debug_data(*debug);
}

void sample_velocity_grid(SampleVelocityResult &result, ObstacleAvoidanceQuery &query, Vec3 position, float radius, float vmax, Vec3 velocity,
                          Vec3 desired_velocity, const ObstacleAvoidanceParams &params, ObstacleAvoidanceDebugData *debug) {
    prepare_sampling(query, position, radius, vmax, desired_velocity, params, debug);

    result = {{0, 0, 0}, 0};

    float cvx = desired_velocity.x * params.vel_bias;
    float cvz = desired_velocity.z * params.vel_bias;
    float cs = (vmax * 2 * (1 - params.vel_bias)) / (float)(params.grid_size - 1);
    float half = ((float)(params.grid_size - 1) * cs) * 0.5f;

    float min_penalty = FLT_MAX;

    float vmax_plus_half_cs = vmax + cs / 2;
    float vmax_sqr = vmax_plus_half_cs * vmax_plus_half_cs;

    for (int y = 0; y < params.grid_size; ++y) {
        for (int x = 0; x < params.grid_size; ++x) {
            Vec3 candidate = {cvx + (float)x * cs - half, 0, cvz + (float)y * cs - half};
            if (candidate.x * candidate.x + candidate.z * candidate.z > vmax_sqr) continue;

            float penalty = process_sample(query, candidate, cs, velocity, desired_velocity, min_penalty, debug);
            result.samples++;

            if (penalty < min_penalty) {
                min_penalty = penalty;
                result.velocity = candidate;
            }
        }
    }

}

static Vec3 normalize_2d(Vec3 v) {
    float d = sqrtf(v.x * v.x + v.z * v.z);
    if (d == 0) return v;
    float inverse_d = 1.0f / d;
    return {v.x * inverse_d, v.y, v.z * inverse_d};
}

static Vec3 rotate_2d(Vec3 v, float angle) {
    float c = cosf(angle);
    float s = sinf(angle);
    return {v.x * c - v.z * s, v.y, v.x * s + v.z * c};
}

void sample_velocity_adaptive(SampleVelocityResult &result, ObstacleAvoidanceQuery &query, Vec3 position, float radius, float vmax, Vec3 velocity,
                              Vec3 desired_velocity, const ObstacleAvoidanceParams &params, ObstacleAvoidanceDebugData *debug) {
    prepare_sampling(query, position, radius, vmax, desired_velocity, params, debug);

    float *pattern = query.pattern;
    int pattern_count = 0;

    int divs = scalar::max(1, scalar::min(query.params.adaptive_divs, MAX_PATTERN_DIVS));
    int rings = scalar::max(1, scalar::min(query.params.adaptive_rings, MAX_PATTERN_RINGS));
    int depth = query.params.adaptive_depth;

    float da = (1.0f / (float)divs) * scalar::PI * 2;
    float ca = cosf(da);
    float sa = sinf(da);

    Vec3 desired_direction = normalize_2d(desired_velocity);
    Vec3 desired_direction_rotated = rotate_2d(desired_direction, da * 0.5f);

    pattern[pattern_count * 2] = 0;
    pattern[pattern_count * 2 + 1] = 0;
    pattern_count++;

    for (int j = 0; j < rings; ++j) {
        float r = (float)(rings - j) / (float)rings;
        Vec3 base_direction = j % 2 == 0 ? desired_direction : desired_direction_rotated;

        pattern[pattern_count * 2] = base_direction.x * r;
        pattern[pattern_count * 2 + 1] = base_direction.z * r;
        int last1 = pattern_count * 2;
        int last2 = last1;
        pattern_count++;

        for (int i = 1; i < divs - 1; i += 2) {
            pattern[pattern_count * 2] = pattern[last1] * ca + pattern[last1 + 1] * sa;
            pattern[pattern_count * 2 + 1] = -pattern[last1] * sa + pattern[last1 + 1] * ca;
            pattern[pattern_count * 2 + 2] = pattern[last2] * ca - pattern[last2 + 1] * sa;
            pattern[pattern_count * 2 + 3] = pattern[last2] * sa + pattern[last2 + 1] * ca;

            last1 = pattern_count * 2;
            last2 = last1 + 2;
            pattern_count += 2;
        }

        if ((divs & 1) == 0) {
            pattern[pattern_count * 2] = pattern[last2] * ca - pattern[last2 + 1] * sa;
            pattern[pattern_count * 2 + 1] = pattern[last2] * sa + pattern[last2 + 1] * ca;
            pattern_count++;
        }
    }

    float cr = vmax * (1.0f - query.params.vel_bias);
    Vec3 res = {desired_velocity.x * query.params.vel_bias, 0, desired_velocity.z * query.params.vel_bias};

    result = {{0, 0, 0}, 0};

    float vmax_plus_epsilon = vmax + 0.001f;
    float vmax_sqr = vmax_plus_epsilon * vmax_plus_epsilon;

    for (int k = 0; k < depth; ++k) {
        float min_penalty = FLT_MAX;
        Vec3 best_velocity = {0, 0, 0};
        float cr_over_ten = cr * 0.1f;

        for (int i = 0; i < pattern_count; ++i) {
            Vec3 candidate = {res.x + pattern[i * 2] * cr, 0, res.z + pattern[i * 2 + 1] * cr};
            if (candidate.x * candidate.x + candidate.z * candidate.z > vmax_sqr) continue;

            float penalty = process_sample(query, candidate, cr_over_ten, velocity, desired_velocity, min_penalty, debug);
            result.samples++;

            if (penalty < min_penalty) {
                min_penalty = penalty;
                best_velocity = candidate;
            }
        }

        res = best_velocity;
        cr *= 0.5f;
    }

    result.velocity = res;
}

}

}
