#include "roam/generate/input_triangle_mesh.h"

#include <math.h>

#include "roam/generate/common.h"

namespace roam {

namespace input_triangle_mesh {

static Vec3 calc_tri_normal(Vec3 v0, Vec3 v1, Vec3 v2) {
    return vec3::normalize(vec3::cross(vec3::subtract(v1, v0), vec3::subtract(v2, v0)));
}

static Vec3 triangle_normal(const float *vertices, const uint32_t *indices, uint32_t triangle) {
    Vec3 v0 = vec3::load(vertices, indices[triangle * 3] * 3);
    Vec3 v1 = vec3::load(vertices, indices[triangle * 3 + 1] * 3);
    Vec3 v2 = vec3::load(vertices, indices[triangle * 3 + 2] * 3);
    return calc_tri_normal(v0, v1, v2);
}

static float walkable_threshold(float walkable_slope_angle) { return cosf(walkable_slope_angle / 180.0f * scalar::PI); }

void mark_walkable_triangles(const float *vertices, const uint32_t *indices, uint32_t index_count, uint8_t *out_tri_area_ids,
                             float walkable_slope_angle) {
    float threshold = walkable_threshold(walkable_slope_angle);
    uint32_t triangle_count = index_count / 3;
    for (uint32_t i = 0; i < triangle_count; ++i) {
        if (triangle_normal(vertices, indices, i).y > threshold) out_tri_area_ids[i] = WALKABLE_AREA;
    }
}

void clear_unwalkable_triangles(const float *vertices, const uint32_t *indices, uint32_t index_count, uint8_t *tri_area_ids,
                                float walkable_slope_angle) {
    float threshold = walkable_threshold(walkable_slope_angle);
    uint32_t triangle_count = index_count / 3;
    for (uint32_t i = 0; i < triangle_count; ++i) {
        if (triangle_normal(vertices, indices, i).y <= threshold) tri_area_ids[i] = NULL_AREA;
    }
}

Box3 calculate_mesh_bounds(const float *vertices, const uint32_t *indices, uint32_t index_count) {
    Box3 bounds = {{INFINITY, INFINITY, INFINITY}, {-INFINITY, -INFINITY, -INFINITY}};
    for (uint32_t i = 0; i < index_count; ++i) bounds = box3::expand_by_point(bounds, vec3::load(vertices, indices[i] * 3));
    return bounds;
}

}

}
