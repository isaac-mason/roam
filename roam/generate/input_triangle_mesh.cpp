#include "roam/generate/input_triangle_mesh.h"

#include <math.h>
#include <stddef.h>

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

static float triangle_normal_y(const float *vertices, const uint32_t *indices, size_t triangle, float &length_sqr) {
    const float *v0 = &vertices[(size_t)indices[triangle * 3] * 3];
    const float *v1 = &vertices[(size_t)indices[triangle * 3 + 1] * 3];
    const float *v2 = &vertices[(size_t)indices[triangle * 3 + 2] * 3];
    float e0x = v1[0] - v0[0];
    float e0y = v1[1] - v0[1];
    float e0z = v1[2] - v0[2];
    float e1x = v2[0] - v0[0];
    float e1y = v2[1] - v0[1];
    float e1z = v2[2] - v0[2];
    float nx = e0y * e1z - e0z * e1y;
    float ny = e0z * e1x - e0x * e1z;
    float nz = e0x * e1y - e0y * e1x;
    length_sqr = nx * nx + ny * ny + nz * nz;
    return ny * (1.0f / scalar::sqrt(length_sqr));
}

void mark_walkable_triangles(Span<uint8_t> tri_area_ids, const InputTriangleMesh &mesh, float walkable_slope_angle) {
    const float *vertices = mesh.positions.data;
    const uint32_t *indices = mesh.indices.data;
    uint32_t index_count = mesh.indices.capacity;
    ROAM_ASSERT(tri_area_ids.capacity >= index_count / 3);
    uint8_t *out_tri_area_ids = tri_area_ids.data;
    float threshold = walkable_threshold(walkable_slope_angle);
    size_t triangle_count = index_count / 3;
    float length_sqr;
    if (threshold >= 0.0f) {
        for (size_t i = 0; i < triangle_count; ++i) {
            if (triangle_normal_y(vertices, indices, i, length_sqr) > threshold) out_tri_area_ids[i] = WALKABLE_AREA;
        }
        return;
    }
    for (size_t i = 0; i < triangle_count; ++i) {
        float normal_y = triangle_normal_y(vertices, indices, i, length_sqr);
        if (normal_y > threshold || !(length_sqr > 0.0f)) out_tri_area_ids[i] = WALKABLE_AREA;
    }
}

void clear_unwalkable_triangles(Span<uint8_t> tri_area_id_buffer, const InputTriangleMesh &mesh, float walkable_slope_angle) {
    const float *vertices = mesh.positions.data;
    const uint32_t *indices = mesh.indices.data;
    uint32_t index_count = mesh.indices.capacity;
    ROAM_ASSERT(tri_area_id_buffer.capacity >= index_count / 3);
    uint8_t *tri_area_ids = tri_area_id_buffer.data;
    float threshold = walkable_threshold(walkable_slope_angle);
    uint32_t triangle_count = index_count / 3;
    for (uint32_t i = 0; i < triangle_count; ++i) {
        if (triangle_normal(vertices, indices, i).y <= threshold) tri_area_ids[i] = NULL_AREA;
    }
}

Box3 calculate_mesh_bounds(const InputTriangleMesh &mesh) {
    const float *vertices = mesh.positions.data;
    const uint32_t *indices = mesh.indices.data;
    uint32_t index_count = mesh.indices.capacity;
    Box3 bounds = {{INFINITY, INFINITY, INFINITY}, {-INFINITY, -INFINITY, -INFINITY}};
    for (uint32_t i = 0; i < index_count; ++i) bounds = box3::expand_by_point(bounds, vec3::load(vertices, indices[i] * 3));
    return bounds;
}

}

}
