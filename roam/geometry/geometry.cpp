#include "roam/geometry/geometry.h"

#include <float.h>
#include <math.h>

namespace roam {

namespace geometry {

static const float EPS = 1e-6f;

Vec3 closest_pt_seg_2d(Vec3 point, Vec3 p, Vec3 q) {
    float pqx = q.x - p.x;
    float pqz = q.z - p.z;
    float dx = point.x - p.x;
    float dz = point.z - p.z;

    float d = pqx * pqx + pqz * pqz;
    float t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;

    return {p.x + t * pqx, p.y, p.z + t * pqz};
}

bool point_in_poly(Vec3 point, const float *vertices, int vertex_count) {
    if (vertex_count <= 0) return false;

    bool inside = false;
    float x = point.x;
    float z = point.z;

    int last = vertex_count - 1;
    float xj = vertices[last * 3];
    float zj = vertices[last * 3 + 2];
    float z_before_j = vertices[(last == 0 ? 0 : last - 1) * 3 + 2];

    for (const float *vertex = vertices, *end = vertices + vertex_count * 3; vertex != end; vertex += 3) {
        float xi = vertex[0];
        float zi = vertex[2];
        if (zj < zi) {
            if (z >= zj && z < zi) {
                float where = (zi - zj) * (x - xi) - (xi - xj) * (z - zi);
                if (where == 0) return true;
                if (where > 0 && (z != zj || z > z_before_j)) inside = !inside;
            }
        } else if (zi < zj) {
            if (z > zi && z <= zj) {
                float where = (zi - zj) * (x - xi) - (xi - xj) * (z - zi);
                if (where == 0) return true;
                if (where < 0 && (z != zj || z < z_before_j)) inside = !inside;
            }
        } else if (z == zi && ((x >= xj && x <= xi) || (x >= xi && x <= xj))) {
            return true;
        }
        z_before_j = zj;
        xj = xi;
        zj = zi;
    }

    return inside;
}

float dist_pt_tri(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 v0 = vec3::subtract(c, a);
    Vec3 v1 = vec3::subtract(b, a);
    Vec3 v2 = vec3::subtract(p, a);

    Vec2 v0_xz = {v0.x, v0.z};
    Vec2 v1_xz = {v1.x, v1.z};
    Vec2 v2_xz = {v2.x, v2.z};

    float dot00 = vec2::dot(v0_xz, v0_xz);
    float dot01 = vec2::dot(v0_xz, v1_xz);
    float dot02 = vec2::dot(v0_xz, v2_xz);
    float dot11 = vec2::dot(v1_xz, v1_xz);
    float dot12 = vec2::dot(v1_xz, v2_xz);

    float inverse_denominator = 1.0f / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * inverse_denominator;
    float v = (dot00 * dot12 - dot01 * dot02) * inverse_denominator;

    const float EPS_TRI = 1e-4f;
    if (u >= -EPS_TRI && v >= -EPS_TRI && u + v <= 1 + EPS_TRI) {
        float y = a.y + v0.y * u + v1.y * v;
        return fabsf(y - p.y);
    }
    return FLT_MAX;
}

float distance_pt_seg(Vec3 point, Vec3 p, Vec3 q) {
    Vec3 pq = vec3::subtract(q, p);
    Vec3 to_point = vec3::subtract(point, p);

    float d = vec3::dot(pq, pq);
    float t = vec3::dot(pq, to_point);
    if (d > 0) t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;

    Vec3 closest = vec3::add(p, vec3::scale(pq, t));
    Vec3 difference = vec3::subtract(closest, point);

    return vec3::dot(difference, difference);
}

DistPtSeg2dResult distance_pt_seg_2d(Vec3 point, Vec3 p, Vec3 q) {
    float pqx = q.x - p.x;
    float pqz = q.z - p.z;
    float dx = point.x - p.x;
    float dz = point.z - p.z;

    float d = pqx * pqx + pqz * pqz;
    float t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;

    float close_dx = p.x + t * pqx - point.x;
    float close_dz = p.z + t * pqz - point.z;

    return {close_dx * close_dx + close_dz * close_dz, t};
}

DistancePtSegSqr2dResult distance_pt_seg_sqr_2d(Vec3 point, Vec3 p, Vec3 q) {
    float pqx = q.x - p.x;
    float pqz = q.z - p.z;
    float dx = point.x - p.x;
    float dz = point.z - p.z;

    float d = pqx * pqx + pqz * pqz;
    float t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;

    float dist_x = p.x + t * pqx - point.x;
    float dist_z = p.z + t * pqz - point.z;

    return {dist_x * dist_x + dist_z * dist_z, t};
}

float dist_to_tri_mesh(Vec3 p, const float *vertices, const int32_t *triangles, int triangle_count) {
    float dmin = FLT_MAX;
    for (int i = 0; i < triangle_count; ++i) {
        Vec3 a = vec3::load(vertices, triangles[i * 4 + 0] * 3);
        Vec3 b = vec3::load(vertices, triangles[i * 4 + 1] * 3);
        Vec3 c = vec3::load(vertices, triangles[i * 4 + 2] * 3);

        float d = dist_pt_tri(p, a, b, c);
        if (d < dmin) dmin = d;
    }
    if (dmin == FLT_MAX) return -1;
    return dmin;
}

float dist_to_poly(int vertex_count, const float *vertices, Vec3 p) {
    float dmin = FLT_MAX;
    bool inside = false;

    for (int i = 0, j = vertex_count - 1; i < vertex_count; j = i++) {
        int vi = i * 3;
        int vj = j * 3;
        if ((vertices[vi + 2] > p.z) != (vertices[vj + 2] > p.z) &&
            p.x < ((vertices[vj] - vertices[vi]) * (p.z - vertices[vi + 2])) / (vertices[vj + 2] - vertices[vi + 2]) + vertices[vi]) {
            inside = !inside;
        }

        DistPtSeg2dResult result = distance_pt_seg_2d(p, vec3::load(vertices, vj), vec3::load(vertices, vi));
        dmin = fminf(dmin, result.dist);
    }
    return inside ? -dmin : dmin;
}

float closest_height_point_triangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    float v0x = c.x - a.x;
    float v0y = c.y - a.y;
    float v0z = c.z - a.z;

    float v1x = b.x - a.x;
    float v1y = b.y - a.y;
    float v1z = b.z - a.z;

    float v2x = p.x - a.x;
    float v2z = p.z - a.z;

    float denominator = v0x * v1z - v0z * v1x;
    if (fabsf(denominator) < EPS) return NAN;

    float u = v1z * v2x - v1x * v2z;
    float v = v0x * v2z - v0z * v2x;

    if (denominator < 0) {
        denominator = -denominator;
        u = -u;
        v = -v;
    }

    if (u >= 0.0f && v >= 0.0f && u + v <= denominator) return a.y + (v0y * u + v1y * v) / denominator;

    return NAN;
}

bool overlap_seg_seg_2d(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    Vec2 ab = vec2::subtract(b, a);
    Vec2 ad = vec2::subtract(d, a);
    float a1 = ab.x * ad.y - ab.y * ad.x;

    Vec2 ac = vec2::subtract(c, a);
    float a2 = ab.x * ac.y - ab.y * ac.x;

    if (a1 * a2 < 0.0f) {
        Vec2 cd = vec2::subtract(d, c);
        Vec2 ca = vec2::subtract(a, c);
        float a3 = cd.x * ca.y - cd.y * ca.x;
        float a4 = a3 + a2 - a1;
        if (a3 * a4 < 0.0f) return true;
    }
    return false;
}

float tri_area_2d(Vec3 a, Vec3 b, Vec3 c) {
    float abx = b.x - a.x;
    float abz = b.z - a.z;
    float acx = c.x - a.x;
    float acz = c.z - a.z;
    return acx * abz - abx * acz;
}

IntersectSegSeg2dResult intersect_seg_seg_2d(Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    float bax = b.x - a.x;
    float baz = b.z - a.z;
    float dcx = d.x - c.x;
    float dcz = d.z - c.z;
    float acx = a.x - c.x;
    float acz = a.z - c.z;
    float denominator = dcz * bax - dcx * baz;
    if (fabsf(denominator) < 1e-12f) return {false, 0, 0};
    float s = (dcx * acz - dcz * acx) / denominator;
    float t = (bax * acz - baz * acx) / denominator;
    bool hit = !(s < 0 || s > 1 || t < 0 || t > 1);
    return {hit, s, t};
}

float poly_min_extent(const float *vertices, int vertex_count) {
    float min_dist = FLT_MAX;

    for (int i = 0; i < vertex_count; i++) {
        int ni = (i + 1) % vertex_count;
        Vec3 p1 = vec3::load(vertices, i * 3);
        Vec3 p2 = vec3::load(vertices, ni * 3);
        float max_edge_dist = 0;
        for (int j = 0; j < vertex_count; j++) {
            if (j == i || j == ni) continue;
            DistPtSeg2dResult result = distance_pt_seg_2d(vec3::load(vertices, j * 3), p1, p2);
            max_edge_dist = fmaxf(max_edge_dist, result.dist);
        }
        min_dist = fminf(min_dist, max_edge_dist);
    }

    return sqrtf(min_dist);
}

static float vperp_2d(Vec3 u, Vec3 v) { return u.z * v.x - u.x * v.z; }

IntersectSegmentPoly2dResult intersect_segment_poly_2d(Vec3 start_position, Vec3 end_position, int vertex_count, const float *vertices) {
    IntersectSegmentPoly2dResult result = {false, 0, 1, -1, -1};

    Vec3 direction = vec3::subtract(end_position, start_position);

    for (int i = 0, j = vertex_count - 1; i < vertex_count; j = i, i++) {
        Vec3 vi = vec3::load(vertices, i * 3);
        Vec3 vj = vec3::load(vertices, j * 3);

        Vec3 edge = vec3::subtract(vi, vj);
        Vec3 to_start = vec3::subtract(start_position, vj);

        float n = vperp_2d(edge, to_start);
        float d = vperp_2d(direction, edge);

        if (fabsf(d) < EPS) {
            if (n < 0) return result;
            continue;
        }

        float t = n / d;

        if (d < 0) {
            if (t > result.tmin) {
                result.tmin = t;
                result.seg_min = j;
                if (result.tmin > result.tmax) return result;
            }
        } else {
            if (t < result.tmax) {
                result.tmax = t;
                result.seg_max = j;
                if (result.tmax < result.tmin) return result;
            }
        }
    }

    result.intersects = true;

    return result;
}

Vec3 random_point_in_convex_poly(int vertex_count, const float *vertices, float *areas, float s, float t) {
    float area_sum = 0;
    Vec3 va = vec3::load(vertices, 0);
    for (int i = 2; i < vertex_count; i++) {
        Vec3 vb = vec3::load(vertices, (i - 1) * 3);
        Vec3 vc = vec3::load(vertices, i * 3);
        areas[i] = tri_area_2d(va, vb, vc);
        area_sum += fmaxf(0.001f, areas[i]);
    }

    float threshold = s * area_sum;
    float accumulated = 0;
    float u = 1;
    int triangle = vertex_count - 1;
    for (int i = 2; i < vertex_count; i++) {
        float area = areas[i];
        if (threshold >= accumulated && threshold < accumulated + area) {
            u = (threshold - accumulated) / area;
            triangle = i;
            break;
        }
        accumulated += area;
    }

    float v = sqrtf(t);

    float a = 1 - v;
    float b = (1 - u) * v;
    float c = u * v;

    Vec3 vb = vec3::load(vertices, (triangle - 1) * 3);
    Vec3 vc = vec3::load(vertices, triangle * 3);

    return {a * va.x + b * vb.x + c * vc.x, a * va.y + b * vb.y + c * vc.y, a * va.z + b * vb.z + c * vc.z};
}

struct Projection {
    float min;
    float max;
};

static Projection project_poly(Vec2 axis, const float *vertices, int vertex_count) {
    float min = axis.x * vertices[0] + axis.y * vertices[2];
    float max = min;

    for (int i = 1; i < vertex_count; i++) {
        float dot = axis.x * vertices[i * 3] + axis.y * vertices[i * 3 + 2];
        min = fminf(min, dot);
        max = fmaxf(max, dot);
    }

    return {min, max};
}

static bool overlap_range(float amin, float amax, float bmin, float bmax, float eps) { return !(amin + eps > bmax || amax - eps < bmin); }

static bool separated_by_edge_normals(const float *edge_vertices, int edge_vertex_count, const float *vertices_a, int vertex_count_a,
                                      const float *vertices_b, int vertex_count_b) {
    const float eps = 1e-4f;

    for (int i = 0, j = edge_vertex_count - 1; i < edge_vertex_count; j = i++) {
        Vec2 va = {edge_vertices[j * 3], edge_vertices[j * 3 + 2]};
        Vec2 vb = {edge_vertices[i * 3], edge_vertices[i * 3 + 2]};
        Vec2 normal = {vb.y - va.y, -(vb.x - va.x)};

        Projection projection_a = project_poly(normal, vertices_a, vertex_count_a);
        Projection projection_b = project_poly(normal, vertices_b, vertex_count_b);

        if (!overlap_range(projection_a.min, projection_a.max, projection_b.min, projection_b.max, eps)) return true;
    }

    return false;
}

bool overlap_poly_poly_2d(const float *vertices_a, int vertex_count_a, const float *vertices_b, int vertex_count_b) {
    if (separated_by_edge_normals(vertices_a, vertex_count_a, vertices_a, vertex_count_a, vertices_b, vertex_count_b)) return false;
    if (separated_by_edge_normals(vertices_b, vertex_count_b, vertices_a, vertex_count_a, vertices_b, vertex_count_b)) return false;
    return true;
}

}

}
