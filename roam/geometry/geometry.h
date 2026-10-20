#pragma once

#include <stdint.h>

#include "roam/math/vec2.h"
#include "roam/math/vec3.h"

namespace roam {

namespace geometry {

struct DistPtSeg2dResult {
    float dist;
    float t;
};

struct DistancePtSegSqr2dResult {
    float dist_sqr;
    float t;
};

struct IntersectSegSeg2dResult {
    bool hit;
    float s;
    float t;
};

struct IntersectSegmentPoly2dResult {
    bool intersects;
    float tmin;
    float tmax;
    int seg_min;
    int seg_max;
};

Vec3 closest_pt_seg_2d(Vec3 point, Vec3 p, Vec3 q);

bool point_in_poly(Vec3 point, const float *vertices, int vertex_count);

float dist_pt_tri(Vec3 p, Vec3 a, Vec3 b, Vec3 c);

float distance_pt_seg(Vec3 point, Vec3 p, Vec3 q);

DistPtSeg2dResult distance_pt_seg_2d(Vec3 point, Vec3 p, Vec3 q);

DistancePtSegSqr2dResult distance_pt_seg_sqr_2d(Vec3 point, Vec3 p, Vec3 q);

float dist_to_tri_mesh(Vec3 p, const float *vertices, const int32_t *triangles, int triangle_count);

float dist_to_poly(int vertex_count, const float *vertices, Vec3 p);

float closest_height_point_triangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c);

bool overlap_seg_seg_2d(Vec2 a, Vec2 b, Vec2 c, Vec2 d);

float tri_area_2d(Vec3 a, Vec3 b, Vec3 c);

IntersectSegSeg2dResult intersect_seg_seg_2d(Vec3 a, Vec3 b, Vec3 c, Vec3 d);

float poly_min_extent(const float *vertices, int vertex_count);

IntersectSegmentPoly2dResult intersect_segment_poly_2d(Vec3 start_position, Vec3 end_position, int vertex_count, const float *vertices);

Vec3 random_point_in_convex_poly(int vertex_count, const float *vertices, float *areas, float s, float t);

bool overlap_poly_poly_2d(const float *vertices_a, int vertex_count_a, const float *vertices_b, int vertex_count_b);

}

}
