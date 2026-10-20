#pragma once

#include <float.h>

#include "roam/math/vec3.h"

namespace roam {

struct Box3 {
    Vec3 min, max;
};

namespace box3 {

inline Box3 empty() { return {{FLT_MAX, FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX, -FLT_MAX}}; }

inline Box3 expand_by_point(Box3 box, Vec3 point) { return {vec3::min(box.min, point), vec3::max(box.max, point)}; }

inline Box3 union_of(Box3 a, Box3 b) { return {vec3::min(a.min, b.min), vec3::max(a.max, b.max)}; }

inline bool intersects(Box3 a, Box3 b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y && a.min.z <= b.max.z &&
           a.max.z >= b.min.z;
}

inline bool contains_point(Box3 box, Vec3 point) {
    return point.x >= box.min.x && point.x <= box.max.x && point.y >= box.min.y && point.y <= box.max.y && point.z >= box.min.z &&
           point.z <= box.max.z;
}

inline Vec3 size(Box3 box) { return vec3::subtract(box.max, box.min); }

}  // namespace box3

}  // namespace roam
