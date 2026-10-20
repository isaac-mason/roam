#pragma once

#include <stdint.h>

#include "roam/math/scalar.h"

namespace roam {

struct Vec3 {
    float x, y, z;
};

namespace vec3 {

inline Vec3 load(const float *buffer, uint32_t start) { return {buffer[start], buffer[start + 1], buffer[start + 2]}; }

inline void store(float *buffer, uint32_t start, Vec3 a) {
    buffer[start] = a.x;
    buffer[start + 1] = a.y;
    buffer[start + 2] = a.z;
}

inline Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 subtract(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 multiply(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3 scale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 scale_and_add(Vec3 a, Vec3 b, float s) { return {a.x + b.x * s, a.y + b.y * s, a.z + b.z * s}; }
inline Vec3 negate(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 min(Vec3 a, Vec3 b) { return {scalar::min(a.x, b.x), scalar::min(a.y, b.y), scalar::min(a.z, b.z)}; }
inline Vec3 max(Vec3 a, Vec3 b) { return {scalar::max(a.x, b.x), scalar::max(a.y, b.y), scalar::max(a.z, b.z)}; }

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

inline float squared_length(Vec3 a) { return dot(a, a); }
inline float length(Vec3 a) { return scalar::sqrt(squared_length(a)); }

inline float squared_distance(Vec3 a, Vec3 b) {
    float x = b.x - a.x;
    float y = b.y - a.y;
    float z = b.z - a.z;
    return x * x + y * y + z * z;
}

inline float distance(Vec3 a, Vec3 b) { return scalar::sqrt(squared_distance(a, b)); }

inline Vec3 normalize(Vec3 a) {
    float inverse_length = squared_length(a);
    if (inverse_length > 0.0f) inverse_length = 1.0f / scalar::sqrt(inverse_length);
    return scale(a, inverse_length);
}

inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)}; }

inline bool equals(Vec3 a, Vec3 b) {
    return scalar::approximately_equal(a.x, b.x) && scalar::approximately_equal(a.y, b.y) && scalar::approximately_equal(a.z, b.z);
}

inline bool exactly_equals(Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

inline bool finite(Vec3 a) { return scalar::finite(a.x) && scalar::finite(a.y) && scalar::finite(a.z); }

}

}
