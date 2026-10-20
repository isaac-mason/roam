#pragma once

#include <stdint.h>

#include "roam/math/scalar.h"

namespace roam {

struct Vec2 {
    float x, y;
};

namespace vec2 {

inline Vec2 load(const float *buffer, uint32_t start) { return {buffer[start], buffer[start + 1]}; }

inline void store(float *buffer, uint32_t start, Vec2 a) {
    buffer[start] = a.x;
    buffer[start + 1] = a.y;
}

inline Vec2 add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 subtract(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 scale(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float squared_length(Vec2 a) { return dot(a, a); }
inline float length(Vec2 a) { return scalar::sqrt(squared_length(a)); }

inline float squared_distance(Vec2 a, Vec2 b) {
    float x = b.x - a.x;
    float y = b.y - a.y;
    return x * x + y * y;
}

inline float distance(Vec2 a, Vec2 b) { return scalar::sqrt(squared_distance(a, b)); }

inline bool equals(Vec2 a, Vec2 b) { return scalar::approximately_equal(a.x, b.x) && scalar::approximately_equal(a.y, b.y); }

inline bool exactly_equals(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }

inline bool finite(Vec2 a) { return scalar::finite(a.x) && scalar::finite(a.y); }

}  // namespace vec2

}  // namespace roam
