#pragma once

#include <float.h>
#include <math.h>

namespace roam {

namespace scalar {

constexpr float EPSILON = 0.000001f;
constexpr float PI = 3.14159265358979323846f;

inline float min(float a, float b) { return a < b ? a : b; }
inline float max(float a, float b) { return a > b ? a : b; }
inline float abs(float a) { return fabsf(a); }
inline float sqrt(float a) { return sqrtf(a); }
inline float floor(float a) { return floorf(a); }
inline float ceil(float a) { return ceilf(a); }
inline float clamp(float value, float low, float high) { return value < low ? low : value > high ? high : value; }
inline bool finite(float a) { return isfinite(a); }
inline float degrees_to_radians(float degrees) { return degrees * (PI / 180.0f); }

inline bool approximately_equal(float a, float b) { return abs(a - b) <= EPSILON * max(1.0f, max(abs(a), abs(b))); }

inline int min(int a, int b) { return a < b ? a : b; }
inline int max(int a, int b) { return a > b ? a : b; }
inline int abs(int a) { return a < 0 ? -a : a; }
inline int clamp(int value, int low, int high) { return value < low ? low : value > high ? high : value; }

}

}
