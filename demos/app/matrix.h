#pragma once

#include <math.h>

#include "roam/math/vec3.h"

struct Mat4 {
    float m[16];
};

namespace matrix {

inline Mat4 identity() { return {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}}; }

inline Mat4 multiply(const Mat4 &a, const Mat4 &b) {
    Mat4 out = {};
    for (int column = 0; column < 4; column++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0;
            for (int k = 0; k < 4; k++) sum += a.m[k * 4 + row] * b.m[column * 4 + k];
            out.m[column * 4 + row] = sum;
        }
    }
    return out;
}

inline Mat4 perspective(float fov_y_radians, float aspect, float near_plane, float far_plane) {
    float f = 1.0f / tanf(fov_y_radians * 0.5f);
    Mat4 out = {};
    out.m[0] = f / aspect;
    out.m[5] = f;
    out.m[10] = (far_plane + near_plane) / (near_plane - far_plane);
    out.m[11] = -1;
    out.m[14] = (2 * far_plane * near_plane) / (near_plane - far_plane);
    return out;
}

inline Mat4 look_at(roam::Vec3 eye, roam::Vec3 target, roam::Vec3 up) {
    using namespace roam;
    Vec3 forward = vec3::normalize(vec3::subtract(target, eye));
    Vec3 side = vec3::normalize(vec3::cross(forward, up));
    Vec3 true_up = vec3::cross(side, forward);
    return {{
        side.x, true_up.x, -forward.x, 0,
        side.y, true_up.y, -forward.y, 0,
        side.z, true_up.z, -forward.z, 0,
        -vec3::dot(side, eye), -vec3::dot(true_up, eye), vec3::dot(forward, eye), 1,
    }};
}

inline roam::Vec3 transform_point(const Mat4 &a, roam::Vec3 p) {
    return {
        a.m[0] * p.x + a.m[4] * p.y + a.m[8] * p.z + a.m[12],
        a.m[1] * p.x + a.m[5] * p.y + a.m[9] * p.z + a.m[13],
        a.m[2] * p.x + a.m[6] * p.y + a.m[10] * p.z + a.m[14],
    };
}

}
