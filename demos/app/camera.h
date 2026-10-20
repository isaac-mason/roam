#pragma once

#include "app/matrix.h"
#include "roam/math/box3.h"
#include "roam/math/vec3.h"

struct Ray {
    roam::Vec3 origin;
    roam::Vec3 direction;
};

struct Camera {
    roam::Vec3 target;
    float yaw;
    float pitch;
    float distance;
    float fov_y;
    float width;
    float height;
    roam::Vec3 eye;
    roam::Vec3 forward;
    roam::Vec3 side;
    roam::Vec3 up;
    Mat4 view_projection;
};

namespace camera {

Camera create();
void frame_bounds(Camera &camera, roam::Box3 bounds);
void look_from(Camera &camera, roam::Vec3 eye, roam::Vec3 target, float fov_y);
void update(Camera &camera, float width, float height);
void orbit(Camera &camera, float dx, float dy);
void pan(Camera &camera, float dx, float dy);
void zoom(Camera &camera, float amount);
Ray screen_ray(const Camera &camera, float x, float y);

}
