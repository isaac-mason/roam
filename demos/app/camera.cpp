#include "app/camera.h"

#include <math.h>

using namespace roam;

namespace camera {

Camera create() {
    Camera camera = {};
    camera.yaw = -0.8f;
    camera.pitch = 0.9f;
    camera.distance = 15;
    camera.fov_y = 60.0f * scalar::PI / 180.0f;
    return camera;
}

void frame_bounds(Camera &camera, Box3 bounds) {
    camera.target = vec3::scale(vec3::add(bounds.min, bounds.max), 0.5f);
    float radius = vec3::length(vec3::subtract(bounds.max, bounds.min)) * 0.5f;
    camera.distance = radius / sinf(camera.fov_y * 0.5f);
}

void update(Camera &camera, float width, float height) {
    camera.width = width;
    camera.height = height;

    Vec3 offset = {
        cosf(camera.pitch) * sinf(camera.yaw) * camera.distance,
        sinf(camera.pitch) * camera.distance,
        cosf(camera.pitch) * cosf(camera.yaw) * camera.distance,
    };
    camera.eye = vec3::add(camera.target, offset);
    camera.forward = vec3::normalize(vec3::subtract(camera.target, camera.eye));
    camera.side = vec3::normalize(vec3::cross(camera.forward, {0, 1, 0}));
    camera.up = vec3::cross(camera.side, camera.forward);

    float near_plane = fmaxf(0.01f, camera.distance * 0.01f);
    float far_plane = camera.distance * 20 + 100;
    Mat4 projection = matrix::perspective(camera.fov_y, width / fmaxf(height, 1), near_plane, far_plane);
    Mat4 view = matrix::look_at(camera.eye, camera.target, {0, 1, 0});
    camera.view_projection = matrix::multiply(projection, view);
}

void orbit(Camera &camera, float dx, float dy) {
    camera.yaw -= dx * 0.008f;
    camera.pitch = scalar::clamp(camera.pitch + dy * 0.008f, -1.5f, 1.5f);
}

void pan(Camera &camera, float dx, float dy) {
    float scale = camera.distance * 0.0015f;
    camera.target = vec3::scale_and_add(camera.target, camera.side, -dx * scale);
    camera.target = vec3::scale_and_add(camera.target, camera.up, dy * scale);
}

void zoom(Camera &camera, float amount) { camera.distance = fmaxf(0.5f, camera.distance * expf(-amount * 0.1f)); }

Ray screen_ray(const Camera &camera, float x, float y) {
    float ndc_x = (x / camera.width) * 2 - 1;
    float ndc_y = 1 - (y / camera.height) * 2;
    float tan_half = tanf(camera.fov_y * 0.5f);
    float aspect = camera.width / fmaxf(camera.height, 1);

    Vec3 direction = camera.forward;
    direction = vec3::scale_and_add(direction, camera.side, ndc_x * tan_half * aspect);
    direction = vec3::scale_and_add(direction, camera.up, ndc_y * tan_half);
    return {camera.eye, vec3::normalize(direction)};
}

}
