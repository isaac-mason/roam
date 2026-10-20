#pragma once

#include <stdint.h>

#include "app/camera.h"
#include "roam/math/box3.h"
#include "roam/util/array.h"

struct Model {
    bool loaded;
    roam::Array<float> positions;
    roam::Array<float> colors;
    roam::Array<uint32_t> indices;
    roam::Array<float> walkable_positions;
    roam::Array<uint32_t> walkable_indices;
    roam::Box3 bounds;
};

struct RayHit {
    bool hit;
    float t;
    roam::Vec3 point;
};

namespace model {

Model load(const char *name);
void release(Model &model);
RayHit raycast(const Model &model, Ray ray);

}
