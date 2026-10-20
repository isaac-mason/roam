#pragma once

#include <stdint.h>

#include "roam/util/array.h"

namespace roam {

struct NavMeshPoly {
    uint32_t vertex_base;
    uint32_t vertex_count;
    uint32_t flags;
    uint8_t area;
};

struct NavMeshPolyDetail {
    uint32_t vertices_base;
    uint32_t triangles_base;
    uint32_t vertices_count;
    uint32_t triangles_count;
};

}
