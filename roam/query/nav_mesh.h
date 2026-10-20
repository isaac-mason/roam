#pragma once

#include <stdint.h>

#include "roam/util/array.h"

namespace roam {

struct NavMeshPoly {
    Array<uint32_t> vertices;
    Array<int> neis;
    uint32_t flags;
    uint8_t area;
};

struct NavMeshPolyDetail {
    uint32_t vertices_base;
    uint32_t triangles_base;
    uint32_t vertices_count;
    uint32_t triangles_count;
};

namespace nav_mesh {

inline void release(NavMeshPoly &poly) {
    poly.vertices.release();
    poly.neis.release();
}

inline void release(Array<NavMeshPoly> &polys) {
    for (NavMeshPoly &poly : polys) release(poly);
    polys.release();
}

}

}
