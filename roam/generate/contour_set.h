#pragma once

#include <stdint.h>

#include "roam/generate/build_context.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/math/box3.h"
#include "roam/util/array.h"

namespace roam {

struct Contour {
    Array<int> vertices;
    int vertex_count;
    Array<int> raw_vertices;
    int raw_vertex_count;
    int reg;
    uint8_t area;
};

struct ContourSet {
    Array<Contour> contours;
    Box3 bounds;
    float cell_size;
    float cell_height;
    int width;
    int height;
    int border_size;
    float max_error;
};

enum ContourBuildFlags : uint32_t {
    CONTOUR_TESS_WALL_EDGES = 0x01,
    CONTOUR_TESS_AREA_EDGES = 0x02,
};

namespace contour_set {

ContourSet build_contours(BuildContext &context, const CompactHeightfield &compact_heightfield, float max_simplification_error,
                          int max_edge_length, uint32_t build_flags);

void release(ContourSet &contour_set);

}

}
