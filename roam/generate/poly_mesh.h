#pragma once

#include <stdint.h>

#include "roam/generate/build_context.h"
#include "roam/generate/contour_set.h"
#include "roam/math/box3.h"
#include "roam/util/array.h"

namespace roam {

struct PolyMesh {
    Array<int> vertices;
    Array<int> polys;
    Array<int> regions;
    Array<uint32_t> flags;
    Array<uint8_t> areas;
    int vertex_count;
    int poly_count;
    int max_vertices_per_poly;
    Box3 bounds;
    int local_width;
    int local_height;
    float cell_size;
    float cell_height;
    int border_size;
    float max_edge_error;
};

namespace poly_mesh {

PolyMesh build_poly_mesh(BuildContext &context, const ContourSet &contour_set, int max_vertices_per_poly);

void release(PolyMesh &poly_mesh);

}

}
