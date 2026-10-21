#pragma once

#include <stdint.h>

#include "roam/math/box3.h"
#include "roam/util/span.h"

namespace roam {

struct InputTriangleMesh {
    Span<const float> positions;
    Span<const uint32_t> indices;
};

namespace input_triangle_mesh {

void mark_walkable_triangles(Span<uint8_t> tri_area_ids, const InputTriangleMesh &mesh, float walkable_slope_angle = 45.0f);

void clear_unwalkable_triangles(Span<uint8_t> tri_area_ids, const InputTriangleMesh &mesh, float walkable_slope_angle = 45.0f);

Box3 calculate_mesh_bounds(const InputTriangleMesh &mesh);

}

}
