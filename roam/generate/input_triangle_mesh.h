#pragma once

#include <stdint.h>

#include "roam/math/box3.h"

namespace roam {

namespace input_triangle_mesh {

void mark_walkable_triangles(const float *vertices, const uint32_t *indices, uint32_t index_count, uint8_t *out_tri_area_ids,
                             float walkable_slope_angle = 45.0f);

void clear_unwalkable_triangles(const float *vertices, const uint32_t *indices, uint32_t index_count, uint8_t *tri_area_ids,
                                float walkable_slope_angle = 45.0f);

Box3 calculate_mesh_bounds(const float *vertices, const uint32_t *indices, uint32_t index_count);

}

}
