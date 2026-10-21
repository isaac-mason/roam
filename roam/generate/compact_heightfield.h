#pragma once

#include <stdint.h>

#include "roam/generate/build_context.h"
#include "roam/generate/heightfield.h"
#include "roam/math/box3.h"
#include "roam/util/array.h"
#include "roam/util/span.h"

namespace roam {

struct CompactHeightfieldSpan {
    uint16_t y;
    uint16_t region;
    uint32_t con : 24;
    uint32_t h : 8;
};

static_assert(sizeof(CompactHeightfieldSpan) == 8);

struct CompactHeightfieldCell {
    uint32_t index;
    uint32_t count;
};

struct CompactHeightfield {
    int width;
    int height;
    int walkable_height_voxels;
    int walkable_climb_voxels;
    int border_size;
    int max_distance;
    int max_regions;
    Box3 bounds;
    float cell_size;
    float cell_height;
    Array<CompactHeightfieldCell> cells;
    Array<CompactHeightfieldSpan> spans;
    Array<uint8_t> areas;
    Array<uint16_t> distances;
};

struct WalkableAreaThreshold {
    uint8_t area_id;
    int walkable_radius_voxels;
};

namespace compact_heightfield {

inline void set_con(CompactHeightfieldSpan &span, int direction, int layer_index) {
    uint32_t shift = (uint32_t)direction * 6;
    uint32_t mask = 0x3fu << shift;
    span.con = (span.con & ~mask) | (((uint32_t)layer_index & 0x3f) << shift);
}

inline int get_con(const CompactHeightfieldSpan &span, int direction) { return (int)((span.con >> ((uint32_t)direction * 6)) & 0x3f); }

CompactHeightfield build(BuildContext &context, const Heightfield &heightfield, int walkable_height_voxels, int walkable_climb_voxels);
void release(CompactHeightfield &compact_heightfield);

void erode_walkable_area(CompactHeightfield &compact_heightfield, int walkable_radius_voxels);
void erode_and_mark_walkable_areas(CompactHeightfield &compact_heightfield, int base_walkable_radius_voxels,
                                   Span<const WalkableAreaThreshold> thresholds);

void mark_box_area(CompactHeightfield &compact_heightfield, Box3 bounds, uint8_t area_id);
void mark_rotated_box_area(CompactHeightfield &compact_heightfield, Vec3 center, Vec3 half_extents, float angle_radians, uint8_t area_id);
void mark_convex_poly_area(CompactHeightfield &compact_heightfield, Span<const float> polygon, float min_y, float max_y, uint8_t area_id);
void mark_cylinder_area(CompactHeightfield &compact_heightfield, Vec3 position, float radius, float height, uint8_t area_id);

bool median_filter_walkable_area(CompactHeightfield &compact_heightfield);

}

}
