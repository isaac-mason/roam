#pragma once

#include <stdint.h>

#include "roam/generate/build_context.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/math/box3.h"
#include "roam/util/array.h"

namespace roam {

constexpr int32_t NULL_SPAN = -1;

struct HeightfieldSpan {
    int min;
    int max;
    uint8_t area;
    int32_t next;
};

struct Heightfield {
    int width;
    int height;
    Box3 bounds;
    float cell_height;
    float cell_size;
    Array<int32_t> spans;
    Array<HeightfieldSpan> span_pool;
    int32_t free_span;
};

struct GridSize {
    int width;
    int height;
};

namespace heightfield {

GridSize calculate_grid_size(Box3 bounds, float cell_size);

Heightfield create(int width, int height, Box3 bounds, float cell_size, float cell_height);
void release(Heightfield &heightfield);

bool add_heightfield_span(Heightfield &heightfield, int x, int z, int min, int max, uint8_t area_id, int flag_merge_threshold);

bool rasterize_triangles(BuildContext &context, Heightfield &heightfield, const InputTriangleMesh &mesh, Span<const uint8_t> tri_area_ids,
                         int flag_merge_threshold = 1);

void filter_low_hanging_walkable_obstacles(Heightfield &heightfield, int walkable_climb);
void filter_ledge_spans(Heightfield &heightfield, int walkable_height, int walkable_climb);
void filter_walkable_low_height_spans(Heightfield &heightfield, int walkable_height);

void rasterize_sphere(Heightfield &heightfield, Vec3 center, float radius, uint8_t area, int flag_merge_threshold);
void rasterize_capsule(Heightfield &heightfield, Vec3 start, Vec3 end, float radius, uint8_t area, int flag_merge_threshold);
void rasterize_cylinder(Heightfield &heightfield, Vec3 start, Vec3 end, float radius, uint8_t area, int flag_merge_threshold);
void rasterize_box(Heightfield &heightfield, Vec3 center, const Vec3 half_edges[3], uint8_t area, int flag_merge_threshold);
void rasterize_convex(Heightfield &heightfield, const InputTriangleMesh &mesh, uint8_t area, int flag_merge_threshold);

}

}
