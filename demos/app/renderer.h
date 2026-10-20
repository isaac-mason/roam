#pragma once

#include <stdint.h>

#include "app/camera.h"
#include "app/model.h"
#include "roam/debug/debug.h"
#include "roam/math/vec3.h"
#include "roam/util/array.h"
#include "sokol_gfx.h"

struct ColorVertex {
    float x, y, z;
    uint32_t color;
};

struct LitVertex {
    float x, y, z;
    float nx, ny, nz;
    uint32_t color;
};

struct Color {
    float r, g, b, a;
};

struct DrawList {
    roam::Array<ColorVertex> opaque_triangles;
    roam::Array<ColorVertex> transparent_triangles;
    roam::Array<ColorVertex> lines;
};

struct GpuBatch {
    sg_buffer buffer;
    int vertex_count;
};

struct StaticDrawing {
    GpuBatch opaque_triangles;
    GpuBatch transparent_triangles;
    GpuBatch lines;
};

struct TransientBuffer {
    sg_buffer buffer;
    uint32_t capacity;
};

struct Renderer {
    sg_shader color_shader;
    sg_shader lit_shader;
    sg_pipeline opaque_pipeline;
    sg_pipeline transparent_pipeline;
    sg_pipeline line_pipeline;
    sg_pipeline lit_pipeline;
    TransientBuffer transient[3];
    GpuBatch level;
    DrawList frame;
};

namespace renderer {

Renderer create();
void destroy(Renderer &renderer);

void set_level(Renderer &renderer, const Model &model);

StaticDrawing create_static(const roam::DebugPrimitives &primitives);
StaticDrawing create_static(const DrawList &list);
void release(StaticDrawing &drawing);

void release(DrawList &list);
void clear(DrawList &list);
void add_primitives(DrawList &list, const roam::DebugPrimitives &primitives);
void line(DrawList &list, roam::Vec3 a, roam::Vec3 b, Color color);
void triangle(DrawList &list, roam::Vec3 a, roam::Vec3 b, roam::Vec3 c, Color color);
void point(DrawList &list, roam::Vec3 position, float radius, Color color);
void box(DrawList &list, roam::Vec3 center, roam::Vec3 half_extents, Color color);
void circle(DrawList &list, roam::Vec3 center, float radius, Color color);
void cylinder(DrawList &list, roam::Vec3 base, float radius, float height, Color color);
void arrow(DrawList &list, roam::Vec3 from, roam::Vec3 to, Color color);
void thick_line(DrawList &list, roam::Vec3 a, roam::Vec3 b, float width, Color color);
void cone(DrawList &list, roam::Vec3 base, roam::Vec3 apex, float radius, Color color);
void arrow_helper(DrawList &list, roam::Vec3 origin, roam::Vec3 direction, float length, Color color, float head_length, float head_width);

void render(Renderer &renderer, const Camera &camera, const StaticDrawing *statics, uint32_t static_count, bool show_level);

}
