#include "app/renderer.h"

#include <math.h>

#include "roam/math/scalar.h"

using namespace roam;

namespace renderer {

#if defined(SOKOL_GLES3)
#define SHADER_HEADER "#version 300 es\nprecision highp float;\n"
#else
#define SHADER_HEADER "#version 410\n"
#endif

static const char *COLOR_VERTEX_SHADER = SHADER_HEADER
    "uniform mat4 mvp;\n"
    "layout(location = 0) in vec3 position;\n"
    "layout(location = 1) in vec4 color0;\n"
    "out vec4 color;\n"
    "void main() {\n"
    "    gl_Position = mvp * vec4(position, 1.0);\n"
    "    color = color0;\n"
    "}\n";

static const char *COLOR_FRAGMENT_SHADER = SHADER_HEADER
    "in vec4 color;\n"
    "layout(location = 0) out vec4 frag_color;\n"
    "void main() {\n"
    "    frag_color = color;\n"
    "}\n";

static const char *LIT_VERTEX_SHADER = SHADER_HEADER
    "uniform mat4 mvp;\n"
    "uniform vec4 light;\n"
    "layout(location = 0) in vec3 position;\n"
    "layout(location = 1) in vec3 normal;\n"
    "layout(location = 2) in vec4 color0;\n"
    "out vec4 color;\n"
    "void main() {\n"
    "    gl_Position = mvp * vec4(position, 1.0);\n"
    "    float shade = 0.45 + 0.55 * abs(dot(normalize(normal), light.xyz));\n"
    "    color = vec4(pow(color0.rgb * shade, vec3(1.0 / 2.2)), 1.0);\n"
    "}\n";

struct ColorUniforms {
    float mvp[16];
};

struct LitUniforms {
    float mvp[16];
    float light[4];
};

static uint32_t pack(Color color) {
    uint32_t r = (uint32_t)(scalar::clamp(color.r, 0.0f, 1.0f) * 255.0f + 0.5f);
    uint32_t g = (uint32_t)(scalar::clamp(color.g, 0.0f, 1.0f) * 255.0f + 0.5f);
    uint32_t b = (uint32_t)(scalar::clamp(color.b, 0.0f, 1.0f) * 255.0f + 0.5f);
    uint32_t a = (uint32_t)(scalar::clamp(color.a, 0.0f, 1.0f) * 255.0f + 0.5f);
    return r | (g << 8) | (b << 16) | (a << 24);
}

static sg_shader make_color_shader() {
    sg_shader_desc desc = {};
    desc.vertex_func.source = COLOR_VERTEX_SHADER;
    desc.fragment_func.source = COLOR_FRAGMENT_SHADER;
    desc.attrs[0].glsl_name = "position";
    desc.attrs[1].glsl_name = "color0";
    desc.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    desc.uniform_blocks[0].size = sizeof(ColorUniforms);
    desc.uniform_blocks[0].glsl_uniforms[0].type = SG_UNIFORMTYPE_MAT4;
    desc.uniform_blocks[0].glsl_uniforms[0].glsl_name = "mvp";
    desc.label = "roam color shader";
    return sg_make_shader(&desc);
}

static sg_shader make_lit_shader() {
    sg_shader_desc desc = {};
    desc.vertex_func.source = LIT_VERTEX_SHADER;
    desc.fragment_func.source = COLOR_FRAGMENT_SHADER;
    desc.attrs[0].glsl_name = "position";
    desc.attrs[1].glsl_name = "normal";
    desc.attrs[2].glsl_name = "color0";
    desc.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    desc.uniform_blocks[0].size = sizeof(LitUniforms);
    desc.uniform_blocks[0].glsl_uniforms[0].type = SG_UNIFORMTYPE_MAT4;
    desc.uniform_blocks[0].glsl_uniforms[0].glsl_name = "mvp";
    desc.uniform_blocks[0].glsl_uniforms[1].type = SG_UNIFORMTYPE_FLOAT4;
    desc.uniform_blocks[0].glsl_uniforms[1].glsl_name = "light";
    desc.label = "roam lit shader";
    return sg_make_shader(&desc);
}

static sg_pipeline make_color_pipeline(sg_shader shader, sg_primitive_type primitive_type, bool blend, bool depth_write, const char *label) {
    sg_pipeline_desc desc = {};
    desc.shader = shader;
    desc.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[1].format = SG_VERTEXFORMAT_UBYTE4N;
    desc.primitive_type = primitive_type;
    desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    desc.depth.write_enabled = depth_write;
    desc.cull_mode = SG_CULLMODE_NONE;
    if (blend) {
        desc.colors[0].blend.enabled = true;
        desc.colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_SRC_ALPHA;
        desc.colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        desc.colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
        desc.colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
    }
    desc.label = label;
    return sg_make_pipeline(&desc);
}

static sg_pipeline make_lit_pipeline(sg_shader shader) {
    sg_pipeline_desc desc = {};
    desc.shader = shader;
    desc.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[2].format = SG_VERTEXFORMAT_UBYTE4N;
    desc.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    desc.depth.write_enabled = true;
    desc.depth.bias = 1.0f;
    desc.depth.bias_slope_scale = 1.0f;
    desc.cull_mode = SG_CULLMODE_NONE;
    desc.label = "roam lit pipeline";
    return sg_make_pipeline(&desc);
}

Renderer create() {
    Renderer renderer = {};
    renderer.color_shader = make_color_shader();
    renderer.lit_shader = make_lit_shader();
    renderer.opaque_pipeline = make_color_pipeline(renderer.color_shader, SG_PRIMITIVETYPE_TRIANGLES, false, true, "roam opaque pipeline");
    renderer.transparent_pipeline = make_color_pipeline(renderer.color_shader, SG_PRIMITIVETYPE_TRIANGLES, true, false, "roam transparent pipeline");
    renderer.line_pipeline = make_color_pipeline(renderer.color_shader, SG_PRIMITIVETYPE_LINES, true, false, "roam line pipeline");
    renderer.lit_pipeline = make_lit_pipeline(renderer.lit_shader);
    return renderer;
}

static void release(GpuBatch &batch) {
    if (batch.buffer.id != SG_INVALID_ID) sg_destroy_buffer(batch.buffer);
    batch = {};
}

void destroy(Renderer &renderer) {
    release(renderer.level);
    for (TransientBuffer &transient : renderer.transient) {
        if (transient.buffer.id != SG_INVALID_ID) sg_destroy_buffer(transient.buffer);
    }
    release(renderer.frame);
    sg_destroy_pipeline(renderer.opaque_pipeline);
    sg_destroy_pipeline(renderer.transparent_pipeline);
    sg_destroy_pipeline(renderer.line_pipeline);
    sg_destroy_pipeline(renderer.lit_pipeline);
    sg_destroy_shader(renderer.color_shader);
    sg_destroy_shader(renderer.lit_shader);
}

static GpuBatch make_batch(const void *data, uint32_t vertex_size, uint32_t vertex_count, const char *label) {
    GpuBatch batch = {};
    if (vertex_count == 0) return batch;
    sg_buffer_desc desc = {};
    desc.data = {data, (size_t)vertex_size * vertex_count};
    desc.label = label;
    batch.buffer = sg_make_buffer(&desc);
    batch.vertex_count = (int)vertex_count;
    return batch;
}

static void push_level_triangle(Array<LitVertex> &vertices, const Model &model, uint32_t first_index) {
    Vec3 corners[3];
    uint32_t colors[3];
    for (int k = 0; k < 3; k++) {
        uint32_t index = model.indices[first_index + (uint32_t)k];
        corners[k] = vec3::load(model.positions.data, index * 3);
        Vec3 color = vec3::load(model.colors.data, index * 3);
        colors[k] = pack({color.x, color.y, color.z, 1});
    }
    Vec3 normal = vec3::normalize(vec3::cross(vec3::subtract(corners[1], corners[0]), vec3::subtract(corners[2], corners[0])));
    for (int k = 0; k < 3; k++) vertices.push({corners[k].x, corners[k].y, corners[k].z, normal.x, normal.y, normal.z, colors[k]});
}

void set_level(Renderer &renderer, const Model &model) {
    release(renderer.level);

    Array<LitVertex> vertices = {};
    vertices.reserve(model.indices.length);
    for (const ModelNode &node : model.nodes) {
        if (node.hidden) continue;
        uint32_t end = node.index_start + node.index_count;
        for (uint32_t i = node.index_start; i + 2 < end; i += 3) push_level_triangle(vertices, model, i);
    }

    renderer.level = make_batch(vertices.data, sizeof(LitVertex), vertices.length, "roam level");
    vertices.release();
}

void release(DrawList &list) {
    list.opaque_triangles.release();
    list.transparent_triangles.release();
    list.lines.release();
}

void clear(DrawList &list) {
    list.opaque_triangles.clear();
    list.transparent_triangles.clear();
    list.lines.clear();
}

static void push(Array<ColorVertex> &vertices, Vec3 p, uint32_t color) { vertices.push({p.x, p.y, p.z, color}); }

void line(DrawList &list, Vec3 a, Vec3 b, Color color) {
    uint32_t packed = pack(color);
    push(list.lines, a, packed);
    push(list.lines, b, packed);
}

void triangle(DrawList &list, Vec3 a, Vec3 b, Vec3 c, Color color) {
    Array<ColorVertex> &vertices = color.a < 1 ? list.transparent_triangles : list.opaque_triangles;
    uint32_t packed = pack(color);
    push(vertices, a, packed);
    push(vertices, b, packed);
    push(vertices, c, packed);
}

void point(DrawList &list, Vec3 position, float radius, Color color) {
    const Vec3 axes[6] = {{radius, 0, 0}, {-radius, 0, 0}, {0, radius, 0}, {0, -radius, 0}, {0, 0, radius}, {0, 0, -radius}};
    const int faces[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
    for (const int *face : faces) {
        triangle(list, vec3::add(position, axes[face[0]]), vec3::add(position, axes[face[1]]), vec3::add(position, axes[face[2]]), color);
    }
}

void box(DrawList &list, Vec3 center, Vec3 half, Color color) {
    Vec3 corners[8];
    for (int i = 0; i < 8; i++) {
        corners[i] = {center.x + ((i & 1) ? half.x : -half.x), center.y + ((i & 2) ? half.y : -half.y), center.z + ((i & 4) ? half.z : -half.z)};
    }
    const int faces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
    for (const int *face : faces) {
        triangle(list, corners[face[0]], corners[face[1]], corners[face[2]], color);
        triangle(list, corners[face[0]], corners[face[2]], corners[face[3]], color);
    }
}

void circle(DrawList &list, Vec3 center, float radius, Color color) {
    const int segment_count = 24;
    for (int i = 0; i < segment_count; i++) {
        float a0 = (float)i / segment_count * scalar::PI * 2;
        float a1 = (float)(i + 1) / segment_count * scalar::PI * 2;
        line(list, {center.x + cosf(a0) * radius, center.y, center.z + sinf(a0) * radius},
             {center.x + cosf(a1) * radius, center.y, center.z + sinf(a1) * radius}, color);
    }
}

void cylinder(DrawList &list, Vec3 base, float radius, float height, Color color) {
    const int segment_count = 16;
    Vec3 top_center = {base.x, base.y + height, base.z};
    for (int i = 0; i < segment_count; i++) {
        float a0 = (float)i / segment_count * scalar::PI * 2;
        float a1 = (float)(i + 1) / segment_count * scalar::PI * 2;
        Vec3 b0 = {base.x + cosf(a0) * radius, base.y, base.z + sinf(a0) * radius};
        Vec3 b1 = {base.x + cosf(a1) * radius, base.y, base.z + sinf(a1) * radius};
        Vec3 t0 = {b0.x, b0.y + height, b0.z};
        Vec3 t1 = {b1.x, b1.y + height, b1.z};
        triangle(list, b0, b1, t1, color);
        triangle(list, b0, t1, t0, color);
        triangle(list, t0, t1, top_center, color);
    }
}

void arrow(DrawList &list, Vec3 from, Vec3 to, Color color) {
    line(list, from, to, color);
    Vec3 direction = vec3::subtract(to, from);
    float length = vec3::length(direction);
    if (length < 1e-4f) return;
    direction = vec3::scale(direction, 1.0f / length);
    Vec3 side = {-direction.z, 0, direction.x};
    float head = fminf(0.2f, length * 0.4f);
    Vec3 back = vec3::scale_and_add(to, direction, -head);
    line(list, to, vec3::scale_and_add(back, side, head * 0.5f), color);
    line(list, to, vec3::scale_and_add(back, side, -head * 0.5f), color);
}

static void perpendicular_axes(Vec3 direction, Vec3 &out_u, Vec3 &out_v) {
    Vec3 reference = fabsf(direction.y) > 0.99f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    out_u = vec3::normalize(vec3::cross(direction, reference));
    out_v = vec3::cross(out_u, direction);
}

void thick_line(DrawList &list, Vec3 a, Vec3 b, float width, Color color) {
    Vec3 direction = vec3::subtract(b, a);
    float length = vec3::length(direction);
    if (length < 1e-6f) return;
    Vec3 u, v;
    perpendicular_axes(vec3::scale(direction, 1.0f / length), u, v);
    const float radius = width * 0.5f;
    const int side_count = 8;
    for (int i = 0; i < side_count; i++) {
        float a0 = (float)i / side_count * scalar::PI * 2;
        float a1 = (float)(i + 1) / side_count * scalar::PI * 2;
        Vec3 o0 = vec3::add(vec3::scale(u, cosf(a0) * radius), vec3::scale(v, sinf(a0) * radius));
        Vec3 o1 = vec3::add(vec3::scale(u, cosf(a1) * radius), vec3::scale(v, sinf(a1) * radius));
        triangle(list, vec3::add(a, o0), vec3::add(a, o1), vec3::add(b, o1), color);
        triangle(list, vec3::add(a, o0), vec3::add(b, o1), vec3::add(b, o0), color);
    }
}

void cone(DrawList &list, Vec3 base, Vec3 apex, float radius, Color color) {
    Vec3 direction = vec3::subtract(apex, base);
    float length = vec3::length(direction);
    if (length < 1e-6f) return;
    Vec3 u, v;
    perpendicular_axes(vec3::scale(direction, 1.0f / length), u, v);
    const int segment_count = 16;
    for (int i = 0; i < segment_count; i++) {
        float a0 = (float)i / segment_count * scalar::PI * 2;
        float a1 = (float)(i + 1) / segment_count * scalar::PI * 2;
        Vec3 b0 = vec3::add(base, vec3::add(vec3::scale(u, cosf(a0) * radius), vec3::scale(v, sinf(a0) * radius)));
        Vec3 b1 = vec3::add(base, vec3::add(vec3::scale(u, cosf(a1) * radius), vec3::scale(v, sinf(a1) * radius)));
        triangle(list, b0, b1, apex, color);
        triangle(list, b0, b1, base, color);
    }
}

void arrow_helper(DrawList &list, Vec3 origin, Vec3 direction, float length, Color color, float head_length, float head_width) {
    Vec3 head_base = vec3::scale_and_add(origin, direction, length - head_length);
    line(list, origin, head_base, color);
    cone(list, head_base, vec3::scale_and_add(origin, direction, length), head_width * 0.5f, color);
}

void add_primitives(DrawList &list, const DebugPrimitives &primitives) {
    for (const DebugPrimitive &primitive : primitives) {
        float alpha = primitive.transparent ? primitive.opacity : 1.0f;
        uint32_t vertex_count = primitive.positions.length / 3;

        auto color_at = [&](uint32_t vertex) {
            return Color{primitive.colors[vertex * 3], primitive.colors[vertex * 3 + 1], primitive.colors[vertex * 3 + 2], alpha};
        };

        switch (primitive.type) {
            case DEBUG_PRIMITIVE_TRIANGLES:
                for (uint32_t i = 0; i + 2 < primitive.indices.length; i += 3) {
                    uint32_t a = primitive.indices[i];
                    triangle(list, vec3::load(primitive.positions.data, a * 3), vec3::load(primitive.positions.data, primitive.indices[i + 1] * 3),
                             vec3::load(primitive.positions.data, primitive.indices[i + 2] * 3), color_at(a));
                }
                break;
            case DEBUG_PRIMITIVE_LINES:
                for (uint32_t i = 0; i + 1 < vertex_count; i += 2) {
                    uint32_t color = pack(color_at(i));
                    push(list.lines, vec3::load(primitive.positions.data, i * 3), color);
                    push(list.lines, vec3::load(primitive.positions.data, (i + 1) * 3), pack(color_at(i + 1)));
                }
                break;
            case DEBUG_PRIMITIVE_POINTS:
                for (uint32_t i = 0; i < vertex_count; i++) {
                    point(list, vec3::load(primitive.positions.data, i * 3), fmaxf(primitive.size, 0.02f), color_at(i));
                }
                break;
            case DEBUG_PRIMITIVE_BOXES:
                for (uint32_t i = 0; i < vertex_count; i++) {
                    box(list, vec3::load(primitive.positions.data, i * 3), vec3::scale(vec3::load(primitive.scales.data, i * 3), 0.5f), color_at(i));
                }
                break;
        }
    }
}

StaticDrawing create_static(const DrawList &list) {
    StaticDrawing drawing = {};
    drawing.opaque_triangles = make_batch(list.opaque_triangles.data, sizeof(ColorVertex), list.opaque_triangles.length, "roam static opaque");
    drawing.transparent_triangles =
        make_batch(list.transparent_triangles.data, sizeof(ColorVertex), list.transparent_triangles.length, "roam static transparent");
    drawing.lines = make_batch(list.lines.data, sizeof(ColorVertex), list.lines.length, "roam static lines");
    return drawing;
}

StaticDrawing create_static(const DebugPrimitives &primitives) {
    DrawList list = {};
    add_primitives(list, primitives);
    StaticDrawing drawing = create_static(list);
    release(list);
    return drawing;
}

void release(StaticDrawing &drawing) {
    release(drawing.opaque_triangles);
    release(drawing.transparent_triangles);
    release(drawing.lines);
}

static GpuBatch write_transient(TransientBuffer &transient, const Array<ColorVertex> &vertices) {
    if (vertices.length == 0) return {};

    uint32_t size = vertices.length * (uint32_t)sizeof(ColorVertex);
    if (size > transient.capacity) {
        if (transient.buffer.id != SG_INVALID_ID) sg_destroy_buffer(transient.buffer);
        uint32_t capacity = transient.capacity > 0 ? transient.capacity : 64 * 1024;
        while (capacity < size) capacity *= 2;
        sg_buffer_desc desc = {};
        desc.size = capacity;
        desc.usage.vertex_buffer = true;
        desc.usage.immutable = false;
        desc.usage.write_transient = true;
        desc.label = "roam frame vertices";
        transient.buffer = sg_make_buffer(&desc);
        transient.capacity = capacity;
    }

    sg_write_buffer_desc write = {};
    write.src.data = {vertices.data, size};
    write.dst.buffer = transient.buffer;
    sg_write_buffer_transient(&write);
    return {transient.buffer, (int)vertices.length};
}

static void draw_batch(sg_pipeline pipeline, const GpuBatch &batch, const ColorUniforms &uniforms) {
    if (batch.vertex_count == 0) return;
    sg_apply_pipeline(pipeline);
    sg_bindings bindings = {};
    bindings.vertex_buffers[0] = batch.buffer;
    sg_apply_bindings(&bindings);
    sg_range range = {&uniforms, sizeof(uniforms)};
    sg_apply_uniforms(0, &range);
    sg_draw(0, batch.vertex_count, 1);
}

void render(Renderer &renderer, const Camera &camera, const StaticDrawing *statics, uint32_t static_count, bool show_level) {
    ColorUniforms uniforms = {};
    for (int i = 0; i < 16; i++) uniforms.mvp[i] = camera.view_projection.m[i];

    GpuBatch frame_opaque = write_transient(renderer.transient[0], renderer.frame.opaque_triangles);
    GpuBatch frame_lines = write_transient(renderer.transient[1], renderer.frame.lines);
    GpuBatch frame_transparent = write_transient(renderer.transient[2], renderer.frame.transparent_triangles);

    if (show_level && renderer.level.vertex_count > 0) {
        LitUniforms lit = {};
        for (int i = 0; i < 16; i++) lit.mvp[i] = camera.view_projection.m[i];
        Vec3 light = vec3::normalize({0.4f, 1.0f, 0.3f});
        lit.light[0] = light.x;
        lit.light[1] = light.y;
        lit.light[2] = light.z;

        sg_apply_pipeline(renderer.lit_pipeline);
        sg_bindings bindings = {};
        bindings.vertex_buffers[0] = renderer.level.buffer;
        sg_apply_bindings(&bindings);
        sg_range range = {&lit, sizeof(lit)};
        sg_apply_uniforms(0, &range);
        sg_draw(0, renderer.level.vertex_count, 1);
    }

    for (uint32_t i = 0; i < static_count; i++) draw_batch(renderer.opaque_pipeline, statics[i].opaque_triangles, uniforms);
    draw_batch(renderer.opaque_pipeline, frame_opaque, uniforms);

    for (uint32_t i = 0; i < static_count; i++) draw_batch(renderer.line_pipeline, statics[i].lines, uniforms);
    draw_batch(renderer.line_pipeline, frame_lines, uniforms);

    for (uint32_t i = 0; i < static_count; i++) draw_batch(renderer.transparent_pipeline, statics[i].transparent_triangles, uniforms);
    draw_batch(renderer.transparent_pipeline, frame_transparent, uniforms);
}

}
