#define CGLTF_IMPLEMENTATION
#include "app/model.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cgltf.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include "stb_image.h"
#pragma clang diagnostic pop

using namespace roam;

namespace model {

struct Builder {
    Model model;
    const cgltf_data *data;
    ModelLoadOptions options;
    Array<int32_t> image_textures;
    Array<InputTriangleMesh> views;
    Array<Array<float>> owned_positions;
    Array<Array<uint32_t>> owned_indices;
};

static int32_t decode_texture(Builder &builder, const cgltf_texture *texture) {
    if (!texture || !texture->image || !texture->image->buffer_view) return -1;
    uint32_t image_index = (uint32_t)cgltf_image_index(builder.data, texture->image);
    if (builder.image_textures[image_index] != -2) return builder.image_textures[image_index];

    builder.image_textures[image_index] = -1;
    const cgltf_buffer_view *view = texture->image->buffer_view;
    const uint8_t *bytes = (const uint8_t *)cgltf_buffer_view_data(view);
    if (!bytes) return -1;

    int width = 0;
    int height = 0;
    int channels = 0;
    uint8_t *pixels = stbi_load_from_memory(bytes, (int)view->size, &width, &height, &channels, 4);
    if (!pixels) return -1;

    ModelTexture decoded = {width, height, {}};
    decoded.pixels.resize((uint32_t)(width * height * 4));
    memcpy(decoded.pixels.data, pixels, decoded.pixels.length);
    stbi_image_free(pixels);

    builder.image_textures[image_index] = (int32_t)builder.model.textures.length;
    builder.model.textures.push(decoded);
    return builder.image_textures[image_index];
}

static void append_primitive(Builder &builder, const cgltf_primitive &primitive, const float *world, bool walkable) {
    if (primitive.type != cgltf_primitive_type_triangles) return;
    Model &model = builder.model;

    const cgltf_accessor *positions = nullptr;
    const cgltf_accessor *uvs = nullptr;
    for (cgltf_size i = 0; i < primitive.attributes_count; i++) {
        const cgltf_attribute &attribute = primitive.attributes[i];
        if (attribute.type == cgltf_attribute_type_position) positions = attribute.data;
        if (attribute.type == cgltf_attribute_type_texcoord && attribute.index == 0) uvs = attribute.data;
    }
    if (!positions) return;

    Vec3 color = {0.8f, 0.8f, 0.8f};
    int32_t texture = -1;
    if (primitive.material && primitive.material->has_pbr_metallic_roughness) {
        const cgltf_pbr_metallic_roughness &pbr = primitive.material->pbr_metallic_roughness;
        color = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
        if (uvs) texture = decode_texture(builder, pbr.base_color_texture.texture);
    }

    uint32_t vertex_count = (uint32_t)positions->count;
    uint32_t base = model.positions.length / 3;
    ModelNode &model_node = model.nodes[model.nodes.length - 1];
    Array<float> world_positions = {};
    world_positions.resize(vertex_count * 3);

    for (uint32_t i = 0; i < vertex_count; i++) {
        float local[3] = {0, 0, 0};
        cgltf_accessor_read_float(positions, i, local, 3);
        Vec3 p = {
            world[0] * local[0] + world[4] * local[1] + world[8] * local[2] + world[12],
            world[1] * local[0] + world[5] * local[1] + world[9] * local[2] + world[13],
            world[2] * local[0] + world[6] * local[1] + world[10] * local[2] + world[14],
        };
        float uv[2] = {0, 0};
        if (uvs) cgltf_accessor_read_float(uvs, i, uv, 2);
        vec3::store(world_positions.data, i * 3, p);
        model.positions.push(p.x);
        model.positions.push(p.y);
        model.positions.push(p.z);
        model.colors.push(color.x);
        model.colors.push(color.y);
        model.colors.push(color.z);
        model.uvs.push(uv[0]);
        model.uvs.push(uv[1]);
        model.vertex_textures.push(texture);
        model.bounds = box3::expand_by_point(model.bounds, p);
        model_node.bounds = box3::expand_by_point(model_node.bounds, p);
    }

    uint32_t index_count = primitive.indices ? (uint32_t)primitive.indices->count : vertex_count;
    Array<uint32_t> indices = {};
    indices.resize(index_count);
    for (uint32_t i = 0; i < index_count; i++) {
        indices[i] = primitive.indices ? (uint32_t)cgltf_accessor_read_index(primitive.indices, i) : i;
        model.indices.push(base + indices[i]);
    }
    model_node.index_count += index_count;

    builder.owned_positions.push(world_positions);
    builder.owned_indices.push(indices);
    if (walkable) builder.views.push({world_positions, {indices.data, index_count}});
}

static bool is_unwalkable(const cgltf_extras &extras) {
    if (!extras.data) return false;
    const char *key = strstr(extras.data, "\"walkable\"");
    if (!key) return false;
    const char *value = key + strlen("\"walkable\"");
    while (*value == ' ' || *value == ':' || *value == '\t' || *value == '\n' || *value == '\r') value++;
    return strncmp(value, "false", 5) == 0;
}

static void append_node(Builder &builder, const cgltf_node &node, bool included, int32_t parent) {
    Model &model = builder.model;
    const ModelLoadOptions &options = builder.options;
    included = included || !options.node_name || (node.name && strcmp(node.name, options.node_name) == 0);

    ModelNode model_node = {};
    if (node.name) snprintf(model_node.name, sizeof(model_node.name), "%s", node.name);
    if (node.extras.data) snprintf(model_node.extras, sizeof(model_node.extras), "%s", node.extras.data);
    model_node.parent = parent;
    model_node.index_start = model.indices.length;
    model_node.bounds = box3::empty();
    model.nodes.push(model_node);
    int32_t node_index = (int32_t)model.nodes.length - 1;

    if (node.mesh && included) {
        float world[16];
        cgltf_node_transform_world(&node, world);
        bool walkable = !options.skip_unwalkable_nodes || !is_unwalkable(node.extras);
        for (cgltf_size i = 0; i < node.mesh->primitives_count; i++) append_primitive(builder, node.mesh->primitives[i], world, walkable);
    }
    for (cgltf_size i = 0; i < node.children_count; i++) append_node(builder, *node.children[i], included, node_index);
}

static uint32_t next_model_id = 1;

static Model build(cgltf_data *data, const ModelLoadOptions &options) {
    Builder builder = {};
    builder.model.id = next_model_id++;
    builder.model.bounds = box3::empty();
    builder.data = data;
    builder.options = options;
    builder.image_textures.resize((uint32_t)data->images_count, -2);

    const cgltf_scene *scene = data->scene ? data->scene : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
    if (scene) {
        for (cgltf_size i = 0; i < scene->nodes_count; i++) append_node(builder, *scene->nodes[i], false, -1);
    }

    Model &model = builder.model;
    MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices(builder.views);
    model.walkable_positions = merged.positions;
    model.walkable_indices = merged.indices;
    model.loaded = model.indices.length > 0;

    for (Array<float> &positions : builder.owned_positions) positions.release();
    for (Array<uint32_t> &indices : builder.owned_indices) indices.release();
    builder.views.release();
    builder.owned_positions.release();
    builder.owned_indices.release();
    builder.image_textures.release();
    return model;
}

Model load_file(const char *path, ModelLoadOptions options) {
    cgltf_options gltf_options = {};
    cgltf_data *data = nullptr;
    if (cgltf_parse_file(&gltf_options, path, &data) != cgltf_result_success) {
        fprintf(stderr, "roam demos: could not parse %s\n", path);
        return {};
    }
    if (cgltf_load_buffers(&gltf_options, data, path) != cgltf_result_success) {
        fprintf(stderr, "roam demos: could not load buffers for %s\n", path);
        cgltf_free(data);
        return {};
    }
    Model model = build(data, options);
    cgltf_free(data);
    return model;
}

Model load_from_memory(const uint8_t *bytes, size_t size, const char *path, ModelLoadOptions options) {
    cgltf_options gltf_options = {};
    cgltf_data *data = nullptr;
    if (cgltf_parse(&gltf_options, bytes, size, &data) != cgltf_result_success) {
        fprintf(stderr, "roam demos: could not parse %s\n", path);
        return {};
    }
    if (cgltf_load_buffers(&gltf_options, data, path) != cgltf_result_success) {
        fprintf(stderr, "roam demos: could not load buffers for %s\n", path);
        cgltf_free(data);
        return {};
    }
    Model model = build(data, options);
    cgltf_free(data);
    return model;
}

Model load(const char *name, ModelLoadOptions options) {
    char path[512];
#if defined(ROAM_DEMOS_ASSETS_DIR)
    snprintf(path, sizeof(path), "%s/%s", ROAM_DEMOS_ASSETS_DIR, name);
#else
    snprintf(path, sizeof(path), "/assets/%s", name);
#endif
    return load_file(path, options);
}

void release(Model &model) {
    model.nodes.release();
    model.positions.release();
    model.colors.release();
    model.uvs.release();
    model.vertex_textures.release();
    for (ModelTexture &texture : model.textures) texture.pixels.release();
    model.textures.release();
    model.indices.release();
    model.walkable_positions.release();
    model.walkable_indices.release();
    model.loaded = false;
}

RayHit raycast(const Model &model, Ray ray) {
    RayHit result = {false, FLT_MAX, {0, 0, 0}};
    for (const ModelNode &node : model.nodes) {
        if (node.hidden) continue;
        for (uint32_t i = node.index_start; i + 2 < node.index_start + node.index_count; i += 3) {
            Vec3 a = vec3::load(model.positions.data, model.indices[i] * 3);
            Vec3 b = vec3::load(model.positions.data, model.indices[i + 1] * 3);
            Vec3 c = vec3::load(model.positions.data, model.indices[i + 2] * 3);

            Vec3 ab = vec3::subtract(b, a);
            Vec3 ac = vec3::subtract(c, a);
            Vec3 p = vec3::cross(ray.direction, ac);
            float determinant = vec3::dot(ab, p);
            if (fabsf(determinant) < 1e-9f) continue;

            float inverse = 1.0f / determinant;
            Vec3 s = vec3::subtract(ray.origin, a);
            float u = vec3::dot(s, p) * inverse;
            if (u < 0 || u > 1) continue;

            Vec3 q = vec3::cross(s, ab);
            float v = vec3::dot(ray.direction, q) * inverse;
            if (v < 0 || u + v > 1) continue;

            float t = vec3::dot(ac, q) * inverse;
            if (t > 0 && t < result.t) {
                result.hit = true;
                result.t = t;
            }
        }
    }
    if (result.hit) result.point = vec3::scale_and_add(ray.origin, ray.direction, result.t);
    return result;
}

bool is_descendant(const Model &model, int32_t node, int32_t ancestor) {
    while (node >= 0) {
        if (node == ancestor) return true;
        node = model.nodes[(uint32_t)node].parent;
    }
    return false;
}

int32_t find_node(const Model &model, const char *name) {
    for (uint32_t i = 0; i < model.nodes.length; i++) {
        if (strcmp(model.nodes[i].name, name) == 0) return (int32_t)i;
    }
    return -1;
}

bool extras_number(const ModelNode &node, const char *key, float &out_value) {
    char quoted[64];
    snprintf(quoted, sizeof(quoted), "\"%s\"", key);
    const char *found = strstr(node.extras, quoted);
    if (!found) return false;
    const char *colon = strchr(found + strlen(quoted), ':');
    if (!colon) return false;
    char *end = nullptr;
    float value = strtof(colon + 1, &end);
    if (end == colon + 1) return false;
    out_value = value;
    return true;
}

}
