#define CGLTF_IMPLEMENTATION
#include "app/model.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "cgltf.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"

using namespace roam;

namespace model {

static void append_primitive(Model &model, Array<MeshView> &views, Array<Array<float>> &owned_positions, Array<Array<uint32_t>> &owned_indices,
                             const cgltf_primitive &primitive, const float *world) {
    if (primitive.type != cgltf_primitive_type_triangles) return;

    const cgltf_accessor *positions = nullptr;
    for (cgltf_size i = 0; i < primitive.attributes_count; i++) {
        if (primitive.attributes[i].type == cgltf_attribute_type_position) positions = primitive.attributes[i].data;
    }
    if (!positions) return;

    Vec3 color = {0.8f, 0.8f, 0.8f};
    if (primitive.material && primitive.material->has_pbr_metallic_roughness) {
        const cgltf_float *factor = primitive.material->pbr_metallic_roughness.base_color_factor;
        color = {factor[0], factor[1], factor[2]};
    }

    uint32_t vertex_count = (uint32_t)positions->count;
    uint32_t base = model.positions.length / 3;
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
        vec3::store(world_positions.data, i * 3, p);
        model.positions.push(p.x);
        model.positions.push(p.y);
        model.positions.push(p.z);
        model.colors.push(color.x);
        model.colors.push(color.y);
        model.colors.push(color.z);
        model.bounds = box3::expand_by_point(model.bounds, p);
    }

    uint32_t index_count = primitive.indices ? (uint32_t)primitive.indices->count : vertex_count;
    Array<uint32_t> indices = {};
    indices.resize(index_count);
    for (uint32_t i = 0; i < index_count; i++) {
        indices[i] = primitive.indices ? (uint32_t)cgltf_accessor_read_index(primitive.indices, i) : i;
        model.indices.push(base + indices[i]);
    }

    owned_positions.push(world_positions);
    owned_indices.push(indices);
    views.push({world_positions.data, indices.data, index_count});
}

static void append_node(Model &model, Array<MeshView> &views, Array<Array<float>> &owned_positions, Array<Array<uint32_t>> &owned_indices,
                        const cgltf_node &node) {
    if (node.mesh) {
        float world[16];
        cgltf_node_transform_world(&node, world);
        for (cgltf_size i = 0; i < node.mesh->primitives_count; i++) {
            append_primitive(model, views, owned_positions, owned_indices, node.mesh->primitives[i], world);
        }
    }
    for (cgltf_size i = 0; i < node.children_count; i++) append_node(model, views, owned_positions, owned_indices, *node.children[i]);
}

Model load(const char *name) {
    Model model = {};
    model.bounds = box3::empty();

    char path[512];
#if defined(ROAM_DEMOS_ASSETS_DIR)
    snprintf(path, sizeof(path), "%s/%s", ROAM_DEMOS_ASSETS_DIR, name);
#else
    snprintf(path, sizeof(path), "/assets/%s", name);
#endif

    cgltf_options options = {};
    cgltf_data *data = nullptr;
    if (cgltf_parse_file(&options, path, &data) != cgltf_result_success) {
        fprintf(stderr, "roam demos: could not parse %s\n", path);
        return model;
    }
    if (cgltf_load_buffers(&options, data, path) != cgltf_result_success) {
        fprintf(stderr, "roam demos: could not load buffers for %s\n", path);
        cgltf_free(data);
        return model;
    }

    Array<MeshView> views = {};
    Array<Array<float>> owned_positions = {};
    Array<Array<uint32_t>> owned_indices = {};

    const cgltf_scene *scene = data->scene ? data->scene : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
    if (scene) {
        for (cgltf_size i = 0; i < scene->nodes_count; i++) append_node(model, views, owned_positions, owned_indices, *scene->nodes[i]);
    }

    MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices(views.data, views.length);
    model.walkable_positions = merged.positions;
    model.walkable_indices = merged.indices;
    model.loaded = model.indices.length > 0;

    for (Array<float> &positions : owned_positions) positions.release();
    for (Array<uint32_t> &indices : owned_indices) indices.release();
    views.release();
    owned_positions.release();
    owned_indices.release();
    cgltf_free(data);
    return model;
}

void release(Model &model) {
    model.positions.release();
    model.colors.release();
    model.indices.release();
    model.walkable_positions.release();
    model.walkable_indices.release();
    model.loaded = false;
}

RayHit raycast(const Model &model, Ray ray) {
    RayHit result = {false, FLT_MAX, {0, 0, 0}};
    for (uint32_t i = 0; i + 2 < model.indices.length; i += 3) {
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
    if (result.hit) result.point = vec3::scale_and_add(ray.origin, ray.direction, result.t);
    return result;
}

}
