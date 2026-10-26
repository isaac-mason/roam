#include "roam/query/bv_tree.h"

#include <math.h>

#include "roam/util/sort.h"

namespace roam {

namespace bv_tree {

static const float SNAP_EPSILON = 1e-3f;

static void calc_extends(const NavMeshBvNode *items, int imin, int imax, uint16_t *bmin, uint16_t *bmax) {
    for (int axis = 0; axis < 3; axis++) {
        bmin[axis] = items[imin].bmin[axis];
        bmax[axis] = items[imin].bmax[axis];
    }
    for (int i = imin + 1; i < imax; ++i) {
        for (int axis = 0; axis < 3; axis++) {
            if (items[i].bmin[axis] < bmin[axis]) bmin[axis] = items[i].bmin[axis];
            if (items[i].bmax[axis] > bmax[axis]) bmax[axis] = items[i].bmax[axis];
        }
    }
}

static int longest_axis(int x, int y, int z) {
    int axis = 0;
    int max_value = x;
    if (y > max_value) {
        axis = 1;
        max_value = y;
    }
    if (z > max_value) axis = 2;
    return axis;
}

static void subdivide(NavMeshBvNode *items, int imin, int imax, int &current_node, NavMeshBvNode *nodes) {
    int inum = imax - imin;
    int icur = current_node;

    NavMeshBvNode &node = nodes[current_node++];

    if (inum == 1) {
        node = items[imin];
        return;
    }

    calc_extends(items, imin, imax, node.bmin, node.bmax);

    int axis = longest_axis(node.bmax[0] - node.bmin[0], node.bmax[1] - node.bmin[1], node.bmax[2] - node.bmin[2]);

    sort::stable(items + imin, (uint32_t)inum, [axis](const NavMeshBvNode &a, const NavMeshBvNode &b) { return a.bmin[axis] < b.bmin[axis]; });

    int isplit = imin + inum / 2;

    subdivide(items, imin, isplit, current_node, nodes);
    subdivide(items, isplit, imax, current_node, nodes);

    nodes[icur].i = -(current_node - icur);
}

static Box3 expand(Box3 bounds, Vec3 point) {
    if (point.x < bounds.min.x) bounds.min.x = point.x;
    if (point.y < bounds.min.y) bounds.min.y = point.y;
    if (point.z < bounds.min.z) bounds.min.z = point.z;
    if (point.x > bounds.max.x) bounds.max.x = point.x;
    if (point.y > bounds.max.y) bounds.max.y = point.y;
    if (point.z > bounds.max.z) bounds.max.z = point.z;
    return bounds;
}

static uint16_t quantize(float value, float origin, float quant_factor) {
    float quantized = (value - origin) * quant_factor;
    float nearest = roundf(quantized);
    if (fabsf(quantized - nearest) < SNAP_EPSILON) quantized = nearest;
    int truncated = (int)quantized;
    return (uint16_t)(truncated < 0 ? 0 : truncated > 0xffff ? 0xffff : truncated);
}

NavMeshTileBvTree build_nav_mesh_bv_tree(const NavMeshTileParams &params) {
    NavMeshTileBvTree tree = {};
    tree.quant_factor = 1.0f / params.cell_size;

    uint32_t poly_count = params.polys.length;
    if (poly_count == 0) return tree;

    Array<NavMeshBvNode> items = {};
    items.resize(poly_count, {});

    for (uint32_t i = 0; i < poly_count; i++) {
        NavMeshBvNode &item = items[i];
        item.i = (int)i;

        const NavMeshPoly &poly = params.polys[i];
        if (poly.vertex_count == 0) continue;

        Vec3 first = vec3::load(params.vertices.data, params.poly_vertices[poly.vertex_base] * 3);
        Box3 bounds = {first, first};

        for (uint32_t j = 1; j < poly.vertex_count; j++) {
            uint32_t vertex_index = params.poly_vertices[poly.vertex_base + j];
            bounds = expand(bounds, vec3::load(params.vertices.data, vertex_index * 3));
        }

        if (params.detail_meshes.length > 0 && params.detail_vertices.length > 0) {
            const NavMeshPolyDetail &detail_mesh = params.detail_meshes[i];
            for (uint32_t j = 0; j < detail_mesh.vertices_count; j++) {
                bounds = expand(bounds, vec3::load(params.detail_vertices.data, (detail_mesh.vertices_base + j) * 3));
            }
        }

        item.bmin[0] = quantize(bounds.min.x, params.bounds.min.x, tree.quant_factor);
        item.bmin[1] = quantize(bounds.min.y, params.bounds.min.y, tree.quant_factor);
        item.bmin[2] = quantize(bounds.min.z, params.bounds.min.z, tree.quant_factor);
        item.bmax[0] = quantize(bounds.max.x, params.bounds.min.x, tree.quant_factor);
        item.bmax[1] = quantize(bounds.max.y, params.bounds.min.y, tree.quant_factor);
        item.bmax[2] = quantize(bounds.max.z, params.bounds.min.z, tree.quant_factor);
    }

    tree.nodes.resize(poly_count * 2, {});
    int current_node = 0;
    subdivide(items.data, 0, (int)poly_count, current_node, tree.nodes.data);
    tree.nodes.resize((uint32_t)current_node);

    items.release();
    return tree;
}

}

}
