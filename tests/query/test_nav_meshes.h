#pragma once

#include <math.h>

#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/util/sort.h"

using namespace roam;

inline roam::NavMesh create_nav_mesh_from_triangles(const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count) {
    Box3 bounds = box3::empty();
    for (uint32_t i = 0; i < vertex_count; i++) bounds = box3::expand_by_point(bounds, vec3::load(positions, i * 3));

    Array<ExternalPolygon> polygons = {};
    for (uint32_t i = 0; i < index_count; i += 3) polygons.push({&indices[i], 3, 1, 0});

    NavMeshTilePolys tile_polys = nav_mesh_tile::polygons_to_nav_mesh_tile_polys(polygons.data, polygons.length, positions, vertex_count, 0, bounds);
    NavMeshTileDetailMesh detail_mesh = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);

    NavMeshTileParams params = {};
    params.bounds = bounds;
    params.vertices = tile_polys.vertices;
    params.polys = tile_polys.polys;
    params.poly_vertices = tile_polys.poly_vertices;
    params.poly_neis = tile_polys.poly_neis;
    params.detail_meshes = detail_mesh.detail_meshes;
    params.detail_vertices = detail_mesh.detail_vertices;
    params.detail_triangles = detail_mesh.detail_triangles;
    params.cell_size = 0.2f;
    params.cell_height = 0.2f;
    params.walkable_height = 0.5f;
    params.walkable_radius = 0.5f;
    params.walkable_climb = 0.5f;

    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = bounds.min;
    nav_mesh.tile_width = bounds.max.x - bounds.min.x;
    nav_mesh.tile_height = bounds.max.z - bounds.min.z;

    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(detail_mesh);
    polygons.release();
    return nav_mesh;
}

inline NavMesh create_tiled_terrain() {
    const int size = 24;
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    for (int z = 0; z <= size; z++) {
        for (int x = 0; x <= size; x++) {
            float height = 0.4f * sinf((float)x * 0.5f) * cosf((float)z * 0.4f);
            positions.push((float)x);
            positions.push(height);
            positions.push((float)z);
        }
    }
    for (int z = 0; z < size; z++) {
        for (int x = 0; x < size; x++) {
            if (x > 9 && x < 13 && z > 6 && z < 15) continue;
            uint32_t a = (uint32_t)(z * (size + 1) + x);
            uint32_t b = a + 1;
            uint32_t c = a + (uint32_t)(size + 1);
            uint32_t d = c + 1;
            const uint32_t quad[] = {a, c, b, b, c, d};
            for (uint32_t index : quad) indices.push(index);
        }
    }

    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    TiledNavMeshOptions options = {
        .cell_size = cell_size,
        .cell_height = cell_height,
        .tile_size_voxels = 32,
        .tile_size_world = 32 * cell_size,
        .walkable_radius_voxels = 1,
        .walkable_radius_world = 0.2f,
        .walkable_climb_voxels = 3,
        .walkable_climb_world = 0.6f,
        .walkable_height_voxels = 5,
        .walkable_height_world = 1.0f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 4,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = cell_size * 6,
        .detail_sample_max_error = cell_height,
    };
    TiledNavMeshResult result = generate_tiled_nav_mesh::generate_tiled_nav_mesh({positions, indices}, options);
    generate_tiled_nav_mesh::release(result.intermediates);
    positions.release();
    indices.release();

    nav_mesh_api::add_off_mesh_connection(result.nav_mesh, {{2, 0, 2}, {20, 0, 20}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 1});
    return result.nav_mesh;
}

inline uint64_t poly_key(const NavMesh &nav_mesh, NodeRef ref) {
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, ref);
    if (node.type != NODE_TYPE_POLY) return 0xffffffffffffffffull;
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
    return ((uint64_t)(uint8_t)tile.tile_x << 56) | ((uint64_t)(uint8_t)tile.tile_y << 48) | ((uint64_t)(uint8_t)tile.tile_layer << 40) |
           (uint64_t)(uint32_t)node.poly_index;
}

struct LinkKey {
    uint64_t from;
    uint64_t to;
    uint32_t data;
};

inline Array<LinkKey> link_keys(const NavMesh &nav_mesh) {
    Array<LinkKey> keys = {};
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (!node.allocated || node.type != NODE_TYPE_POLY) continue;
        for (int32_t l = node.first_link; l != NULL_LINK; l = nav_mesh.links[(uint32_t)l].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)l];
            uint32_t data = ((uint32_t)link.edge << 24) | ((uint32_t)link.side << 16) | ((uint32_t)link.bmin << 8) | link.bmax;
            keys.push({poly_key(nav_mesh, node.ref), poly_key(nav_mesh, link.to_node_ref), data});
        }
    }
    sort::stable(keys.data, keys.length, [](const LinkKey &a, const LinkKey &b) {
        if (a.from != b.from) return a.from < b.from;
        if (a.to != b.to) return a.to < b.to;
        return a.data < b.data;
    });
    return keys;
}

inline bool same_links(const Array<LinkKey> &a, const Array<LinkKey> &b) {
    if (a.length != b.length) return false;
    for (uint32_t i = 0; i < a.length; i++) {
        if (a[i].from != b[i].from || a[i].to != b[i].to || a[i].data != b[i].data) return false;
    }
    return true;
}
