#pragma once

#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"

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

