#pragma once

#include "roam/generate/common.h"
#include "roam/query/nav_mesh_api.h"
#include "tests/harness/fixture.h"

using namespace roam;

inline NodeRef ref_from_f64(double value) { return value < 0 ? INVALID_NODE_REF : (NodeRef)value; }

inline NavMeshTileParams load_tile_params(const fixture::Fixture &expected) {
    NavMeshTileParams params = {};

    fixture::F64 vertices = fixture::f64(expected, "tile.watershed.vertices");
    fixture::I32 vertex_counts = fixture::i32(expected, "tile.watershed.poly_vertex_counts");
    fixture::I32 poly_vertices = fixture::i32(expected, "tile.watershed.poly_vertices");
    fixture::I32 poly_neis = fixture::i32(expected, "tile.watershed.poly_neis");
    fixture::I32 poly_areas = fixture::i32(expected, "tile.watershed.poly_areas");
    fixture::I32 detail_meshes = fixture::i32(expected, "tile.watershed.detail_meshes");
    fixture::F64 detail_vertices = fixture::f64(expected, "tile.watershed.detail_vertices");
    fixture::I32 detail_triangles = fixture::i32(expected, "tile.watershed.detail_triangles");
    fixture::F64 walkable = fixture::f64(expected, "nav_mesh.walkable");
    fixture::F64 bounds = fixture::f64(expected, "poly_mesh.watershed.bounds");

    for (uint32_t i = 0; i < vertices.count; i++) params.vertices.push((float)vertices.data[i]);
    uint32_t base = 0;
    for (uint32_t i = 0; i < vertex_counts.count; i++) {
        uint8_t area = (uint8_t)poly_areas.data[i];
        if (area == WALKABLE_AREA) area = 0;
        params.polys.push({base, (uint32_t)vertex_counts.data[i], area == 0 ? 1u : 0u, area});
        base += (uint32_t)vertex_counts.data[i];
    }
    for (uint32_t i = 0; i < poly_vertices.count; i++) params.poly_vertices.push((uint32_t)poly_vertices.data[i]);
    for (uint32_t i = 0; i < poly_neis.count; i++) params.poly_neis.push(poly_neis.data[i]);
    for (uint32_t i = 0; i < detail_meshes.count; i += 4) {
        params.detail_meshes.push({(uint32_t)detail_meshes.data[i], (uint32_t)detail_meshes.data[i + 1], (uint32_t)detail_meshes.data[i + 2],
                                   (uint32_t)detail_meshes.data[i + 3]});
    }
    for (uint32_t i = 0; i < detail_vertices.count; i++) params.detail_vertices.push((float)detail_vertices.data[i]);
    for (uint32_t i = 0; i < detail_triangles.count; i++) params.detail_triangles.push(detail_triangles.data[i]);

    params.bounds = {{(float)bounds.data[0], (float)bounds.data[1], (float)bounds.data[2]}, {(float)bounds.data[3], (float)bounds.data[4], (float)bounds.data[5]}};
    params.cell_size = (float)fixture::number(expected, "options.cell_size");
    params.cell_height = (float)fixture::number(expected, "options.cell_height");
    params.walkable_height = (float)walkable.data[0];
    params.walkable_radius = (float)walkable.data[1];
    params.walkable_climb = (float)walkable.data[2];
    return params;
}

inline void release(NavMeshTileParams &params) {
    params.vertices.release();
    params.polys.release();
    params.poly_vertices.release();
    params.poly_neis.release();
    params.detail_meshes.release();
    params.detail_vertices.release();
    params.detail_triangles.release();
}


inline NavMesh load_nav_test_nav_mesh(const fixture::Fixture &expected, bool with_off_mesh) {
    NavMeshTileParams params = load_tile_params(expected);
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = params.bounds.min;
    nav_mesh.tile_width = params.bounds.max.x - params.bounds.min.x;
    nav_mesh.tile_height = params.bounds.max.z - params.bounds.min.z;
    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));
    release(params);

    if (with_off_mesh) {
        fixture::F64 start = fixture::f64(expected, "off_mesh.start");
        fixture::F64 end = fixture::f64(expected, "off_mesh.end");
        nav_mesh_api::add_off_mesh_connection(nav_mesh, {{(float)start.data[0], (float)start.data[1], (float)start.data[2]},
                                                         {(float)end.data[0], (float)end.data[1], (float)end.data[2]},
                                                         0.5f,
                                                         OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
                                                         1,
                                                         0});
    }
    return nav_mesh;
}
