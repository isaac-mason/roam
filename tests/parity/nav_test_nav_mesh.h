#pragma once

#include <stdio.h>

#include "roam/generate/common.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"
#include "tests/harness/fixture.h"
#include "tests/harness/test.h"

using namespace roam;

inline NodeRef ref_from_f64(double value) { return value < 0 ? INVALID_NODE_REF : (NodeRef)value; }

struct NavTestTile {
    NavMeshTilePolys polys;
    NavMeshTileDetailMesh detail_mesh;
    NavMeshTileParams params;
};

inline NavTestTile load_tile(const fixture::Fixture &expected) {
    NavTestTile tile = {};
    NavMeshTileParams &params = tile.params;

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

    for (uint32_t i = 0; i < vertices.count; i++) tile.polys.vertices.push((float)vertices.data[i]);
    uint32_t base = 0;
    for (uint32_t i = 0; i < vertex_counts.count; i++) {
        uint8_t area = (uint8_t)poly_areas.data[i];
        if (area == WALKABLE_AREA) area = 0;
        tile.polys.polys.push({base, (uint32_t)vertex_counts.data[i], area == 0 ? 1u : 0u, area});
        base += (uint32_t)vertex_counts.data[i];
    }
    for (uint32_t i = 0; i < poly_vertices.count; i++) tile.polys.poly_vertices.push((uint32_t)poly_vertices.data[i]);
    for (uint32_t i = 0; i < poly_neis.count; i++) tile.polys.poly_neis.push(poly_neis.data[i]);
    for (uint32_t i = 0; i < detail_meshes.count; i += 4) {
        tile.detail_mesh.detail_meshes.push({(uint32_t)detail_meshes.data[i], (uint32_t)detail_meshes.data[i + 1], (uint32_t)detail_meshes.data[i + 2],
                                   (uint32_t)detail_meshes.data[i + 3]});
    }
    for (uint32_t i = 0; i < detail_vertices.count; i++) tile.detail_mesh.detail_vertices.push((float)detail_vertices.data[i]);
    for (uint32_t i = 0; i < detail_triangles.count; i++) tile.detail_mesh.detail_triangles.push(detail_triangles.data[i]);

    params.bounds = {{(float)bounds.data[0], (float)bounds.data[1], (float)bounds.data[2]}, {(float)bounds.data[3], (float)bounds.data[4], (float)bounds.data[5]}};
    params.cell_size = (float)fixture::number(expected, "options.cell_size");
    params.cell_height = (float)fixture::number(expected, "options.cell_height");
    params.walkable_height = (float)walkable.data[0];
    params.walkable_radius = (float)walkable.data[1];
    params.walkable_climb = (float)walkable.data[2];
    params.vertices = tile.polys.vertices;
    params.polys = tile.polys.polys;
    params.poly_vertices = tile.polys.poly_vertices;
    params.poly_neis = tile.polys.poly_neis;
    params.detail_meshes = tile.detail_mesh.detail_meshes;
    params.detail_vertices = tile.detail_mesh.detail_vertices;
    params.detail_triangles = tile.detail_mesh.detail_triangles;
    return tile;
}

inline void release(NavTestTile &tile) {
    nav_mesh_tile::release(tile.polys);
    nav_mesh_tile::release(tile.detail_mesh);
}

inline NavMesh load_nav_test_nav_mesh(const fixture::Fixture &expected, bool with_off_mesh) {
    NavTestTile tile = load_tile(expected);
    const NavMeshTileParams &params = tile.params;
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = params.bounds.min;
    nav_mesh.tile_width = params.bounds.max.x - params.bounds.min.x;
    nav_mesh.tile_height = params.bounds.max.z - params.bounds.min.z;
    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));
    release(tile);

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

inline void check_nav_mesh(const char *prefix, const NavMesh &nav_mesh, const fixture::Fixture &expected) {
    char name[256];
    snprintf(name, sizeof(name), "%s.node_refs", prefix);
    fixture::F64 node_refs = fixture::f64(expected, name);
    snprintf(name, sizeof(name), "%s.node_allocated", prefix);
    fixture::I32 node_allocated = fixture::i32(expected, name);
    snprintf(name, sizeof(name), "%s.link_counts", prefix);
    fixture::I32 link_counts = fixture::i32(expected, name);
    snprintf(name, sizeof(name), "%s.link_to_refs", prefix);
    fixture::F64 link_to_refs = fixture::f64(expected, name);
    snprintf(name, sizeof(name), "%s.link_fields", prefix);
    fixture::I32 link_fields = fixture::i32(expected, name);

    REQUIRE(nav_mesh.nodes.length == node_refs.count);

    uint32_t link_cursor = 0;
    uint32_t mismatched_nodes = 0;
    for (uint32_t i = 0; i < nav_mesh.nodes.length; i++) {
        const NavMeshNode &node = nav_mesh.nodes[i];
        bool same = node.allocated == (node_allocated.data[i] != 0) && (!node.allocated || node.ref == ref_from_f64(node_refs.data[i]));

        int32_t count = 0;
        for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
            const NavMeshLink &navmesh_link = nav_mesh.links[(uint32_t)link];
            uint32_t at = link_cursor + (uint32_t)count;
            if (count < link_counts.data[i]) {
                same = same && navmesh_link.to_node_ref == ref_from_f64(link_to_refs.data[at]) && navmesh_link.edge == link_fields.data[at * 4] &&
                       navmesh_link.side == link_fields.data[at * 4 + 1] && navmesh_link.bmin == link_fields.data[at * 4 + 2] &&
                       navmesh_link.bmax == link_fields.data[at * 4 + 3];
            }
            count++;
        }
        same = same && count == link_counts.data[i];
        link_cursor += (uint32_t)link_counts.data[i];
        if (!same) mismatched_nodes++;
    }
    CHECK(mismatched_nodes == 0);
}
