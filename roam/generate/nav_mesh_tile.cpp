#include "roam/generate/nav_mesh_tile.h"

#include "roam/generate/common.h"
#include "roam/generate/poly_neighbours.h"

namespace roam {

namespace nav_mesh_tile {

NavMeshTilePolys poly_mesh_to_tile_polys(const PolyMesh &poly_mesh) {
    NavMeshTilePolys tile_polys = {};
    for (int value : poly_mesh.vertices) tile_polys.vertices.push((float)value);

    int nvp = poly_mesh.max_vertices_per_poly;

    for (int i = 0; i < poly_mesh.poly_count; i++) {
        NavMeshPoly poly = {};
        poly.vertex_base = tile_polys.poly_vertices.length;
        poly.flags = poly_mesh.flags[(uint32_t)i];
        poly.area = poly_mesh.areas[(uint32_t)i];

        for (int j = 0; j < nvp; j++) {
            int vertex_index = poly_mesh.polys[(uint32_t)(i * nvp + j)];
            if (vertex_index == MESH_NULL_IDX) break;
            tile_polys.poly_vertices.push((uint32_t)vertex_index);
            poly.vertex_count++;
        }

        tile_polys.polys.push(poly);
    }

    poly_neighbours::build_poly_neighbours(tile_polys.polys, tile_polys.poly_vertices, tile_polys.poly_neis, tile_polys.vertices.data, tile_polys.vertices.length / 3, poly_mesh.border_size, 0, 0,
                                           (float)poly_mesh.local_width, (float)poly_mesh.local_height);

    for (uint32_t i = 0; i < tile_polys.vertices.length; i += 3) {
        tile_polys.vertices[i] = poly_mesh.bounds.min.x + tile_polys.vertices[i] * poly_mesh.cell_size;
        tile_polys.vertices[i + 1] = poly_mesh.bounds.min.y + tile_polys.vertices[i + 1] * poly_mesh.cell_height;
        tile_polys.vertices[i + 2] = poly_mesh.bounds.min.z + tile_polys.vertices[i + 2] * poly_mesh.cell_size;
    }

    return tile_polys;
}

NavMeshTilePolys polygons_to_nav_mesh_tile_polys(const ExternalPolygon *polygons, uint32_t polygon_count, const float *vertices,
                                                 uint32_t vertex_count, int border_size, Box3 bounds) {
    NavMeshTilePolys tile_polys = {};
    for (uint32_t i = 0; i < vertex_count * 3; i++) tile_polys.vertices.push(vertices[i]);

    for (uint32_t i = 0; i < polygon_count; i++) {
        tile_polys.polys.push({tile_polys.poly_vertices.length, polygons[i].vertex_count, polygons[i].flags, polygons[i].area});
        for (uint32_t j = 0; j < polygons[i].vertex_count; j++) tile_polys.poly_vertices.push(polygons[i].vertices[j]);
    }

    poly_neighbours::build_poly_neighbours(tile_polys.polys, tile_polys.poly_vertices, tile_polys.poly_neis, tile_polys.vertices.data, vertex_count, border_size, bounds.min.x, bounds.min.z,
                                           bounds.max.x, bounds.max.z);

    return tile_polys;
}

NavMeshTileDetailMesh polys_to_tile_detail_mesh(const Array<NavMeshPoly> &polys) {
    NavMeshTileDetailMesh detail_mesh = {};

    uint32_t triangles_base = 0;

    for (const NavMeshPoly &poly : polys) {
        uint32_t vertex_count = poly.vertex_count;

        detail_mesh.detail_meshes.push({0, triangles_base, 0, vertex_count - 2});

        for (uint32_t j = 2; j < vertex_count; j++) {
            ROAM_ASSERT(j <= 0xff);
            detail_mesh.detail_triangles.push(0);
            detail_mesh.detail_triangles.push((uint8_t)(j - 1));
            detail_mesh.detail_triangles.push((uint8_t)j);

            uint8_t edge_flags = 1 << 2;
            if (j == 2) edge_flags |= 1 << 0;
            if (j == vertex_count - 1) edge_flags |= 1 << 4;

            detail_mesh.detail_triangles.push(edge_flags);
            triangles_base++;
        }
    }

    return detail_mesh;
}

NavMeshTileDetailMesh poly_mesh_detail_to_tile_detail_mesh(const Array<NavMeshPoly> &polys, const PolyMeshDetail &poly_mesh_detail) {
    NavMeshTileDetailMesh detail_mesh = {};

    uint32_t vertices_base = 0;

    for (uint32_t i = 0; i < polys.length; i++) {
        int poly_vertex_count = (int)polys[i].vertex_count;

        int vb = poly_mesh_detail.meshes[i * 4];
        int detail_vertex_count = poly_mesh_detail.meshes[i * 4 + 1];
        int triangles_base = poly_mesh_detail.meshes[i * 4 + 2];
        int triangles_count = poly_mesh_detail.meshes[i * 4 + 3];

        int additional_vertex_count = detail_vertex_count - poly_vertex_count;

        detail_mesh.detail_meshes.push({vertices_base, (uint32_t)triangles_base, (uint32_t)additional_vertex_count, (uint32_t)triangles_count});

        if (additional_vertex_count <= 0) continue;

        for (int j = poly_vertex_count; j < detail_vertex_count; j++) {
            uint32_t detail_vertex_index = (uint32_t)((vb + j) * 3);
            detail_mesh.detail_vertices.push(poly_mesh_detail.vertices[detail_vertex_index]);
            detail_mesh.detail_vertices.push(poly_mesh_detail.vertices[detail_vertex_index + 1]);
            detail_mesh.detail_vertices.push(poly_mesh_detail.vertices[detail_vertex_index + 2]);
        }

        vertices_base += (uint32_t)additional_vertex_count;
    }

    detail_mesh.detail_triangles.reserve(poly_mesh_detail.triangles.length);
    for (int value : poly_mesh_detail.triangles) {
        ROAM_ASSERT(value >= 0 && value <= 0xff);
        detail_mesh.detail_triangles.push((uint8_t)value);
    }

    return detail_mesh;
}

void release(NavMeshTilePolys &tile_polys) {
    tile_polys.vertices.release();
    tile_polys.polys.release();
    tile_polys.poly_vertices.release();
    tile_polys.poly_neis.release();
}

void release(NavMeshTileDetailMesh &detail_mesh) {
    detail_mesh.detail_meshes.release();
    detail_mesh.detail_vertices.release();
    detail_mesh.detail_triangles.release();
}

}

}
