#include "roam/generate/poly_neighbours.h"

#include "roam/generate/common.h"

namespace roam {

namespace poly_neighbours {

void build_poly_neighbours(const Array<NavMeshPoly> &polys, const Array<uint32_t> &poly_vertices, Array<int> &poly_neis, Span<const float> vertices,
                           int border_size, float min_x, float min_z, float max_x, float max_z) {
    poly_neis.clear();
    poly_neis.resize(poly_vertices.length, MESH_NULL_IDX);

    build_mesh_adjacency(polys, poly_vertices, poly_neis, vertices.length / 3);

    if (border_size > 0) find_portal_edges(polys, poly_vertices, poly_neis, vertices, min_x, min_z, max_x, max_z);

    finalize_poly_neighbours(poly_neis);
}

struct Edge {
    uint32_t vertex[2];
    uint32_t poly_edge[2];
    uint32_t poly[2];
};

void build_mesh_adjacency(const Array<NavMeshPoly> &polys, const Array<uint32_t> &poly_vertices, Array<int> &poly_neis, uint32_t vertex_count) {
    Array<int> first_edge = {};
    Array<int> next_edge = {};
    Array<Edge> edges = {};
    first_edge.resize(vertex_count, MESH_NULL_IDX);
    next_edge.resize(poly_vertices.length, MESH_NULL_IDX);

    for (uint32_t i = 0; i < polys.length; i++) {
        const NavMeshPoly &poly = polys[i];
        for (uint32_t j = 0; j < poly.vertex_count; j++) {
            uint32_t v0 = poly_vertices[poly.vertex_base + j];
            uint32_t v1 = poly_vertices[poly.vertex_base + (j + 1) % poly.vertex_count];
            if (v0 >= v1) continue;

            next_edge[edges.length] = first_edge[v0];
            first_edge[v0] = (int)edges.length;
            edges.push({{v0, v1}, {j, 0}, {i, i}});
        }
    }

    for (uint32_t i = 0; i < polys.length; i++) {
        const NavMeshPoly &poly = polys[i];
        for (uint32_t j = 0; j < poly.vertex_count; j++) {
            uint32_t v0 = poly_vertices[poly.vertex_base + j];
            uint32_t v1 = poly_vertices[poly.vertex_base + (j + 1) % poly.vertex_count];
            if (v0 <= v1) continue;

            for (int e = first_edge[v1]; e != MESH_NULL_IDX; e = next_edge[(uint32_t)e]) {
                Edge &edge = edges[(uint32_t)e];
                if (edge.vertex[1] == v0 && edge.poly[0] == edge.poly[1]) {
                    edge.poly[1] = i;
                    edge.poly_edge[1] = j;
                    break;
                }
            }
        }
    }

    for (const Edge &edge : edges) {
        if (edge.poly[0] == edge.poly[1]) continue;
        poly_neis[polys[edge.poly[0]].vertex_base + edge.poly_edge[0]] = (int)edge.poly[1];
        poly_neis[polys[edge.poly[1]].vertex_base + edge.poly_edge[1]] = (int)edge.poly[0];
    }

    first_edge.release();
    next_edge.release();
    edges.release();
}

void find_portal_edges(const Array<NavMeshPoly> &polys, const Array<uint32_t> &poly_vertices, Array<int> &poly_neis, Span<const float> vertices,
                       float min_x, float min_z, float max_x, float max_z) {
    for (const NavMeshPoly &poly : polys) {
        for (uint32_t j = 0; j < poly.vertex_count; j++) {
            int &nei = poly_neis[poly.vertex_base + j];
            if (nei != MESH_NULL_IDX) continue;

            uint32_t nj = (j + 1) % poly.vertex_count;
            const float *va = &vertices[poly_vertices[poly.vertex_base + j] * 3];
            const float *vb = &vertices[poly_vertices[poly.vertex_base + nj] * 3];

            if (va[0] == min_x && vb[0] == min_x) {
                nei = POLY_NEIS_FLAG_EXT_LINK | 0;
            } else if (va[2] == max_z && vb[2] == max_z) {
                nei = POLY_NEIS_FLAG_EXT_LINK | 1;
            } else if (va[0] == max_x && vb[0] == max_x) {
                nei = POLY_NEIS_FLAG_EXT_LINK | 2;
            } else if (va[2] == min_z && vb[2] == min_z) {
                nei = POLY_NEIS_FLAG_EXT_LINK | 3;
            }
        }
    }
}

void finalize_poly_neighbours(Array<int> &poly_neis) {
    for (int &nei : poly_neis) {
        if (!(nei & POLY_NEIS_FLAG_EXT_LINK)) {
            nei = nei + 1;
            continue;
        }

        int direction = nei & 0xf;
        if (direction == 0) {
            nei = POLY_NEIS_FLAG_EXT_LINK | 4;
        } else if (direction == 1) {
            nei = POLY_NEIS_FLAG_EXT_LINK | 2;
        } else if (direction == 2) {
            nei = POLY_NEIS_FLAG_EXT_LINK | 0;
        } else if (direction == 3) {
            nei = POLY_NEIS_FLAG_EXT_LINK | 6;
        } else {
            nei = 0;
        }
    }
}

}

}
