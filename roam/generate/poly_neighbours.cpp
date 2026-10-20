#include "roam/generate/poly_neighbours.h"

#include "roam/generate/common.h"

namespace roam {

namespace poly_neighbours {

void build_poly_neighbours(Array<NavMeshPoly> &polys, const float *vertices, uint32_t vertex_count, int border_size, float min_x, float min_z,
                           float max_x, float max_z) {
    for (NavMeshPoly &poly : polys) {
        poly.neis.clear();
        poly.neis.resize(poly.vertices.length, MESH_NULL_IDX);
    }

    build_mesh_adjacency(polys, vertex_count);

    if (border_size > 0) find_portal_edges(polys, vertices, min_x, min_z, max_x, max_z);

    finalize_poly_neighbours(polys);
}

struct Edge {
    uint32_t vertex[2];
    uint32_t poly_edge[2];
    uint32_t poly[2];
};

void build_mesh_adjacency(Array<NavMeshPoly> &polys, uint32_t vertex_count) {
    uint32_t max_edge_count = 0;
    for (const NavMeshPoly &poly : polys) max_edge_count += poly.vertices.length;

    Array<int> first_edge = {};
    Array<int> next_edge = {};
    Array<Edge> edges = {};
    first_edge.resize(vertex_count, MESH_NULL_IDX);
    next_edge.resize(max_edge_count, MESH_NULL_IDX);

    for (uint32_t i = 0; i < polys.length; i++) {
        const NavMeshPoly &poly = polys[i];
        for (uint32_t j = 0; j < poly.vertices.length; j++) {
            uint32_t v0 = poly.vertices[j];
            uint32_t v1 = poly.vertices[(j + 1) % poly.vertices.length];
            if (v0 >= v1) continue;

            next_edge[edges.length] = first_edge[v0];
            first_edge[v0] = (int)edges.length;
            edges.push({{v0, v1}, {j, 0}, {i, i}});
        }
    }

    for (uint32_t i = 0; i < polys.length; i++) {
        const NavMeshPoly &poly = polys[i];
        for (uint32_t j = 0; j < poly.vertices.length; j++) {
            uint32_t v0 = poly.vertices[j];
            uint32_t v1 = poly.vertices[(j + 1) % poly.vertices.length];
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
        polys[edge.poly[0]].neis[edge.poly_edge[0]] = (int)edge.poly[1];
        polys[edge.poly[1]].neis[edge.poly_edge[1]] = (int)edge.poly[0];
    }

    first_edge.release();
    next_edge.release();
    edges.release();
}

void find_portal_edges(Array<NavMeshPoly> &polys, const float *vertices, float min_x, float min_z, float max_x, float max_z) {
    for (NavMeshPoly &poly : polys) {
        for (uint32_t j = 0; j < poly.vertices.length; j++) {
            if (poly.neis[j] != MESH_NULL_IDX) continue;

            uint32_t nj = (j + 1) % poly.vertices.length;
            const float *va = &vertices[poly.vertices[j] * 3];
            const float *vb = &vertices[poly.vertices[nj] * 3];

            if (va[0] == min_x && vb[0] == min_x) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 0;
            } else if (va[2] == max_z && vb[2] == max_z) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 1;
            } else if (va[0] == max_x && vb[0] == max_x) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 2;
            } else if (va[2] == min_z && vb[2] == min_z) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 3;
            }
        }
    }
}

void finalize_poly_neighbours(Array<NavMeshPoly> &polys) {
    for (NavMeshPoly &poly : polys) {
        for (int &nei : poly.neis) {
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

}
