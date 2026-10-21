#pragma once

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "roam/util/array.h"

struct DetailMeshRange {
    int vertex_base;
    int vertex_count;
    int triangle_base;
    int triangle_count;
};

enum DetailMeshMatch {
    DETAIL_MESH_SAME,
    DETAIL_MESH_TIED,
    DETAIL_MESH_DIFFERENT,
};

inline int rotate_edge_flags(int flags, int steps) {
    int rotated = flags;
    for (int i = 0; i < steps; i++) rotated = ((rotated >> 2) | ((rotated & 0x3) << 4)) & 0x3f;
    return rotated;
}

inline bool same_triangle(const int *actual, const int32_t *expected) {
    for (int steps = 0; steps < 3; steps++) {
        bool vertices_match = actual[0] == expected[steps % 3] && actual[1] == expected[(steps + 1) % 3] && actual[2] == expected[(steps + 2) % 3];
        if (vertices_match && actual[3] == rotate_edge_flags(expected[3], steps)) return true;
    }
    return false;
}

inline DetailMeshMatch compare_detail_mesh(DetailMeshRange actual, const float *actual_vertices, const int *actual_triangles, DetailMeshRange expected,
                                           const double *expected_vertices, const int32_t *expected_triangles) {
    if (actual.vertex_count != expected.vertex_count) {
        return abs(actual.vertex_count - expected.vertex_count) <= 1 ? DETAIL_MESH_TIED : DETAIL_MESH_DIFFERENT;
    }
    if (actual.triangle_count != expected.triangle_count) return DETAIL_MESH_TIED;

    for (int v = 0; v < actual.vertex_count * 3; v++) {
        if (fabs((double)actual_vertices[actual.vertex_base * 3 + v] - expected_vertices[expected.vertex_base * 3 + v]) > 1e-4) {
            return DETAIL_MESH_TIED;
        }
    }

    for (int t = 0; t < actual.triangle_count; t++) {
        if (!same_triangle(&actual_triangles[(actual.triangle_base + t) * 4], &expected_triangles[(expected.triangle_base + t) * 4])) {
            return DETAIL_MESH_TIED;
        }
    }
    return DETAIL_MESH_SAME;
}

inline DetailMeshMatch compare_detail_mesh(DetailMeshRange actual, const float *actual_vertices, const uint8_t *actual_triangles, DetailMeshRange expected,
                                           const double *expected_vertices, const int32_t *expected_triangles) {
    roam::Array<int> widened = {};
    for (int i = 0; i < actual.triangle_count * 4; i++) widened.push(actual_triangles[actual.triangle_base * 4 + i]);
    DetailMeshRange local = {actual.vertex_base, actual.vertex_count, 0, actual.triangle_count};
    DetailMeshMatch match = compare_detail_mesh(local, actual_vertices, widened.data, expected, expected_vertices, expected_triangles);
    widened.release();
    return match;
}
