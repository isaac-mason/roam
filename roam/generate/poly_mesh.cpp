#include "roam/generate/poly_mesh.h"

#include "roam/generate/common.h"
#include "roam/math/scalar.h"
#include "roam/util/map.h"

namespace roam {

namespace poly_mesh {

static const int VERTEX_Y_TOLERANCE = 2;
static const uint32_t REMOVABLE_FLAG = 0x80000000u;
static const uint32_t INDEX_MASK = 0x0fffffffu;

struct VertexEntry {
    int y;
    int index;
    int32_t next;
};

struct VertexBucket {
    int32_t head;
    int32_t tail;
};

struct VertexMap {
    Map<uint64_t, VertexBucket> buckets;
    Array<VertexEntry> entries;
};

static uint64_t vertex_key(int x, int z) { return ((uint64_t)(uint32_t)x << 32) | (uint64_t)(uint32_t)z; }

static int add_vertex(int x, int y, int z, Array<int> &vertices, Array<int> &vertex_flags, VertexMap &vertex_map) {
    uint64_t key = vertex_key(x, z);
    VertexBucket *bucket = vertex_map.buckets.get(key);

    if (bucket) {
        for (int32_t entry = bucket->head; entry != -1; entry = vertex_map.entries[(uint32_t)entry].next) {
            const VertexEntry &vertex_entry = vertex_map.entries[(uint32_t)entry];
            if (scalar::abs(vertex_entry.y - y) <= VERTEX_Y_TOLERANCE) return vertex_entry.index;
        }
    }

    int i = (int)vertices.length / 3;
    vertices.push(x);
    vertices.push(y);
    vertices.push(z);

    int32_t entry = (int32_t)vertex_map.entries.length;
    vertex_map.entries.push({y, i, -1});

    if (bucket) {
        vertex_map.entries[(uint32_t)bucket->tail].next = entry;
        bucket->tail = entry;
    } else {
        vertex_map.buckets.set(key, {entry, entry});
    }

    vertex_flags.push(0);

    return i;
}

static int previous(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
static int next(int i, int n) { return i + 1 < n ? i + 1 : 0; }

struct Point {
    int x;
    int z;
};

static int area2(Point a, Point b, Point c) { return (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z); }

static bool xorb(bool x, bool y) { return !x != !y; }

static bool left(Point a, Point b, Point c) { return area2(a, b, c) < 0; }

static bool left_on(Point a, Point b, Point c) { return area2(a, b, c) <= 0; }

static bool collinear(Point a, Point b, Point c) { return area2(a, b, c) == 0; }

static bool intersect_prop(Point a, Point b, Point c, Point d) {
    if (collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b)) return false;
    return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
}

static bool between(Point a, Point b, Point c) {
    if (!collinear(a, b, c)) return false;
    if (a.x != b.x) return (a.x <= c.x && c.x <= b.x) || (a.x >= c.x && c.x >= b.x);
    return (a.z <= c.z && c.z <= b.z) || (a.z >= c.z && c.z >= b.z);
}

static bool intersect(Point a, Point b, Point c, Point d) {
    if (intersect_prop(a, b, c, d)) return true;
    return between(a, b, c) || between(a, b, d) || between(c, d, a) || between(c, d, b);
}

static bool equal_xz(Point a, Point b) { return a.x == b.x && a.z == b.z; }

static Point indexed_point(const int *vertices, const uint32_t *indices, int i) {
    uint32_t vertex = (indices[i] & INDEX_MASK) * 4;
    return {vertices[vertex], vertices[vertex + 2]};
}

template <bool loose>
static bool diagonalie(int start, int end, int vertex_count, const int *vertices, const uint32_t *indices) {
    Point diagonal_start = indexed_point(vertices, indices, start);
    Point diagonal_end = indexed_point(vertices, indices, end);

    for (int k = 0; k < vertex_count; k++) {
        int k1 = next(k, vertex_count);
        if (k == start || k1 == start || k == end || k1 == end) continue;

        Point edge_start = indexed_point(vertices, indices, k);
        Point edge_end = indexed_point(vertices, indices, k1);

        if (equal_xz(diagonal_start, edge_start) || equal_xz(diagonal_end, edge_start) || equal_xz(diagonal_start, edge_end) ||
            equal_xz(diagonal_end, edge_end)) {
            continue;
        }

        bool crosses = loose ? intersect_prop(diagonal_start, diagonal_end, edge_start, edge_end)
                             : intersect(diagonal_start, diagonal_end, edge_start, edge_end);
        if (crosses) return false;
    }
    return true;
}

template <bool loose>
static bool in_cone(int cone, int test, int vertex_count, const int *vertices, const uint32_t *indices) {
    Point cone_vertex = indexed_point(vertices, indices, cone);
    Point test_vertex = indexed_point(vertices, indices, test);
    Point next_vertex = indexed_point(vertices, indices, next(cone, vertex_count));
    Point previous_vertex = indexed_point(vertices, indices, previous(cone, vertex_count));

    if (left_on(previous_vertex, cone_vertex, next_vertex)) {
        if (loose) return left_on(cone_vertex, test_vertex, previous_vertex) && left_on(test_vertex, cone_vertex, next_vertex);
        return left(cone_vertex, test_vertex, previous_vertex) && left(test_vertex, cone_vertex, next_vertex);
    }
    return !(left_on(cone_vertex, test_vertex, next_vertex) && left_on(test_vertex, cone_vertex, previous_vertex));
}

static bool diagonal(int start, int end, int vertex_count, const int *vertices, const uint32_t *indices) {
    return in_cone<false>(start, end, vertex_count, vertices, indices) && diagonalie<false>(start, end, vertex_count, vertices, indices);
}

static bool diagonal_loose(int start, int end, int vertex_count, const int *vertices, const uint32_t *indices) {
    return in_cone<true>(start, end, vertex_count, vertices, indices) && diagonalie<true>(start, end, vertex_count, vertices, indices);
}

static int squared_xz_distance(const int *vertices, uint32_t a, uint32_t b) {
    int dx = vertices[(b & INDEX_MASK) * 4] - vertices[(a & INDEX_MASK) * 4];
    int dz = vertices[(b & INDEX_MASK) * 4 + 2] - vertices[(a & INDEX_MASK) * 4 + 2];
    return dx * dx + dz * dz;
}

static int triangulate(int vertex_count, const int *vertices, uint32_t *indices, int *triangles) {
    int triangle_count = 0;
    int destination = 0;

    for (int i = 0; i < vertex_count; i++) {
        int i1 = next(i, vertex_count);
        int i2 = next(i1, vertex_count);
        if (diagonal(i, i2, vertex_count, vertices, indices)) indices[i1] |= REMOVABLE_FLAG;
    }

    int n = vertex_count;
    while (n > 3) {
        int min_length = -1;
        int min_i = -1;

        for (int i = 0; i < n; i++) {
            int i1 = next(i, n);
            if (!(indices[i1] & REMOVABLE_FLAG)) continue;
            int length = squared_xz_distance(vertices, indices[i], indices[next(i1, n)]);
            if (min_length < 0 || length < min_length) {
                min_length = length;
                min_i = i;
            }
        }

        if (min_i == -1) {
            for (int i = 0; i < n; i++) {
                int i1 = next(i, n);
                int i2 = next(i1, n);
                if (!diagonal_loose(i, i2, n, vertices, indices)) continue;
                int length = squared_xz_distance(vertices, indices[i], indices[next(i2, n)]);
                if (min_length < 0 || length < min_length) {
                    min_length = length;
                    min_i = i;
                }
            }
            if (min_i == -1) return -triangle_count;
        }

        int i = min_i;
        int i1 = next(i, n);
        int i2 = next(i1, n);

        triangles[destination++] = (int)(indices[i] & INDEX_MASK);
        triangles[destination++] = (int)(indices[i1] & INDEX_MASK);
        triangles[destination++] = (int)(indices[i2] & INDEX_MASK);
        triangle_count++;

        n--;
        for (int k = i1; k < n; k++) indices[k] = indices[k + 1];

        if (i1 >= n) i1 = 0;
        int i_previous = previous(i1, n);

        if (diagonal(previous(i_previous, n), i1, n, vertices, indices)) {
            indices[i_previous] |= REMOVABLE_FLAG;
        } else {
            indices[i_previous] &= INDEX_MASK;
        }

        if (diagonal(i_previous, next(i1, n), n, vertices, indices)) {
            indices[i1] |= REMOVABLE_FLAG;
        } else {
            indices[i1] &= INDEX_MASK;
        }
    }

    triangles[destination++] = (int)(indices[0] & INDEX_MASK);
    triangles[destination++] = (int)(indices[1] & INDEX_MASK);
    triangles[destination++] = (int)(indices[2] & INDEX_MASK);
    triangle_count++;

    return triangle_count;
}

static int count_poly_vertices(const int *polys, uint32_t poly_start, int max_vertices_per_poly) {
    for (int i = 0; i < max_vertices_per_poly; i++) {
        if (polys[poly_start + (uint32_t)i] == MESH_NULL_IDX) return i;
    }
    return max_vertices_per_poly;
}

static bool uleft(const int *vertices, int a, int b, int c) {
    return (vertices[b * 3] - vertices[a * 3]) * (vertices[c * 3 + 2] - vertices[a * 3 + 2]) -
               (vertices[c * 3] - vertices[a * 3]) * (vertices[b * 3 + 2] - vertices[a * 3 + 2]) <
           0;
}

struct PolyMergeValue {
    int value;
    int ea;
    int eb;
};

static PolyMergeValue get_poly_merge_value(const int *polys, uint32_t poly_a, uint32_t poly_b, const int *vertices, int max_vertices_per_poly) {
    int vertex_count_a = count_poly_vertices(polys, poly_a, max_vertices_per_poly);
    int vertex_count_b = count_poly_vertices(polys, poly_b, max_vertices_per_poly);

    if (vertex_count_a + vertex_count_b - 2 > max_vertices_per_poly) return {-1, -1, -1};

    int ea = -1;
    int eb = -1;

    for (int i = 0; i < vertex_count_a; i++) {
        int va0 = polys[poly_a + (uint32_t)i];
        int va1 = polys[poly_a + (uint32_t)((i + 1) % vertex_count_a)];
        if (va0 > va1) {
            int swap = va0;
            va0 = va1;
            va1 = swap;
        }

        for (int j = 0; j < vertex_count_b; j++) {
            int vb0 = polys[poly_b + (uint32_t)j];
            int vb1 = polys[poly_b + (uint32_t)((j + 1) % vertex_count_b)];
            if (vb0 > vb1) {
                int swap = vb0;
                vb0 = vb1;
                vb1 = swap;
            }

            if (va0 == vb0 && va1 == vb1) {
                ea = i;
                eb = j;
                break;
            }
        }
    }

    if (ea == -1 || eb == -1) return {-1, -1, -1};

    int va = polys[poly_a + (uint32_t)((ea + vertex_count_a - 1) % vertex_count_a)];
    int vb = polys[poly_a + (uint32_t)ea];
    int vc = polys[poly_b + (uint32_t)((eb + 2) % vertex_count_b)];
    if (!uleft(vertices, va, vb, vc)) return {-1, -1, -1};

    va = polys[poly_b + (uint32_t)((eb + vertex_count_b - 1) % vertex_count_b)];
    vb = polys[poly_b + (uint32_t)eb];
    vc = polys[poly_a + (uint32_t)((ea + 2) % vertex_count_a)];
    if (!uleft(vertices, va, vb, vc)) return {-1, -1, -1};

    int edge_a = polys[poly_a + (uint32_t)ea];
    int edge_b = polys[poly_a + (uint32_t)((ea + 1) % vertex_count_a)];
    int dx = vertices[edge_a * 3] - vertices[edge_b * 3];
    int dz = vertices[edge_a * 3 + 2] - vertices[edge_b * 3 + 2];

    return {dx * dx + dz * dz, ea, eb};
}

static void merge_poly_vertices(int *polys, uint32_t poly_a, uint32_t poly_b, int edge_a, int edge_b, uint32_t temp_start, int max_vertices_per_poly) {
    int vertex_count_a = count_poly_vertices(polys, poly_a, max_vertices_per_poly);
    int vertex_count_b = count_poly_vertices(polys, poly_b, max_vertices_per_poly);

    for (int i = 0; i < max_vertices_per_poly; i++) polys[temp_start + (uint32_t)i] = MESH_NULL_IDX;

    uint32_t n = 0;
    for (int i = 0; i < vertex_count_a - 1; i++) polys[temp_start + n++] = polys[poly_a + (uint32_t)((edge_a + 1 + i) % vertex_count_a)];
    for (int i = 0; i < vertex_count_b - 1; i++) polys[temp_start + n++] = polys[poly_b + (uint32_t)((edge_b + 1 + i) % vertex_count_b)];

    for (int i = 0; i < max_vertices_per_poly; i++) polys[poly_a + (uint32_t)i] = polys[temp_start + (uint32_t)i];
}

struct BestMerge {
    int value;
    int pa;
    int pb;
    int ea;
    int eb;
};

static BestMerge find_best_merge(const int *polys, int poly_count, const int *vertices, int nvp) {
    BestMerge best = {0, 0, 0, 0, 0};
    for (int j = 0; j < poly_count - 1; j++) {
        for (int k = j + 1; k < poly_count; k++) {
            PolyMergeValue result = get_poly_merge_value(polys, (uint32_t)(j * nvp), (uint32_t)(k * nvp), vertices, nvp);
            if (result.value > best.value) best = {result.value, j, k, result.ea, result.eb};
        }
    }
    return best;
}

static void set_poly(Array<int> &polys, int index, int nvp, int value) {
    uint32_t end = (uint32_t)((index + 1) * nvp);
    if (polys.length < end) polys.resize(end, MESH_NULL_IDX);
    for (int j = 0; j < nvp; j++) polys[(uint32_t)(index * nvp + j)] = value;
}

static void store_poly(PolyMesh &mesh, const int *source, int region, uint8_t area) {
    int nvp = mesh.max_vertices_per_poly;
    set_poly(mesh.polys, mesh.poly_count, nvp, MESH_NULL_IDX);
    for (int k = 0; k < nvp; k++) mesh.polys[(uint32_t)(mesh.poly_count * nvp + k)] = source[k];
    if (mesh.regions.length <= (uint32_t)mesh.poly_count) mesh.regions.resize((uint32_t)mesh.poly_count + 1, 0);
    if (mesh.areas.length <= (uint32_t)mesh.poly_count) mesh.areas.resize((uint32_t)mesh.poly_count + 1, 0);
    mesh.regions[(uint32_t)mesh.poly_count] = region;
    mesh.areas[(uint32_t)mesh.poly_count] = area;
    mesh.poly_count++;
}

static bool can_remove_vertex(const PolyMesh &mesh, int removed_vertex) {
    int nvp = mesh.max_vertices_per_poly;

    int touched_vertex_count = 0;
    int remaining_edge_count = 0;

    for (int i = 0; i < mesh.poly_count; i++) {
        uint32_t poly_start = (uint32_t)(i * nvp);
        int n = count_poly_vertices(mesh.polys.data, poly_start, nvp);
        int removed_count = 0;
        int vertex_count = 0;

        for (int j = 0; j < n; j++) {
            if (mesh.polys[poly_start + (uint32_t)j] == removed_vertex) {
                touched_vertex_count++;
                removed_count++;
            }
            vertex_count++;
        }

        if (removed_count) remaining_edge_count += vertex_count - (removed_count + 1);
    }

    if (remaining_edge_count <= 2) return false;

    Array<int> edges = {};
    edges.resize((uint32_t)(touched_vertex_count * 2 * 3), 0);
    int edge_count = 0;

    for (int i = 0; i < mesh.poly_count; i++) {
        uint32_t poly_start = (uint32_t)(i * nvp);
        int n = count_poly_vertices(mesh.polys.data, poly_start, nvp);

        for (int j = 0, k = n - 1; j < n; k = j++) {
            int a = mesh.polys[poly_start + (uint32_t)j];
            int b = mesh.polys[poly_start + (uint32_t)k];
            if (a != removed_vertex && b != removed_vertex) continue;

            if (b == removed_vertex) {
                int swap = a;
                a = b;
                b = swap;
            }

            bool exists = false;
            for (int m = 0; m < edge_count; m++) {
                if (edges[(uint32_t)(m * 3 + 1)] == b) {
                    edges[(uint32_t)(m * 3 + 2)]++;
                    exists = true;
                    break;
                }
            }

            if (!exists) {
                edges[(uint32_t)(edge_count * 3)] = a;
                edges[(uint32_t)(edge_count * 3 + 1)] = b;
                edges[(uint32_t)(edge_count * 3 + 2)] = 1;
                edge_count++;
            }
        }
    }

    int open_edge_count = 0;
    for (int i = 0; i < edge_count; i++) {
        if (edges[(uint32_t)(i * 3 + 2)] < 2) open_edge_count++;
    }

    edges.release();
    return open_edge_count <= 2;
}

static void push_front(int value, int *values, int &count) {
    count++;
    for (int i = count - 1; i > 0; i--) values[i] = values[i - 1];
    values[0] = value;
}

static void push_back(int value, int *values, int &count) { values[count++] = value; }

static bool remove_vertex(BuildContext &context, PolyMesh &mesh, int removed_vertex) {
    int nvp = mesh.max_vertices_per_poly;

    int removed_vertex_count = 0;
    for (int i = 0; i < mesh.poly_count; i++) {
        uint32_t poly_start = (uint32_t)(i * nvp);
        int n = count_poly_vertices(mesh.polys.data, poly_start, nvp);
        for (int j = 0; j < n; j++) {
            if (mesh.polys[poly_start + (uint32_t)j] == removed_vertex) removed_vertex_count++;
        }
    }

    uint32_t capacity = (uint32_t)(removed_vertex_count * nvp);
    Array<int> edges = {};
    Array<int> hole = {};
    Array<int> hole_regions = {};
    Array<int> hole_areas = {};
    edges.resize(capacity * 4, 0);
    hole.resize(capacity, 0);
    hole_regions.resize(capacity, 0);
    hole_areas.resize(capacity, 0);
    int edge_count = 0;

    for (int i = 0; i < mesh.poly_count; i++) {
        uint32_t poly_start = (uint32_t)(i * nvp);
        int n = count_poly_vertices(mesh.polys.data, poly_start, nvp);
        bool has_removed = false;

        for (int j = 0; j < n; j++) {
            if (mesh.polys[poly_start + (uint32_t)j] == removed_vertex) {
                has_removed = true;
                break;
            }
        }

        if (!has_removed) continue;

        for (int j = 0, k = n - 1; j < n; k = j++) {
            if (mesh.polys[poly_start + (uint32_t)j] != removed_vertex && mesh.polys[poly_start + (uint32_t)k] != removed_vertex) {
                uint32_t e = (uint32_t)(edge_count * 4);
                edges[e] = mesh.polys[poly_start + (uint32_t)k];
                edges[e + 1] = mesh.polys[poly_start + (uint32_t)j];
                edges[e + 2] = mesh.regions[(uint32_t)i];
                edges[e + 3] = mesh.areas[(uint32_t)i];
                edge_count++;
            }
        }

        uint32_t last_poly_start = (uint32_t)((mesh.poly_count - 1) * nvp);
        if (poly_start != last_poly_start) {
            for (int j = 0; j < nvp; j++) mesh.polys[poly_start + (uint32_t)j] = mesh.polys[last_poly_start + (uint32_t)j];
        }
        for (int j = 0; j < nvp; j++) mesh.polys[last_poly_start + (uint32_t)j] = MESH_NULL_IDX;

        mesh.regions[(uint32_t)i] = mesh.regions[(uint32_t)(mesh.poly_count - 1)];
        mesh.areas[(uint32_t)i] = mesh.areas[(uint32_t)(mesh.poly_count - 1)];
        mesh.poly_count--;
        i--;
    }

    for (int i = removed_vertex; i < mesh.vertex_count - 1; i++) {
        for (int k = 0; k < 3; k++) mesh.vertices[(uint32_t)(i * 3 + k)] = mesh.vertices[(uint32_t)((i + 1) * 3 + k)];
    }
    mesh.vertex_count--;

    for (int i = 0; i < mesh.poly_count; i++) {
        uint32_t poly_start = (uint32_t)(i * nvp);
        int n = count_poly_vertices(mesh.polys.data, poly_start, nvp);
        for (int j = 0; j < n; j++) {
            if (mesh.polys[poly_start + (uint32_t)j] > removed_vertex) mesh.polys[poly_start + (uint32_t)j]--;
        }
    }

    for (int i = 0; i < edge_count; i++) {
        if (edges[(uint32_t)(i * 4)] > removed_vertex) edges[(uint32_t)(i * 4)]--;
        if (edges[(uint32_t)(i * 4 + 1)] > removed_vertex) edges[(uint32_t)(i * 4 + 1)]--;
    }

    bool result = true;
    Array<int> triangles = {};
    Array<int> triangle_vertices = {};
    Array<uint32_t> triangle_hole = {};
    Array<int> polys = {};
    Array<int> poly_regions = {};
    Array<uint8_t> poly_areas = {};

    if (edge_count > 0) {
        int hole_count = 0;
        int hole_region_count = 0;
        int hole_area_count = 0;

        push_back(edges[0], hole.data, hole_count);
        push_back(edges[2], hole_regions.data, hole_region_count);
        push_back(edges[3], hole_areas.data, hole_area_count);

        while (edge_count > 0) {
            bool match = false;

            for (int i = 0; i < edge_count; i++) {
                int ea = edges[(uint32_t)(i * 4)];
                int eb = edges[(uint32_t)(i * 4 + 1)];
                int r = edges[(uint32_t)(i * 4 + 2)];
                int a = edges[(uint32_t)(i * 4 + 3)];
                bool add = false;

                if (hole[0] == eb) {
                    push_front(ea, hole.data, hole_count);
                    push_front(r, hole_regions.data, hole_region_count);
                    push_front(a, hole_areas.data, hole_area_count);
                    add = true;
                } else if (hole[(uint32_t)(hole_count - 1)] == ea) {
                    push_back(eb, hole.data, hole_count);
                    push_back(r, hole_regions.data, hole_region_count);
                    push_back(a, hole_areas.data, hole_area_count);
                    add = true;
                }

                if (add) {
                    for (int k = 0; k < 4; k++) edges[(uint32_t)(i * 4 + k)] = edges[(uint32_t)((edge_count - 1) * 4 + k)];
                    edge_count--;
                    match = true;
                    i--;
                }
            }

            if (!match) break;
        }

        triangles.resize((uint32_t)(hole_count * 3), 0);
        triangle_vertices.resize((uint32_t)(hole_count * 4), 0);
        triangle_hole.resize((uint32_t)hole_count, 0);

        for (int i = 0; i < hole_count; i++) {
            int pi = hole[(uint32_t)i];
            triangle_vertices[(uint32_t)(i * 4)] = mesh.vertices[(uint32_t)(pi * 3)];
            triangle_vertices[(uint32_t)(i * 4 + 1)] = mesh.vertices[(uint32_t)(pi * 3 + 1)];
            triangle_vertices[(uint32_t)(i * 4 + 2)] = mesh.vertices[(uint32_t)(pi * 3 + 2)];
            triangle_vertices[(uint32_t)(i * 4 + 3)] = 0;
            triangle_hole[(uint32_t)i] = (uint32_t)i;
        }

        int triangle_count = triangulate(hole_count, triangle_vertices.data, triangle_hole.data, triangles.data);
        if (triangle_count < 0) {
            triangle_count = -triangle_count;
            build_context::warn(context, "removeVertex: triangulate() returned bad results");
        }

        polys.resize((uint32_t)((triangle_count + 1) * nvp), MESH_NULL_IDX);
        poly_regions.resize((uint32_t)triangle_count, 0);
        poly_areas.resize((uint32_t)triangle_count, 0);

        int poly_count = 0;
        for (int j = 0; j < triangle_count; j++) {
            int t0 = triangles[(uint32_t)(j * 3)];
            int t1 = triangles[(uint32_t)(j * 3 + 1)];
            int t2 = triangles[(uint32_t)(j * 3 + 2)];
            if (t0 == t1 || t0 == t2 || t1 == t2) continue;

            polys[(uint32_t)(poly_count * nvp)] = hole[(uint32_t)t0];
            polys[(uint32_t)(poly_count * nvp + 1)] = hole[(uint32_t)t1];
            polys[(uint32_t)(poly_count * nvp + 2)] = hole[(uint32_t)t2];

            if (hole_regions[(uint32_t)t0] != hole_regions[(uint32_t)t1] || hole_regions[(uint32_t)t1] != hole_regions[(uint32_t)t2]) {
                poly_regions[(uint32_t)poly_count] = MULTIPLE_REGS;
            } else {
                poly_regions[(uint32_t)poly_count] = hole_regions[(uint32_t)t0];
            }

            poly_areas[(uint32_t)poly_count] = (uint8_t)hole_areas[(uint32_t)t0];
            poly_count++;
        }

        if (poly_count > 0) {
            if (nvp > 3) {
                while (true) {
                    BestMerge best = find_best_merge(polys.data, poly_count, mesh.vertices.data, nvp);
                    if (best.value <= 0) break;

                    uint32_t pa_start = (uint32_t)(best.pa * nvp);
                    uint32_t pb_start = (uint32_t)(best.pb * nvp);
                    merge_poly_vertices(polys.data, pa_start, pb_start, best.ea, best.eb, (uint32_t)(triangle_count * nvp), nvp);

                    if (poly_regions[(uint32_t)best.pa] != poly_regions[(uint32_t)best.pb]) poly_regions[(uint32_t)best.pa] = MULTIPLE_REGS;

                    uint32_t last_start = (uint32_t)((poly_count - 1) * nvp);
                    if (pb_start != last_start) {
                        for (int m = 0; m < nvp; m++) polys[pb_start + (uint32_t)m] = polys[last_start + (uint32_t)m];
                    }
                    poly_regions[(uint32_t)best.pb] = poly_regions[(uint32_t)(poly_count - 1)];
                    poly_areas[(uint32_t)best.pb] = poly_areas[(uint32_t)(poly_count - 1)];
                    poly_count--;
                }
            }

            for (int i = 0; i < poly_count; i++) {
                store_poly(mesh, &polys[(uint32_t)(i * nvp)], poly_regions[(uint32_t)i], poly_areas[(uint32_t)i]);
            }
        }
    }

    edges.release();
    hole.release();
    hole_regions.release();
    hole_areas.release();
    triangles.release();
    triangle_vertices.release();
    triangle_hole.release();
    polys.release();
    poly_regions.release();
    poly_areas.release();
    return result;
}

PolyMesh build_poly_mesh(BuildContext &context, const ContourSet &contour_set, int max_vertices_per_poly) {
    int max_vertices_per_contour = 0;
    for (const Contour &contour : contour_set.contours) {
        if (contour.vertex_count < 3) continue;
        max_vertices_per_contour = scalar::max(max_vertices_per_contour, contour.vertex_count);
    }

    PolyMesh mesh = {};
    mesh.max_vertices_per_poly = max_vertices_per_poly;
    mesh.bounds = contour_set.bounds;
    mesh.local_width = contour_set.width;
    mesh.local_height = contour_set.height;
    mesh.cell_size = contour_set.cell_size;
    mesh.cell_height = contour_set.cell_height;
    mesh.border_size = contour_set.border_size;
    mesh.max_edge_error = contour_set.max_error;

    int nvp = max_vertices_per_poly;

    Array<int> vertex_flags = {};
    VertexMap vertex_map = {};
    Array<uint32_t> indices = {};
    Array<int> triangles = {};
    Array<int> polys = {};
    indices.resize((uint32_t)max_vertices_per_contour, 0);
    triangles.resize((uint32_t)(max_vertices_per_contour * 3), 0);
    polys.resize((uint32_t)((max_vertices_per_contour + 1) * nvp), MESH_NULL_IDX);
    uint32_t temp_poly_start = (uint32_t)(max_vertices_per_contour * nvp);

    for (uint32_t i = 0; i < contour_set.contours.length; i++) {
        const Contour &contour = contour_set.contours[i];
        if (contour.vertex_count < 3) continue;

        for (int j = 0; j < contour.vertex_count; j++) indices[(uint32_t)j] = (uint32_t)j;

        int triangle_count = triangulate(contour.vertex_count, contour.vertices.data, indices.data, triangles.data);
        if (triangle_count <= 0) {
            build_context::warn(context, "Bad triangulation for contour %u", i);
            triangle_count = scalar::abs(triangle_count);
        }

        for (int j = 0; j < contour.vertex_count; j++) {
            uint32_t v = (uint32_t)(j * 4);
            int index = add_vertex(contour.vertices[v], contour.vertices[v + 1], contour.vertices[v + 2], mesh.vertices, vertex_flags, vertex_map);
            indices[(uint32_t)j] = (uint32_t)index;
            if (contour.vertices[v + 3] & BORDER_VERTEX) vertex_flags[(uint32_t)index] = 1;
        }

        int poly_count = 0;
        for (uint32_t j = 0; j < temp_poly_start; j++) polys[j] = MESH_NULL_IDX;

        for (int j = 0; j < triangle_count; j++) {
            int t0 = triangles[(uint32_t)(j * 3)];
            int t1 = triangles[(uint32_t)(j * 3 + 1)];
            int t2 = triangles[(uint32_t)(j * 3 + 2)];
            if (t0 == t1 || t0 == t2 || t1 == t2) continue;
            polys[(uint32_t)(poly_count * nvp)] = (int)indices[(uint32_t)t0];
            polys[(uint32_t)(poly_count * nvp + 1)] = (int)indices[(uint32_t)t1];
            polys[(uint32_t)(poly_count * nvp + 2)] = (int)indices[(uint32_t)t2];
            poly_count++;
        }

        if (poly_count == 0) continue;

        if (nvp > 3) {
            while (true) {
                BestMerge best = find_best_merge(polys.data, poly_count, mesh.vertices.data, nvp);
                if (best.value <= 0) break;

                uint32_t pa_start = (uint32_t)(best.pa * nvp);
                uint32_t pb_start = (uint32_t)(best.pb * nvp);
                merge_poly_vertices(polys.data, pa_start, pb_start, best.ea, best.eb, temp_poly_start, nvp);

                for (int m = 0; m < nvp; m++) polys[pb_start + (uint32_t)m] = polys[(uint32_t)((poly_count - 1) * nvp + m)];
                poly_count--;
            }
        }

        for (int j = 0; j < poly_count; j++) store_poly(mesh, &polys[(uint32_t)(j * nvp)], contour.reg, contour.area);
    }

    mesh.vertex_count = (int)mesh.vertices.length / 3;

    for (int i = 0; i < mesh.vertex_count; i++) {
        if (!vertex_flags[(uint32_t)i]) continue;
        if (!can_remove_vertex(mesh, i)) continue;

        if (remove_vertex(context, mesh, i)) {
            for (int j = i; j < mesh.vertex_count; j++) vertex_flags[(uint32_t)j] = vertex_flags[(uint32_t)(j + 1)];
            i--;
        } else {
            build_context::error(context, "Failed to remove edge vertex %d", i);
        }
    }

    mesh.polys.resize((uint32_t)(mesh.poly_count * nvp));
    mesh.regions.resize((uint32_t)mesh.poly_count);
    mesh.areas.resize((uint32_t)mesh.poly_count);
    mesh.vertices.resize((uint32_t)(mesh.vertex_count * 3));
    mesh.flags.resize((uint32_t)mesh.poly_count, 0);

    vertex_flags.release();
    vertex_map.buckets.release();
    vertex_map.entries.release();
    indices.release();
    triangles.release();
    polys.release();
    return mesh;
}

void release(PolyMesh &poly_mesh) {
    poly_mesh.vertices.release();
    poly_mesh.polys.release();
    poly_mesh.regions.release();
    poly_mesh.flags.release();
    poly_mesh.areas.release();
}

}

}
