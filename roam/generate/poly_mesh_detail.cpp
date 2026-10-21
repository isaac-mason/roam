#include "roam/generate/poly_mesh_detail.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "roam/generate/common.h"
#include "roam/geometry/geometry.h"
#include "roam/math/scalar.h"

namespace roam {

namespace poly_mesh_detail {

static const int UNSET_HEIGHT = 0xffff;
static const int MAX_VERTS = 127;
static const int MAX_TRIS = 255;
static const int MAX_VERTS_PER_EDGE = 32;
static const int RETRACT_SIZE = 256;

static const int EV_UNDEF = -1;
static const int EV_HULL = -2;

using compact_heightfield::get_con;

struct HeightPatch {
    Array<int> data;
    int xmin;
    int ymin;
    int width;
    int height;
};

static Vec2 xz(const float *values, int index) { return {values[index], values[index + 2]}; }

static float get_jitter_x(int i) { return (float)(((uint32_t)i * 0x8da6b343u) & 0xffff) / 65535.0f * 2.0f - 1.0f; }

static float get_jitter_y(int i) { return (float)(((uint32_t)i * 0xd8163841u) & 0xffff) / 65535.0f * 2.0f - 1.0f; }

static int previous(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
static int next(int i, int n) { return i + 1 < n ? i + 1 : 0; }

static int floor_div(int value, int divisor) { return (int)floor((double)value / (double)divisor); }

static void fill(Array<int> &values, int value) {
    for (int &entry : values) entry = value;
}

static int get_height(float fx, float fy, float fz, float ics, float ch, int radius, const HeightPatch &hp) {
    int ix = (int)floorf(fx * ics + 0.01f);
    int iz = (int)floorf(fz * ics + 0.01f);
    ix = scalar::clamp(ix - hp.xmin, 0, hp.width - 1);
    iz = scalar::clamp(iz - hp.ymin, 0, hp.height - 1);
    int h = hp.data[(uint32_t)(ix + iz * hp.width)];

    if (h != UNSET_HEIGHT) return h;

    int x = 1;
    int z = 0;
    int dx = 1;
    int dz = 0;
    int max_size = radius * 2 + 1;
    int max_iterations = max_size * max_size - 1;

    int next_ring_iteration_start = 8;
    int next_ring_iterations = 16;

    float dmin = FLT_MAX;
    for (int i = 0; i < max_iterations; i++) {
        int nx = ix + x;
        int nz = iz + z;

        if (nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height) {
            int nh = hp.data[(uint32_t)(nx + nz * hp.width)];
            if (nh != UNSET_HEIGHT) {
                float d = fabsf((float)nh * ch - fy);
                if (d < dmin) {
                    h = nh;
                    dmin = d;
                }
            }
        }

        if (i + 1 == next_ring_iteration_start) {
            if (h != UNSET_HEIGHT) break;
            next_ring_iteration_start += next_ring_iterations;
            next_ring_iterations += 8;
        }

        if (x == z || (x < 0 && x == -z) || (x > 0 && x == 1 - z)) {
            int swap = dx;
            dx = -dz;
            dz = swap;
        }
        x += dx;
        z += dz;
    }
    return h;
}

static int find_edge(const Array<int> &edges, int edge_count, int s, int t) {
    for (int i = 0; i < edge_count; i++) {
        uint32_t e = (uint32_t)(i * 4);
        if ((edges[e] == s && edges[e + 1] == t) || (edges[e] == t && edges[e + 1] == s)) return i;
    }
    return EV_UNDEF;
}

static int add_edge(BuildContext &context, Array<int> &edges, int &edge_count, int max_edges, int s, int t, int l, int r) {
    if (edge_count >= max_edges) {
        build_context::error(context, "addEdge: Too many edges (%d/%d).", edge_count, max_edges);
        return EV_UNDEF;
    }

    if (find_edge(edges, edge_count, s, t) != EV_UNDEF) return EV_UNDEF;

    uint32_t e = (uint32_t)(edge_count * 4);
    edges[e] = s;
    edges[e + 1] = t;
    edges[e + 2] = l;
    edges[e + 3] = r;
    return edge_count++;
}

static void update_left_face(Array<int> &edges, int edge_index, int s, int t, int f) {
    uint32_t e = (uint32_t)(edge_index * 4);
    if (edges[e] == s && edges[e + 1] == t && edges[e + 2] == EV_UNDEF) {
        edges[e + 2] = f;
    } else if (edges[e + 1] == s && edges[e] == t && edges[e + 3] == EV_UNDEF) {
        edges[e + 3] = f;
    }
}

static bool overlap_edges(const float *points, const Array<int> &edges, int edge_count, int s1, int t1) {
    for (int i = 0; i < edge_count; ++i) {
        int s0 = edges[(uint32_t)(i * 4)];
        int t0 = edges[(uint32_t)(i * 4 + 1)];
        if (s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1) continue;
        if (geometry::overlap_seg_seg_2d(xz(points, s0 * 3), xz(points, t0 * 3), xz(points, s1 * 3), xz(points, t1 * 3))) return true;
    }
    return false;
}

struct Circle {
    Vec2 center;
    float radius;
};

static Circle circumcircle(Vec2 a, Vec2 b, Vec2 c) {
    float bx = b.x - a.x;
    float by = b.y - a.y;
    float cx = c.x - a.x;
    float cy = c.y - a.y;
    float cp = bx * cy - by * cx;
    if (fabsf(cp) > scalar::EPSILON) {
        float b_sqr = bx * bx + by * by;
        float c_sqr = cx * cx + cy * cy;
        float d = 0.5f / cp;
        float ux = (cy * b_sqr - by * c_sqr) * d;
        float uy = (bx * c_sqr - cx * b_sqr) * d;
        return {{ux + a.x, uy + a.y}, sqrtf(ux * ux + uy * uy)};
    }
    return {a, 0};
}

static void complete_facet(BuildContext &context, const float *points, int point_count, Array<int> &edges, int &edge_count, int max_edges,
                           int &face_count, int e) {
    const float EPS_FACET = 1e-5f;

    uint32_t edge_index = (uint32_t)(e * 4);

    int s;
    int t;
    if (edges[edge_index + 2] == EV_UNDEF) {
        s = edges[edge_index];
        t = edges[edge_index + 1];
    } else if (edges[edge_index + 3] == EV_UNDEF) {
        s = edges[edge_index + 1];
        t = edges[edge_index];
    } else {
        return;
    }

    int pt = point_count;
    Vec2 center = {0, 0};
    float r = -1;

    for (int u = 0; u < point_count; ++u) {
        if (u == s || u == t) continue;

        Vec2 point_s = xz(points, s * 3);
        Vec2 to_t = vec2::subtract(xz(points, t * 3), point_s);
        Vec2 to_u = vec2::subtract(xz(points, u * 3), point_s);
        float cross_product = to_t.x * to_u.y - to_t.y * to_u.x;

        if (cross_product <= EPS_FACET) continue;

        if (r < 0) {
            pt = u;
            Circle circle = circumcircle(xz(points, s * 3), xz(points, t * 3), xz(points, u * 3));
            center = circle.center;
            r = circle.radius;
            continue;
        }

        float d = vec2::distance(center, xz(points, u * 3));
        const float tolerance = 0.001f;

        if (d > r * (1 + tolerance)) continue;

        if (d >= r * (1 - tolerance)) {
            if (overlap_edges(points, edges, edge_count, s, u)) continue;
            if (overlap_edges(points, edges, edge_count, t, u)) continue;
        }

        pt = u;
        Circle circle = circumcircle(xz(points, s * 3), xz(points, t * 3), xz(points, u * 3));
        center = circle.center;
        r = circle.radius;
    }

    if (pt < point_count) {
        update_left_face(edges, e, s, t, face_count);

        int new_edge = find_edge(edges, edge_count, pt, s);
        if (new_edge == EV_UNDEF) {
            add_edge(context, edges, edge_count, max_edges, pt, s, face_count, EV_UNDEF);
        } else {
            update_left_face(edges, new_edge, pt, s, face_count);
        }

        new_edge = find_edge(edges, edge_count, t, pt);
        if (new_edge == EV_UNDEF) {
            add_edge(context, edges, edge_count, max_edges, t, pt, face_count, EV_UNDEF);
        } else {
            update_left_face(edges, new_edge, t, pt, face_count);
        }

        face_count++;
    } else {
        update_left_face(edges, e, s, t, EV_HULL);
    }
}

static void delaunay_hull(BuildContext &context, int point_count, const float *points, int hull_count, const int *hull, Array<int> &triangles,
                          Array<int> &edges) {
    int face_count = 0;
    int edge_count = 0;
    int max_edges = point_count * 10;

    edges.resize((uint32_t)(max_edges * 4), 0);

    for (int i = 0, j = hull_count - 1; i < hull_count; j = i++) add_edge(context, edges, edge_count, max_edges, hull[j], hull[i], EV_HULL, EV_UNDEF);

    for (int current_edge = 0; current_edge < edge_count; current_edge++) {
        if (edges[(uint32_t)(current_edge * 4 + 2)] == EV_UNDEF) {
            complete_facet(context, points, point_count, edges, edge_count, max_edges, face_count, current_edge);
        }
        if (edges[(uint32_t)(current_edge * 4 + 3)] == EV_UNDEF) {
            complete_facet(context, points, point_count, edges, edge_count, max_edges, face_count, current_edge);
        }
    }

    triangles.resize((uint32_t)(face_count * 4), -1);
    fill(triangles, -1);

    for (int i = 0; i < edge_count; ++i) {
        uint32_t e = (uint32_t)(i * 4);
        if (edges[e + 3] >= 0) {
            uint32_t t = (uint32_t)(edges[e + 3] * 4);
            if (triangles[t] == -1) {
                triangles[t] = edges[e];
                triangles[t + 1] = edges[e + 1];
            } else if (triangles[t] == edges[e + 1]) {
                triangles[t + 2] = edges[e];
            } else if (triangles[t + 1] == edges[e]) {
                triangles[t + 2] = edges[e + 1];
            }
        }
        if (edges[e + 2] >= 0) {
            uint32_t t = (uint32_t)(edges[e + 2] * 4);
            if (triangles[t] == -1) {
                triangles[t] = edges[e + 1];
                triangles[t + 1] = edges[e];
            } else if (triangles[t] == edges[e]) {
                triangles[t + 2] = edges[e + 1];
            } else if (triangles[t + 1] == edges[e + 1]) {
                triangles[t + 2] = edges[e];
            }
        }
    }

    for (int i = 0; i < (int)triangles.length / 4; ++i) {
        uint32_t t = (uint32_t)(i * 4);
        if (triangles[t] != -1 && triangles[t + 1] != -1 && triangles[t + 2] != -1) continue;

        build_context::warn(context, "delaunayHull: Removing dangling face %d [%d,%d,%d].", i, triangles[t], triangles[t + 1], triangles[t + 2]);
        uint32_t last = triangles.length - 4;
        for (uint32_t k = 0; k < 4; k++) triangles[t + k] = triangles[last + k];
        triangles.resize(last);
        --i;
    }
}

static void push_triangle(Array<int> &triangles, int a, int b, int c) {
    triangles.push(a);
    triangles.push(b);
    triangles.push(c);
    triangles.push(0);
}

static void triangulate_hull(const float *vertices, int hull_count, const int *hull, int input_count, Array<int> &triangles) {
    int start = 0;
    int left = 1;
    int right = hull_count - 1;

    float dmin = FLT_MAX;
    for (int i = 0; i < hull_count; i++) {
        if (hull[i] >= input_count) continue;
        int pi = previous(i, hull_count);
        int ni = next(i, hull_count);
        Vec2 previous_vertex = xz(vertices, hull[pi] * 3);
        Vec2 current_vertex = xz(vertices, hull[i] * 3);
        Vec2 next_vertex = xz(vertices, hull[ni] * 3);

        float d = vec2::distance(previous_vertex, current_vertex) + vec2::distance(current_vertex, next_vertex) +
                  vec2::distance(next_vertex, previous_vertex);
        if (d < dmin) {
            start = i;
            left = ni;
            right = pi;
            dmin = d;
        }
    }

    push_triangle(triangles, hull[start], hull[left], hull[right]);

    while (next(left, hull_count) != right) {
        int next_left = next(left, hull_count);
        int next_right = previous(right, hull_count);

        Vec2 current_left_vertex = xz(vertices, hull[left] * 3);
        Vec2 next_left_vertex = xz(vertices, hull[next_left] * 3);
        Vec2 current_right_vertex = xz(vertices, hull[right] * 3);
        Vec2 next_right_vertex = xz(vertices, hull[next_right] * 3);

        float d_left = vec2::distance(current_left_vertex, next_left_vertex) + vec2::distance(next_left_vertex, current_right_vertex);
        float d_right = vec2::distance(current_right_vertex, next_right_vertex) + vec2::distance(current_left_vertex, next_right_vertex);

        if (d_left < d_right) {
            push_triangle(triangles, hull[left], hull[next_left], hull[right]);
            left = next_left;
        } else {
            push_triangle(triangles, hull[left], hull[next_right], hull[right]);
            right = next_right;
        }
    }
}

static bool on_hull(int a, int b, int hull_count, const int *hull) {
    if (a >= hull_count || b >= hull_count) return false;
    for (int j = hull_count - 1, i = 0; i < hull_count; j = i++) {
        if (a == hull[j] && b == hull[i]) return true;
    }
    return false;
}

static void set_tri_flags(Array<int> &triangles, int hull_count, const int *hull) {
    for (uint32_t i = 0; i < triangles.length; i += 4) {
        int a = triangles[i];
        int b = triangles[i + 1];
        int c = triangles[i + 2];
        int flags = 0;
        flags |= (on_hull(a, b, hull_count, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 0;
        flags |= (on_hull(b, c, hull_count, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 2;
        flags |= (on_hull(c, a, hull_count, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 4;
        triangles[i + 3] = flags;
    }
}

static const int SEED_OFFSETS[9][2] = {{0, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}};

static void push_seed(Array<int> &seeds, int x, int y, int index) {
    seeds.push(x);
    seeds.push(y);
    seeds.push(index);
}

static void seed_array_with_poly_center(BuildContext &context, const CompactHeightfield &chf, const int *poly, int poly_vertex_count,
                                        const int *vertices, int border_size, HeightPatch &hp, Array<int> &seeds) {
    int start_cell_x = 0;
    int start_cell_y = 0;
    int start_span_index = -1;
    int dmin = UNSET_HEIGHT;

    for (int j = 0; j < poly_vertex_count && dmin > 0; ++j) {
        for (int k = 0; k < 9 && dmin > 0; ++k) {
            int ax = vertices[poly[j] * 3] + SEED_OFFSETS[k][0];
            int ay = vertices[poly[j] * 3 + 1];
            int az = vertices[poly[j] * 3 + 2] + SEED_OFFSETS[k][1];
            if (ax < hp.xmin || ax >= hp.xmin + hp.width || az < hp.ymin || az >= hp.ymin + hp.height) continue;

            CompactHeightfieldCell c = chf.cells[(uint32_t)(ax + border_size + (az + border_size) * chf.width)];
            for (uint32_t i = c.index, ni = c.index + c.count; i < ni && dmin > 0; ++i) {
                int d = scalar::abs(ay - chf.spans[i].y);
                if (d < dmin) {
                    start_cell_x = ax;
                    start_cell_y = az;
                    start_span_index = (int)i;
                    dmin = d;
                }
            }
        }
    }

    int pcx = 0;
    int pcy = 0;
    for (int j = 0; j < poly_vertex_count; ++j) {
        pcx += vertices[poly[j] * 3];
        pcy += vertices[poly[j] * 3 + 2];
    }
    pcx = floor_div(pcx, poly_vertex_count);
    pcy = floor_div(pcy, poly_vertex_count);

    seeds.clear();
    push_seed(seeds, start_cell_x, start_cell_y, start_span_index);

    int directions[4] = {0, 1, 2, 3};
    fill(hp.data, 0);

    int cx = -1;
    int cy = -1;
    int ci = -1;
    while (true) {
        if (seeds.length < 3) {
            build_context::warn(context, "Walk towards polygon center failed to reach center");
            break;
        }

        ci = seeds.pop();
        cy = seeds.pop();
        cx = seeds.pop();

        if (cx == pcx && cy == pcy) break;

        int direct_direction = cx == pcx ? common::get_dir_for_offset(0, pcy > cy ? 1 : -1) : common::get_dir_for_offset(pcx > cx ? 1 : -1, 0);

        int swap = directions[direct_direction];
        directions[direct_direction] = directions[3];
        directions[3] = swap;

        const CompactHeightfieldSpan &cs = chf.spans[(uint32_t)ci];
        for (int i = 0; i < 4; i++) {
            int direction = directions[i];
            if (get_con(cs, direction) == NOT_CONNECTED) continue;

            int new_x = cx + common::get_dir_offset_x(direction);
            int new_y = cy + common::get_dir_offset_y(direction);

            int hpx = new_x - hp.xmin;
            int hpy = new_y - hp.ymin;
            if (hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height) continue;

            if (hp.data[(uint32_t)(hpx + hpy * hp.width)] != 0) continue;

            hp.data[(uint32_t)(hpx + hpy * hp.width)] = 1;
            push_seed(seeds, new_x, new_y,
                      (int)chf.cells[(uint32_t)(new_x + border_size + (new_y + border_size) * chf.width)].index + get_con(cs, direction));
        }

        swap = directions[direct_direction];
        directions[direct_direction] = directions[3];
        directions[3] = swap;
    }

    seeds.clear();
    push_seed(seeds, cx + border_size, cy + border_size, ci);

    fill(hp.data, UNSET_HEIGHT);
    hp.data[(uint32_t)(cx - hp.xmin + (cy - hp.ymin) * hp.width)] = chf.spans[(uint32_t)ci].y;
}

static void get_height_data(BuildContext &context, const CompactHeightfield &chf, const int *poly, int poly_vertex_count, const int *vertices,
                            int border_size, HeightPatch &hp, Array<int> &queue, int region) {
    queue.clear();
    fill(hp.data, UNSET_HEIGHT);

    bool empty = true;

    if (region != common::MULTIPLE_REGS) {
        for (int hy = 0; hy < hp.height; hy++) {
            int y = hp.ymin + hy + border_size;
            for (int hx = 0; hx < hp.width; hx++) {
                int x = hp.xmin + hx + border_size;
                CompactHeightfieldCell c = chf.cells[(uint32_t)(x + y * chf.width)];
                for (uint32_t i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    const CompactHeightfieldSpan &s = chf.spans[i];
                    if (s.region != region) continue;

                    hp.data[(uint32_t)(hx + hy * hp.width)] = s.y;
                    empty = false;

                    bool border = false;
                    for (int direction = 0; direction < 4; ++direction) {
                        if (get_con(s, direction) == NOT_CONNECTED) continue;
                        int ax = x + common::get_dir_offset_x(direction);
                        int ay = y + common::get_dir_offset_y(direction);
                        uint32_t ai = chf.cells[(uint32_t)(ax + ay * chf.width)].index + (uint32_t)get_con(s, direction);
                        if (chf.spans[ai].region != region) {
                            border = true;
                            break;
                        }
                    }
                    if (border) push_seed(queue, x, y, (int)i);
                    break;
                }
            }
        }
    }

    if (empty) seed_array_with_poly_center(context, chf, poly, poly_vertex_count, vertices, border_size, hp, queue);

    uint32_t head = 0;

    while (head * 3 < queue.length) {
        int cx = queue[head * 3];
        int cy = queue[head * 3 + 1];
        int ci = queue[head * 3 + 2];
        head++;

        if (head >= (uint32_t)RETRACT_SIZE) {
            head = 0;
            uint32_t removed = (uint32_t)(RETRACT_SIZE * 3);
            uint32_t remaining = queue.length > removed ? queue.length - removed : 0;
            if (remaining > 0) memmove(queue.data, queue.data + removed, remaining * sizeof(int));
            queue.resize(remaining);
        }

        const CompactHeightfieldSpan &cs = chf.spans[(uint32_t)ci];
        for (int direction = 0; direction < 4; ++direction) {
            if (get_con(cs, direction) == NOT_CONNECTED) continue;

            int ax = cx + common::get_dir_offset_x(direction);
            int ay = cy + common::get_dir_offset_y(direction);
            int hx = ax - hp.xmin - border_size;
            int hy = ay - hp.ymin - border_size;

            if (hx < 0 || hx >= hp.width || hy < 0 || hy >= hp.height) continue;
            if (hp.data[(uint32_t)(hx + hy * hp.width)] != UNSET_HEIGHT) continue;

            uint32_t ai = chf.cells[(uint32_t)(ax + ay * chf.width)].index + (uint32_t)get_con(cs, direction);
            hp.data[(uint32_t)(hx + hy * hp.width)] = chf.spans[ai].y;

            push_seed(queue, ax, ay, (int)ai);
        }
    }
}

static void store_vertex(Array<float> &vertices, int index, Vec3 vertex) {
    uint32_t end = (uint32_t)((index + 1) * 3);
    if (vertices.length < end) vertices.resize(end, 0);
    vec3::store(vertices.data, (uint32_t)(index * 3), vertex);
}

static bool build_poly_detail(BuildContext &context, const float *in_vertices, int input_count, float sample_distance, float sample_max_error,
                              int height_search_radius, const CompactHeightfield &chf, const HeightPatch &hp, Array<float> &vertices,
                              Array<int> &triangles, Array<int> &edges, Array<int> &samples) {
    float edge[(MAX_VERTS_PER_EDGE + 1) * 3];
    int hull[MAX_VERTS];
    int hull_count = 0;

    int vertex_count = input_count;

    vertices.clear();
    for (int i = 0; i < input_count; ++i) store_vertex(vertices, i, vec3::load(in_vertices, (uint32_t)(i * 3)));

    edges.clear();
    triangles.clear();

    float cs = chf.cell_size;
    float ics = 1.0f / cs;

    float min_extent = geometry::poly_min_extent(vertices.data, vertex_count);

    if (sample_distance > 0) {
        for (int i = 0, j = input_count - 1; i < input_count; j = i++) {
            int vj_start = j * 3;
            int vi_start = i * 3;
            bool swapped = false;

            if (fabsf(in_vertices[vj_start] - in_vertices[vi_start]) < 1e-6f) {
                if (in_vertices[vj_start + 2] > in_vertices[vi_start + 2]) {
                    int swap = vi_start;
                    vi_start = vj_start;
                    vj_start = swap;
                    swapped = true;
                }
            } else if (in_vertices[vj_start] > in_vertices[vi_start]) {
                int swap = vi_start;
                vi_start = vj_start;
                vj_start = swap;
                swapped = true;
            }

            Vec3 vj = vec3::load(in_vertices, (uint32_t)vj_start);
            Vec3 vi = vec3::load(in_vertices, (uint32_t)vi_start);

            float dx = vi.x - vj.x;
            float dy = vi.y - vj.y;
            float dz = vi.z - vj.z;
            float d = sqrtf(dx * dx + dz * dz);
            int nn = 1 + (int)floorf(d / sample_distance);
            if (nn >= MAX_VERTS_PER_EDGE) nn = MAX_VERTS_PER_EDGE - 1;
            if (vertex_count + nn >= MAX_VERTS) nn = MAX_VERTS - 1 - vertex_count;

            for (int k = 0; k <= nn; ++k) {
                float u = (float)k / (float)nn;
                int pos = k * 3;
                edge[pos] = vj.x + dx * u;
                edge[pos + 1] = vj.y + dy * u;
                edge[pos + 2] = vj.z + dz * u;
                edge[pos + 1] = (float)get_height(edge[pos], edge[pos + 1], edge[pos + 2], ics, chf.cell_height, height_search_radius, hp) *
                                chf.cell_height;
            }

            int idx[MAX_VERTS_PER_EDGE] = {};
            idx[0] = 0;
            idx[1] = nn;
            int idx_count = 2;

            for (int k = 0; k < idx_count - 1;) {
                int a = idx[k];
                int b = idx[k + 1];
                Vec3 va = vec3::load(edge, (uint32_t)(a * 3));
                Vec3 vb = vec3::load(edge, (uint32_t)(b * 3));

                float maxd = 0;
                int maxi = -1;
                for (int m = a + 1; m < b; ++m) {
                    float deviation = geometry::distance_pt_seg(vec3::load(edge, (uint32_t)(m * 3)), va, vb);
                    if (deviation > maxd) {
                        maxd = deviation;
                        maxi = m;
                    }
                }

                if (maxi != -1 && maxd > sample_max_error * sample_max_error) {
                    for (int m = idx_count; m > k; --m) idx[m] = idx[m - 1];
                    idx[k + 1] = maxi;
                    idx_count++;
                } else {
                    ++k;
                }
            }

            hull[hull_count++] = j;
            if (swapped) {
                for (int k = idx_count - 2; k > 0; --k) {
                    store_vertex(vertices, vertex_count, vec3::load(edge, (uint32_t)(idx[k] * 3)));
                    hull[hull_count++] = vertex_count;
                    vertex_count++;
                }
            } else {
                for (int k = 1; k < idx_count - 1; ++k) {
                    store_vertex(vertices, vertex_count, vec3::load(edge, (uint32_t)(idx[k] * 3)));
                    hull[hull_count++] = vertex_count;
                    vertex_count++;
                }
            }
        }
    }

    if (min_extent < sample_distance * 2) {
        triangulate_hull(vertices.data, hull_count, hull, input_count, triangles);
        set_tri_flags(triangles, hull_count, hull);
        return true;
    }

    triangulate_hull(vertices.data, hull_count, hull, input_count, triangles);

    if (triangles.length == 0) {
        build_context::warn(context, "buildPolyDetail: Could not triangulate polygon (%d verts).", vertex_count);
        return true;
    }

    if (sample_distance > 0) {
        Vec3 bmin = vec3::load(in_vertices, 0);
        Vec3 bmax = bmin;
        for (int i = 1; i < input_count; ++i) {
            Vec3 vertex = vec3::load(in_vertices, (uint32_t)(i * 3));
            bmin = vec3::min(bmin, vertex);
            bmax = vec3::max(bmax, vertex);
        }
        int x0 = (int)floorf(bmin.x / sample_distance);
        int x1 = (int)ceilf(bmax.x / sample_distance);
        int z0 = (int)floorf(bmin.z / sample_distance);
        int z1 = (int)ceilf(bmax.z / sample_distance);
        samples.clear();
        for (int z = z0; z < z1; ++z) {
            for (int x = x0; x < x1; ++x) {
                Vec3 point = {(float)x * sample_distance, (bmax.y + bmin.y) * 0.5f, (float)z * sample_distance};
                if (geometry::dist_to_poly(input_count, in_vertices, point) > -sample_distance / 2) continue;
                samples.push(x);
                samples.push(get_height(point.x, point.y, point.z, ics, chf.cell_height, height_search_radius, hp));
                samples.push(z);
                samples.push(0);
            }
        }

        int sample_count = (int)samples.length / 4;
        for (int iteration = 0; iteration < sample_count; ++iteration) {
            if (vertex_count >= MAX_VERTS) break;

            Vec3 best_point = {0, 0, 0};
            float best_d = 0;
            int best_i = -1;
            for (int i = 0; i < sample_count; ++i) {
                uint32_t s = (uint32_t)(i * 4);
                if (samples[s + 3]) continue;
                Vec3 point = {(float)samples[s] * sample_distance + get_jitter_x(i) * cs * 0.1f, (float)samples[s + 1] * chf.cell_height,
                              (float)samples[s + 2] * sample_distance + get_jitter_y(i) * cs * 0.1f};
                float d = geometry::dist_to_tri_mesh(point, vertices.data, triangles.data, (int)triangles.length / 4);
                if (d < 0) continue;
                if (d > best_d) {
                    best_d = d;
                    best_i = i;
                    best_point = point;
                }
            }

            if (best_d <= sample_max_error || best_i == -1) break;

            samples[(uint32_t)(best_i * 4 + 3)] = 1;
            store_vertex(vertices, vertex_count, best_point);
            vertex_count++;

            edges.clear();
            triangles.clear();
            delaunay_hull(context, vertex_count, vertices.data, hull_count, hull, triangles, edges);
        }
    }

    int triangle_count = (int)triangles.length / 4;
    if (triangle_count > MAX_TRIS) {
        triangles.resize((uint32_t)(MAX_TRIS * 4));
        build_context::warn(context, "buildPolyMeshDetail: Shrinking triangle count from %d to max %d.", triangle_count, MAX_TRIS);
    }

    set_tri_flags(triangles, hull_count, hull);

    return true;
}

PolyMeshDetail build_poly_mesh_detail(BuildContext &context, const PolyMesh &poly_mesh, const CompactHeightfield &compact_heightfield,
                                      float sample_distance, float sample_max_error) {
    PolyMeshDetail detail = {};
    if (poly_mesh.vertex_count == 0 || poly_mesh.poly_count == 0) return detail;

    int nvp = poly_mesh.max_vertices_per_poly;
    float cs = poly_mesh.cell_size;
    float ch = poly_mesh.cell_height;
    Vec3 origin = poly_mesh.bounds.min;
    int border_size = poly_mesh.border_size;
    int height_search_radius = scalar::max(1, (int)ceilf(poly_mesh.max_edge_error));

    HeightPatch hp = {};

    int max_width = 0;
    int max_height = 0;

    Array<int> bounds = {};
    bounds.resize((uint32_t)(poly_mesh.poly_count * 4), 0);

    for (int i = 0; i < poly_mesh.poly_count; ++i) {
        uint32_t p = (uint32_t)(i * nvp);
        int xmin = compact_heightfield.width;
        int xmax = 0;
        int ymin = compact_heightfield.height;
        int ymax = 0;
        for (int j = 0; j < nvp; ++j) {
            if (poly_mesh.polys[p + (uint32_t)j] == MESH_NULL_IDX) break;
            uint32_t v = (uint32_t)(poly_mesh.polys[p + (uint32_t)j] * 3);
            xmin = scalar::min(xmin, poly_mesh.vertices[v]);
            xmax = scalar::max(xmax, poly_mesh.vertices[v]);
            ymin = scalar::min(ymin, poly_mesh.vertices[v + 2]);
            ymax = scalar::max(ymax, poly_mesh.vertices[v + 2]);
        }
        uint32_t b = (uint32_t)(i * 4);
        bounds[b] = scalar::max(0, xmin - 1);
        bounds[b + 1] = scalar::min(compact_heightfield.width, xmax + 1);
        bounds[b + 2] = scalar::max(0, ymin - 1);
        bounds[b + 3] = scalar::min(compact_heightfield.height, ymax + 1);
        if (bounds[b] >= bounds[b + 1] || bounds[b + 2] >= bounds[b + 3]) continue;
        max_width = scalar::max(max_width, bounds[b + 1] - bounds[b]);
        max_height = scalar::max(max_height, bounds[b + 3] - bounds[b + 2]);
    }

    hp.data.resize((uint32_t)(max_width * max_height), 0);

    detail.meshes.resize((uint32_t)(poly_mesh.poly_count * 4), 0);
    detail.mesh_count = poly_mesh.poly_count;

    Array<int> edges = {};
    Array<int> triangles = {};
    Array<int> queue = {};
    Array<int> samples = {};
    Array<float> vertices = {};
    Array<float> poly = {};
    poly.resize((uint32_t)(nvp * 3), 0);

    for (int i = 0; i < poly_mesh.poly_count; ++i) {
        uint32_t p = (uint32_t)(i * nvp);

        int poly_vertex_count = 0;
        for (int j = 0; j < nvp; ++j) {
            if (poly_mesh.polys[p + (uint32_t)j] == MESH_NULL_IDX) break;
            uint32_t v = (uint32_t)(poly_mesh.polys[p + (uint32_t)j] * 3);
            poly[(uint32_t)(j * 3)] = (float)poly_mesh.vertices[v] * cs;
            poly[(uint32_t)(j * 3 + 1)] = (float)poly_mesh.vertices[v + 1] * ch;
            poly[(uint32_t)(j * 3 + 2)] = (float)poly_mesh.vertices[v + 2] * cs;
            poly_vertex_count++;
        }

        uint32_t b = (uint32_t)(i * 4);
        hp.xmin = bounds[b];
        hp.ymin = bounds[b + 2];
        hp.width = bounds[b + 1] - bounds[b];
        hp.height = bounds[b + 3] - bounds[b + 2];
        get_height_data(context, compact_heightfield, &poly_mesh.polys[p], poly_vertex_count, poly_mesh.vertices.data, border_size, hp, queue,
                        poly_mesh.regions[(uint32_t)i]);

        if (!build_poly_detail(context, poly.data, poly_vertex_count, sample_distance, sample_max_error, height_search_radius, compact_heightfield, hp,
                               vertices, triangles, edges, samples)) {
            build_context::error(context, "buildPolyMeshDetail: Failed to build detail mesh for poly %d.", i);
            continue;
        }

        for (uint32_t v = 0; v < vertices.length; v += 3) {
            vertices[v] += origin.x;
            vertices[v + 1] += origin.y + compact_heightfield.cell_height;
            vertices[v + 2] += origin.z;
        }

        int triangle_count = (int)triangles.length / 4;

        detail.meshes[b] = detail.vertex_count;
        detail.meshes[b + 1] = (int)vertices.length / 3;
        detail.meshes[b + 2] = detail.triangle_count;
        detail.meshes[b + 3] = triangle_count;

        for (float value : vertices) detail.vertices.push(value);
        detail.vertex_count += (int)vertices.length / 3;

        for (int value : triangles) detail.triangles.push(value);
        detail.triangle_count += triangle_count;
    }

    hp.data.release();
    bounds.release();
    edges.release();
    triangles.release();
    queue.release();
    samples.release();
    vertices.release();
    poly.release();
    return detail;
}

void release(PolyMeshDetail &poly_mesh_detail) {
    poly_mesh_detail.meshes.release();
    poly_mesh_detail.vertices.release();
    poly_mesh_detail.triangles.release();
}

}

}
