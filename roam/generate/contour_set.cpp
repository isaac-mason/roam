#include "roam/generate/contour_set.h"

#include <math.h>
#include <string.h>

#include "roam/generate/common.h"
#include "roam/math/scalar.h"
#include "roam/util/sort.h"

namespace roam {

namespace contour_set {

static const int MAX_CONTOUR_WALK_ITERATIONS = 40000;

using compact_heightfield::get_con;

struct ContourPoint {
    int x;
    int z;
};

static uint32_t neighbor_index(const CompactHeightfield &chf, int x, int y, int direction, const CompactHeightfieldSpan &span) {
    int ax = x + get_dir_offset_x(direction);
    int ay = y + get_dir_offset_y(direction);
    return chf.cells[(uint32_t)(ax + ay * chf.width)].index + (uint32_t)get_con(span, direction);
}

static int region_and_area(const CompactHeightfield &chf, uint32_t i) { return chf.spans[i].region | (chf.areas[i] << 16); }

static int get_corner_height(int x, int y, uint32_t i, int direction, const CompactHeightfield &chf, bool &is_border_vertex) {
    const CompactHeightfieldSpan &s = chf.spans[i];
    int ch = s.y;
    int direction_p = (direction + 1) & 0x3;

    int regs[4] = {0, 0, 0, 0};

    regs[0] = region_and_area(chf, i);

    if (get_con(s, direction) != NOT_CONNECTED) {
        int ax = x + get_dir_offset_x(direction);
        int ay = y + get_dir_offset_y(direction);
        uint32_t ai = neighbor_index(chf, x, y, direction, s);
        const CompactHeightfieldSpan &as = chf.spans[ai];
        ch = scalar::max(ch, as.y);
        regs[1] = region_and_area(chf, ai);
        if (get_con(as, direction_p) != NOT_CONNECTED) {
            uint32_t ai2 = neighbor_index(chf, ax, ay, direction_p, as);
            ch = scalar::max(ch, chf.spans[ai2].y);
            regs[2] = region_and_area(chf, ai2);
        }
    }
    if (get_con(s, direction_p) != NOT_CONNECTED) {
        int ax = x + get_dir_offset_x(direction_p);
        int ay = y + get_dir_offset_y(direction_p);
        uint32_t ai = neighbor_index(chf, x, y, direction_p, s);
        const CompactHeightfieldSpan &as = chf.spans[ai];
        ch = scalar::max(ch, as.y);
        regs[3] = region_and_area(chf, ai);
        if (get_con(as, direction) != NOT_CONNECTED) {
            uint32_t ai2 = neighbor_index(chf, ax, ay, direction, as);
            ch = scalar::max(ch, chf.spans[ai2].y);
            regs[2] = region_and_area(chf, ai2);
        }
    }

    for (int j = 0; j < 4; ++j) {
        int a = j;
        int b = (j + 1) & 0x3;
        int c = (j + 2) & 0x3;
        int d = (j + 3) & 0x3;

        bool two_same_exteriors = (regs[a] & regs[b] & BORDER_REG) != 0 && regs[a] == regs[b];
        bool two_interiors = ((regs[c] | regs[d]) & BORDER_REG) == 0;
        bool interiors_same_area = regs[c] >> 16 == regs[d] >> 16;
        bool no_zeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;
        if (two_same_exteriors && two_interiors && interiors_same_area && no_zeros) {
            is_border_vertex = true;
            break;
        }
    }

    return ch;
}

static void push_point(Array<int> &points, int x, int y, int z, int w) {
    points.push(x);
    points.push(y);
    points.push(z);
    points.push(w);
}

static void walk_contour(BuildContext &context, int x, int y, uint32_t i, const CompactHeightfield &chf, int *flags, Array<int> &points) {
    int direction = 0;
    while ((flags[i] & (1 << direction)) == 0) direction++;

    int start_direction = direction;
    uint32_t start_i = i;

    uint8_t area = chf.areas[i];

    int iteration = 0;
    int current_x = x;
    int current_y = y;
    uint32_t current_i = i;

    uint32_t checkpoint_i = current_i;
    int checkpoint_direction = direction;
    int steps_since_checkpoint = 0;
    int checkpoint_interval = 1;

    while (++iteration < MAX_CONTOUR_WALK_ITERATIONS) {
        const CompactHeightfieldSpan &s = chf.spans[current_i];
        if (flags[current_i] & (1 << direction)) {
            bool is_border_vertex = false;
            bool is_area_border = false;
            int px = current_x;
            int py = get_corner_height(current_x, current_y, current_i, direction, chf, is_border_vertex);
            int pz = current_y;
            switch (direction) {
                case 0:
                    pz++;
                    break;
                case 1:
                    px++;
                    pz++;
                    break;
                case 2:
                    px++;
                    break;
            }
            int r = 0;
            if (get_con(s, direction) != NOT_CONNECTED) {
                uint32_t ai = neighbor_index(chf, current_x, current_y, direction, s);
                r = chf.spans[ai].region;
                if (area != chf.areas[ai]) is_area_border = true;
            }
            if (is_border_vertex) r |= BORDER_VERTEX;
            if (is_area_border) r |= AREA_BORDER;
            push_point(points, px, py, pz, r);

            flags[current_i] &= ~(1 << direction);
            direction = (direction + 1) & 0x3;

            checkpoint_i = current_i;
            checkpoint_direction = direction;
            steps_since_checkpoint = 0;
            checkpoint_interval = 1;
        } else {
            if (get_con(s, direction) == NOT_CONNECTED) {
                build_context::warn(context, "walkContour: encountered unexpected disconnected neighbour at (%d, %d)", current_x, current_y);
                return;
            }
            uint32_t ni = neighbor_index(chf, current_x, current_y, direction, s);
            current_x += get_dir_offset_x(direction);
            current_y += get_dir_offset_y(direction);
            current_i = ni;
            direction = (direction + 3) & 0x3;

            if (current_i == checkpoint_i && direction == checkpoint_direction) break;
            if (++steps_since_checkpoint == checkpoint_interval) {
                checkpoint_i = current_i;
                checkpoint_direction = direction;
                steps_since_checkpoint = 0;
                checkpoint_interval *= 2;
            }
        }

        if (start_i == current_i && start_direction == direction) break;
    }
}

static float distance_pt_seg(int x, int z, int px, int pz, int qx, int qz) {
    float pqx = (float)(qx - px);
    float pqz = (float)(qz - pz);
    float dx = (float)(x - px);
    float dz = (float)(z - pz);
    float d = pqx * pqx + pqz * pqz;
    float t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    float final_dx = (float)px + t * pqx - (float)x;
    float final_dz = (float)pz + t * pqz - (float)z;

    return final_dx * final_dx + final_dz * final_dz;
}

static void insert_simplified_point(Array<int> &simplified, int after, const Array<int> &points, int point_index) {
    simplified.resize(simplified.length + 4);
    int n = (int)simplified.length / 4;
    for (int j = n - 1; j > after; --j) {
        for (int k = 0; k < 4; k++) simplified[(uint32_t)(j * 4 + k)] = simplified[(uint32_t)((j - 1) * 4 + k)];
    }
    simplified[(uint32_t)((after + 1) * 4 + 0)] = points[(uint32_t)(point_index * 4 + 0)];
    simplified[(uint32_t)((after + 1) * 4 + 1)] = points[(uint32_t)(point_index * 4 + 1)];
    simplified[(uint32_t)((after + 1) * 4 + 2)] = points[(uint32_t)(point_index * 4 + 2)];
    simplified[(uint32_t)((after + 1) * 4 + 3)] = point_index;
}

static void simplify_contour(const Array<int> &points, Array<int> &simplified, float max_error, int max_edge_length, uint32_t build_flags) {
    bool has_connections = false;
    for (uint32_t i = 0; i < points.length; i += 4) {
        if ((points[i + 3] & CONTOUR_REG_MASK) != 0) {
            has_connections = true;
            break;
        }
    }

    if (has_connections) {
        for (int i = 0, ni = (int)points.length / 4; i < ni; ++i) {
            int ii = (i + 1) % ni;
            bool different_regions = (points[(uint32_t)(i * 4 + 3)] & CONTOUR_REG_MASK) != (points[(uint32_t)(ii * 4 + 3)] & CONTOUR_REG_MASK);
            bool area_borders = (points[(uint32_t)(i * 4 + 3)] & AREA_BORDER) != (points[(uint32_t)(ii * 4 + 3)] & AREA_BORDER);
            if (different_regions || area_borders) {
                push_point(simplified, points[(uint32_t)(i * 4 + 0)], points[(uint32_t)(i * 4 + 1)], points[(uint32_t)(i * 4 + 2)], i);
            }
        }
    }

    if (simplified.length == 0) {
        int llx = points[0];
        int lly = points[1];
        int llz = points[2];
        int lli = 0;
        int urx = points[0];
        int ury = points[1];
        int urz = points[2];
        int uri = 0;
        for (uint32_t i = 0; i < points.length; i += 4) {
            int x = points[i + 0];
            int y = points[i + 1];
            int z = points[i + 2];
            if (x < llx || (x == llx && z < llz)) {
                llx = x;
                lly = y;
                llz = z;
                lli = (int)i / 4;
            }
            if (x > urx || (x == urx && z > urz)) {
                urx = x;
                ury = y;
                urz = z;
                uri = (int)i / 4;
            }
        }
        push_point(simplified, llx, lly, llz, lli);
        push_point(simplified, urx, ury, urz, uri);
    }

    int pn = (int)points.length / 4;
    for (int i = 0; i < (int)simplified.length / 4;) {
        int ii = (i + 1) % ((int)simplified.length / 4);

        int ax = simplified[(uint32_t)(i * 4 + 0)];
        int az = simplified[(uint32_t)(i * 4 + 2)];
        int ai = simplified[(uint32_t)(i * 4 + 3)];

        int bx = simplified[(uint32_t)(ii * 4 + 0)];
        int bz = simplified[(uint32_t)(ii * 4 + 2)];
        int bi = simplified[(uint32_t)(ii * 4 + 3)];

        float maxd = 0;
        int maxi = -1;
        int ci;
        int cinc;
        int endi;

        int segment_ax = ax;
        int segment_az = az;
        int segment_bx = bx;
        int segment_bz = bz;
        if (bx > ax || (bx == ax && bz > az)) {
            cinc = 1;
            ci = (ai + cinc) % pn;
            endi = bi;
        } else {
            cinc = pn - 1;
            ci = (bi + cinc) % pn;
            endi = ai;
            segment_ax = bx;
            segment_bx = ax;
            segment_az = bz;
            segment_bz = az;
        }

        if ((points[(uint32_t)(ci * 4 + 3)] & CONTOUR_REG_MASK) == 0 || points[(uint32_t)(ci * 4 + 3)] & AREA_BORDER) {
            while (ci != endi) {
                float d = distance_pt_seg(points[(uint32_t)(ci * 4 + 0)], points[(uint32_t)(ci * 4 + 2)], segment_ax, segment_az, segment_bx, segment_bz);
                if (d > maxd) {
                    maxd = d;
                    maxi = ci;
                }
                ci = (ci + cinc) % pn;
            }
        }

        if (maxi != -1 && maxd > max_error * max_error) {
            insert_simplified_point(simplified, i, points, maxi);
        } else {
            ++i;
        }
    }

    if (max_edge_length > 0 && (build_flags & (CONTOUR_TESS_WALL_EDGES | CONTOUR_TESS_AREA_EDGES)) != 0) {
        for (int i = 0; i < (int)simplified.length / 4;) {
            int ii = (i + 1) % ((int)simplified.length / 4);

            int ax = simplified[(uint32_t)(i * 4 + 0)];
            int az = simplified[(uint32_t)(i * 4 + 2)];
            int ai = simplified[(uint32_t)(i * 4 + 3)];

            int bx = simplified[(uint32_t)(ii * 4 + 0)];
            int bz = simplified[(uint32_t)(ii * 4 + 2)];
            int bi = simplified[(uint32_t)(ii * 4 + 3)];

            int maxi = -1;
            int ci = (ai + 1) % pn;

            bool tessellate = false;
            if (build_flags & CONTOUR_TESS_WALL_EDGES && (points[(uint32_t)(ci * 4 + 3)] & CONTOUR_REG_MASK) == 0) tessellate = true;
            if (build_flags & CONTOUR_TESS_AREA_EDGES && points[(uint32_t)(ci * 4 + 3)] & AREA_BORDER) tessellate = true;

            if (tessellate) {
                int dx = bx - ax;
                int dz = bz - az;
                if (dx * dx + dz * dz > max_edge_length * max_edge_length) {
                    int n = bi < ai ? bi + pn - ai : bi - ai;
                    if (n > 1) {
                        if (bx > ax || (bx == ax && bz > az)) {
                            maxi = (ai + n / 2) % pn;
                        } else {
                            maxi = (ai + (n + 1) / 2) % pn;
                        }
                    }
                }
            }

            if (maxi != -1) {
                insert_simplified_point(simplified, i, points, maxi);
            } else {
                ++i;
            }
        }
    }

    for (uint32_t i = 0; i < simplified.length / 4; ++i) {
        int ai = (simplified[i * 4 + 3] + 1) % pn;
        int bi = simplified[i * 4 + 3];
        simplified[i * 4 + 3] = (points[(uint32_t)(ai * 4 + 3)] & (CONTOUR_REG_MASK | AREA_BORDER)) | (points[(uint32_t)(bi * 4 + 3)] & BORDER_VERTEX);
    }
}

static int floor_div_2(int value) { return (int)floor((double)value / 2.0); }

static int calc_area_of_polygon_2d(const Array<int> &vertices, int vertex_count) {
    int area = 0;
    for (int i = 0, j = vertex_count - 1; i < vertex_count; j = i++) {
        uint32_t vi = (uint32_t)i * 4;
        uint32_t vj = (uint32_t)j * 4;
        area += vertices[vi] * vertices[vj + 2] - vertices[vj] * vertices[vi + 2];
    }
    return floor_div_2(area + 1);
}

static int previous(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
static int next(int i, int n) { return i + 1 < n ? i + 1 : 0; }

static int area2(ContourPoint a, ContourPoint b, ContourPoint c) { return (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z); }

static bool left(ContourPoint a, ContourPoint b, ContourPoint c) { return area2(a, b, c) < 0; }

static bool left_on(ContourPoint a, ContourPoint b, ContourPoint c) { return area2(a, b, c) <= 0; }

static bool collinear(ContourPoint a, ContourPoint b, ContourPoint c) { return area2(a, b, c) == 0; }

static bool xorb(bool x, bool y) { return !x != !y; }

static bool intersect_prop(ContourPoint a, ContourPoint b, ContourPoint c, ContourPoint d) {
    if (collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b)) return false;
    return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
}

static bool between(ContourPoint a, ContourPoint b, ContourPoint c) {
    if (!collinear(a, b, c)) return false;
    if (a.x != b.x) return (a.x <= c.x && c.x <= b.x) || (a.x >= c.x && c.x >= b.x);
    return (a.z <= c.z && c.z <= b.z) || (a.z >= c.z && c.z >= b.z);
}

static bool intersect(ContourPoint a, ContourPoint b, ContourPoint c, ContourPoint d) {
    if (intersect_prop(a, b, c, d)) return true;
    return between(a, b, c) || between(a, b, d) || between(c, d, a) || between(c, d, b);
}

static ContourPoint contour_point(const Array<int> &vertices, int index) {
    return {vertices[(uint32_t)(index * 4)], vertices[(uint32_t)(index * 4 + 2)]};
}

static bool points_equal(ContourPoint a, ContourPoint b) { return a.x == b.x && a.z == b.z; }

static bool intersect_seg_contour(ContourPoint d0, ContourPoint d1, int i, int n, const Array<int> &vertices) {
    for (int k = 0; k < n; k++) {
        int k1 = next(k, n);
        if (i == k || i == k1) continue;

        ContourPoint p0 = contour_point(vertices, k);
        ContourPoint p1 = contour_point(vertices, k1);

        if (points_equal(d0, p0) || points_equal(d1, p0) || points_equal(d0, p1) || points_equal(d1, p1)) continue;

        if (intersect(d0, d1, p0, p1)) return true;
    }

    return false;
}

static bool in_cone(int i, int n, const Array<int> &vertices, ContourPoint pj) {
    ContourPoint pi = contour_point(vertices, i);
    ContourPoint pi1 = contour_point(vertices, next(i, n));
    ContourPoint pin1 = contour_point(vertices, previous(i, n));

    if (left_on(pin1, pi, pi1)) return left(pi, pj, pin1) && left(pj, pi, pi1);
    return !(left_on(pi, pj, pi1) && left_on(pj, pi, pin1));
}

static void remove_degenerate_segments(Array<int> &simplified) {
    int point_count = (int)simplified.length / 4;
    for (int i = point_count - 1; i >= 0; --i) {
        int ni = next(i, point_count);

        if (!points_equal(contour_point(simplified, i), contour_point(simplified, ni))) continue;

        for (int j = i; j < (int)simplified.length / 4 - 1; ++j) {
            for (int k = 0; k < 4; k++) simplified[(uint32_t)(j * 4 + k)] = simplified[(uint32_t)((j + 1) * 4 + k)];
        }
        simplified.resize(simplified.length - 4);
        point_count--;
    }
}

static void merge_contours(Contour &contour_a, Contour &contour_b, int ia, int ib) {
    Array<int> vertices = {};
    vertices.reserve((uint32_t)(contour_a.vertex_count + contour_b.vertex_count + 2) * 4);

    int vertex_count = 0;
    for (int i = 0; i <= contour_a.vertex_count; ++i) {
        uint32_t source = (uint32_t)(((ia + i) % contour_a.vertex_count) * 4);
        for (uint32_t k = 0; k < 4; k++) vertices.push(contour_a.vertices[source + k]);
        vertex_count++;
    }

    for (int i = 0; i <= contour_b.vertex_count; ++i) {
        uint32_t source = (uint32_t)(((ib + i) % contour_b.vertex_count) * 4);
        for (uint32_t k = 0; k < 4; k++) vertices.push(contour_b.vertices[source + k]);
        vertex_count++;
    }

    contour_a.vertices.release();
    contour_a.vertices = vertices;
    contour_a.vertex_count = vertex_count;

    contour_b.vertices.release();
    contour_b.vertex_count = 0;
}

struct ContourHole {
    uint32_t contour;
    int min_x;
    int min_z;
    int leftmost;
};

struct ContourRegion {
    int outline;
    Array<ContourHole> holes;
};

struct PotentialDiagonal {
    int vertex;
    int distance;
};

static void find_left_most_vertex(const Contour &contour, ContourHole &hole) {
    int min_x = contour.vertices[0];
    int min_z = contour.vertices[2];
    int leftmost = 0;
    for (int i = 1; i < contour.vertex_count; i++) {
        int x = contour.vertices[(uint32_t)(i * 4 + 0)];
        int z = contour.vertices[(uint32_t)(i * 4 + 2)];
        if (x < min_x || (x == min_x && z < min_z)) {
            min_x = x;
            min_z = z;
            leftmost = i;
        }
    }
    hole.min_x = min_x;
    hole.min_z = min_z;
    hole.leftmost = leftmost;
}

static bool hole_less(const ContourHole &a, const ContourHole &b) {
    if (a.min_x == b.min_x) return a.min_z < b.min_z;
    return a.min_x < b.min_x;
}

static void merge_region_holes(BuildContext &context, ContourSet &contour_set, ContourRegion &region) {
    for (ContourHole &hole : region.holes) find_left_most_vertex(contour_set.contours[hole.contour], hole);

    sort::stable(region.holes.data, region.holes.length, hole_less);

    Contour &outline = contour_set.contours[(uint32_t)region.outline];

    int max_vertices = outline.vertex_count;
    for (const ContourHole &hole : region.holes) max_vertices += contour_set.contours[hole.contour].vertex_count;

    Array<PotentialDiagonal> diagonals = {};
    diagonals.resize((uint32_t)max_vertices, {0, 0});

    for (uint32_t i = 0; i < region.holes.length; i++) {
        Contour &hole = contour_set.contours[region.holes[i].contour];

        int index = -1;
        int best_vertex = region.holes[i].leftmost;
        for (int iteration = 0; iteration < hole.vertex_count; iteration++) {
            int diagonal_count = 0;
            ContourPoint corner = contour_point(hole.vertices, best_vertex);

            for (int j = 0; j < outline.vertex_count; j++) {
                if (!in_cone(j, outline.vertex_count, outline.vertices, corner)) continue;
                int dx = outline.vertices[(uint32_t)(j * 4 + 0)] - corner.x;
                int dz = outline.vertices[(uint32_t)(j * 4 + 2)] - corner.z;
                diagonals[(uint32_t)diagonal_count] = {j, dx * dx + dz * dz};
                diagonal_count++;
            }

            if (diagonal_count > 1) {
                for (int a = 0; a < diagonal_count - 1; a++) {
                    for (int b = a + 1; b < diagonal_count; b++) {
                        if (diagonals[(uint32_t)a].distance > diagonals[(uint32_t)b].distance) {
                            PotentialDiagonal swap = diagonals[(uint32_t)a];
                            diagonals[(uint32_t)a] = diagonals[(uint32_t)b];
                            diagonals[(uint32_t)b] = swap;
                        }
                    }
                }
            }

            index = -1;
            for (int j = 0; j < diagonal_count; j++) {
                int vertex = diagonals[(uint32_t)j].vertex;
                ContourPoint point = contour_point(outline.vertices, vertex);

                bool intersects = intersect_seg_contour(point, corner, vertex, outline.vertex_count, outline.vertices);
                for (uint32_t k = i; k < region.holes.length && !intersects; k++) {
                    const Contour &other = contour_set.contours[region.holes[k].contour];
                    intersects = intersect_seg_contour(point, corner, -1, other.vertex_count, other.vertices);
                }

                if (!intersects) {
                    index = vertex;
                    break;
                }
            }

            if (index != -1) break;

            best_vertex = (best_vertex + 1) % hole.vertex_count;
        }

        if (index == -1) {
            build_context::warn(context, "mergeHoles: Failed to find merge points for outline and hole.");
            continue;
        }

        merge_contours(outline, hole, index, best_vertex);
    }

    diagonals.release();
}

ContourSet build_contours(BuildContext &context, const CompactHeightfield &chf, float max_simplification_error, int max_edge_length,
                          uint32_t build_flags) {
    int width = chf.width;
    int height = chf.height;
    int border_size = chf.border_size;

    ContourSet contour_set = {};
    contour_set.bounds = chf.bounds;
    contour_set.cell_size = chf.cell_size;
    contour_set.cell_height = chf.cell_height;
    contour_set.width = chf.width - chf.border_size * 2;
    contour_set.height = chf.height - chf.border_size * 2;
    contour_set.border_size = chf.border_size;
    contour_set.max_error = max_simplification_error;

    if (border_size > 0) {
        float pad = (float)border_size * chf.cell_size;
        contour_set.bounds.min.x += pad;
        contour_set.bounds.min.z += pad;
        contour_set.bounds.max.x -= pad;
        contour_set.bounds.max.z -= pad;
    }

    Array<int> flags = {};
    flags.resize(chf.spans.length, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            CompactHeightfieldCell c = chf.cells[(uint32_t)(x + y * width)];
            for (uint32_t i = c.index; i < c.index + c.count; ++i) {
                const CompactHeightfieldSpan &s = chf.spans[i];
                if (!s.region || s.region & BORDER_REG) {
                    flags[i] = 0;
                    continue;
                }
                int result = 0;
                for (int direction = 0; direction < 4; ++direction) {
                    int r = 0;
                    if (get_con(s, direction) != NOT_CONNECTED) r = chf.spans[neighbor_index(chf, x, y, direction, s)].region;
                    if (r == s.region) result |= 1 << direction;
                }
                flags[i] = result ^ 0xf;
            }
        }
    }

    Array<int> vertices = {};
    Array<int> simplified = {};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            CompactHeightfieldCell c = chf.cells[(uint32_t)(x + y * width)];
            for (uint32_t i = c.index; i < c.index + c.count; ++i) {
                if (flags[i] == 0 || flags[i] == 0xf) {
                    flags[i] = 0;
                    continue;
                }
                int region = chf.spans[i].region;
                if (!region || region & BORDER_REG) continue;
                uint8_t area = chf.areas[i];

                vertices.clear();
                simplified.clear();

                walk_contour(context, x, y, i, chf, flags.data, vertices);
                simplify_contour(vertices, simplified, max_simplification_error, max_edge_length, build_flags);
                remove_degenerate_segments(simplified);

                if ((int)simplified.length / 4 < 3) continue;

                Contour contour = {};
                contour.vertex_count = (int)simplified.length / 4;
                contour.vertices.resize(simplified.length);
                memcpy(contour.vertices.data, simplified.data, simplified.length * sizeof(int));
                contour.raw_vertex_count = (int)vertices.length / 4;
                contour.raw_vertices.resize(vertices.length);
                memcpy(contour.raw_vertices.data, vertices.data, vertices.length * sizeof(int));
                contour.reg = region;
                contour.area = area;

                if (border_size > 0) {
                    for (int j = 0; j < contour.vertex_count; ++j) {
                        contour.vertices[(uint32_t)(j * 4 + 0)] -= border_size;
                        contour.vertices[(uint32_t)(j * 4 + 2)] -= border_size;
                    }
                    for (int j = 0; j < contour.raw_vertex_count; ++j) {
                        contour.raw_vertices[(uint32_t)(j * 4 + 0)] -= border_size;
                        contour.raw_vertices[(uint32_t)(j * 4 + 2)] -= border_size;
                    }
                }

                contour_set.contours.push(contour);
            }
        }
    }

    vertices.release();
    simplified.release();
    flags.release();

    if (contour_set.contours.length == 0) return contour_set;

    Array<int> winding = {};
    int hole_count = 0;
    for (const Contour &contour : contour_set.contours) {
        int contour_winding = calc_area_of_polygon_2d(contour.vertices, contour.vertex_count) < 0 ? -1 : 1;
        winding.push(contour_winding);
        if (contour_winding < 0) hole_count++;
    }

    if (hole_count > 0) {
        uint32_t region_count = (uint32_t)chf.max_regions + 1;
        Array<ContourRegion> regions = {};
        regions.resize(region_count, {-1, {}});

        for (uint32_t i = 0; i < contour_set.contours.length; ++i) {
            const Contour &contour = contour_set.contours[i];
            ContourRegion &region = regions[(uint32_t)contour.reg];

            if (winding[i] > 0) {
                if (region.outline != -1) build_context::error(context, "buildContours: Multiple outlines for region %d.", contour.reg);
                region.outline = (int)i;
            } else {
                region.holes.push({i, 0, 0, 0});
            }
        }

        for (uint32_t i = 0; i < region_count; i++) {
            ContourRegion &region = regions[i];
            if (region.holes.length == 0) continue;

            if (region.outline != -1) {
                merge_region_holes(context, contour_set, region);
            } else {
                build_context::error(context, "buildContours: Bad outline for region %d, contour simplification is likely too aggressive.",
                                     (int)i);
            }
        }

        for (ContourRegion &region : regions) region.holes.release();
        regions.release();
    }

    winding.release();
    return contour_set;
}

void release(ContourSet &contour_set) {
    for (Contour &contour : contour_set.contours) {
        contour.vertices.release();
        contour.raw_vertices.release();
    }
    contour_set.contours.release();
}

}

}
