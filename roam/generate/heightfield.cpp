#include "roam/generate/heightfield.h"

#include <math.h>

#include "roam/generate/common.h"
#include "roam/math/scalar.h"

namespace roam {

namespace heightfield {

static const int SPAN_MAX_HEIGHT = 0x1fff;
static const int MAX_HEIGHTFIELD_HEIGHT = 0xffff;

GridSize calculate_grid_size(Box3 bounds, float cell_size) {
    return {(int)floorf((bounds.max.x - bounds.min.x) / cell_size + 0.5f), (int)floorf((bounds.max.z - bounds.min.z) / cell_size + 0.5f)};
}

Heightfield create(int width, int height, Box3 bounds, float cell_size, float cell_height) {
    Heightfield heightfield = {};
    heightfield.width = width;
    heightfield.height = height;
    heightfield.bounds = bounds;
    heightfield.cell_size = cell_size;
    heightfield.cell_height = cell_height;
    heightfield.spans.resize((uint32_t)(width * height), NULL_SPAN);
    heightfield.free_span = NULL_SPAN;
    return heightfield;
}

void release(Heightfield &heightfield) {
    heightfield.spans.release();
    heightfield.span_pool.release();
    heightfield.free_span = NULL_SPAN;
}

static int32_t allocate_span(Heightfield &heightfield) {
    if (heightfield.free_span != NULL_SPAN) {
        int32_t index = heightfield.free_span;
        heightfield.free_span = heightfield.span_pool[(uint32_t)index].next;
        return index;
    }
    heightfield.span_pool.push({});
    return (int32_t)heightfield.span_pool.length - 1;
}

static void free_span(Heightfield &heightfield, int32_t index) {
    heightfield.span_pool[(uint32_t)index].next = heightfield.free_span;
    heightfield.free_span = index;
}

bool add_heightfield_span(Heightfield &heightfield, int x, int z, int min, int max, uint8_t area_id, int flag_merge_threshold) {
    uint32_t column_index = (uint32_t)(x + z * heightfield.width);
    int new_min = min;
    int new_max = max;
    uint8_t new_area = area_id;

    int32_t previous = NULL_SPAN;
    int32_t current = heightfield.spans[column_index];

    while (current != NULL_SPAN) {
        HeightfieldSpan current_span = heightfield.span_pool[(uint32_t)current];

        if (current_span.min > new_max) break;

        if (current_span.max < new_min) {
            previous = current;
            current = current_span.next;
            continue;
        }

        if (current_span.min < new_min) new_min = current_span.min;
        if (current_span.max > new_max) new_max = current_span.max;

        if (scalar::abs(new_max - current_span.max) <= flag_merge_threshold) {
            new_area = new_area > current_span.area ? new_area : current_span.area;
        }

        int32_t next = current_span.next;
        if (previous != NULL_SPAN) {
            heightfield.span_pool[(uint32_t)previous].next = next;
        } else {
            heightfield.spans[column_index] = next;
        }
        free_span(heightfield, current);
        current = next;
    }

    int32_t index = allocate_span(heightfield);
    HeightfieldSpan &span = heightfield.span_pool[(uint32_t)index];
    span.min = new_min;
    span.max = new_max;
    span.area = new_area;

    if (previous != NULL_SPAN) {
        span.next = heightfield.span_pool[(uint32_t)previous].next;
        heightfield.span_pool[(uint32_t)previous].next = index;
    } else {
        span.next = heightfield.spans[column_index];
        heightfield.spans[column_index] = index;
    }

    return true;
}

struct DividedPoly {
    int vertex_count_1;
    int vertex_count_2;
};

static void push_vertex(float *vertices, int &count, float x, float y, float z) {
    vertices[count * 3] = x;
    vertices[count * 3 + 1] = y;
    vertices[count * 3 + 2] = z;
    count++;
}

static DividedPoly divide_poly(const float *in_vertices, int in_vertex_count, float *out_vertices_1, float *out_vertices_2, float axis_offset,
                               int axis) {
    float in_vertex_axis_delta[12];
    for (int in_vertex = 0; in_vertex < in_vertex_count; ++in_vertex) {
        in_vertex_axis_delta[in_vertex] = axis_offset - in_vertices[in_vertex * 3 + axis];
    }

    int poly_1_vertex = 0;
    int poly_2_vertex = 0;

    for (int in_vertex_a = 0, in_vertex_b = in_vertex_count - 1; in_vertex_a < in_vertex_count; in_vertex_b = in_vertex_a, ++in_vertex_a) {
        float delta_a = in_vertex_axis_delta[in_vertex_a];
        float delta_b = in_vertex_axis_delta[in_vertex_b];

        float ax = in_vertices[in_vertex_a * 3];
        float ay = in_vertices[in_vertex_a * 3 + 1];
        float az = in_vertices[in_vertex_a * 3 + 2];

        if ((delta_a >= 0) != (delta_b >= 0)) {
            float bx = in_vertices[in_vertex_b * 3];
            float by = in_vertices[in_vertex_b * 3 + 1];
            float bz = in_vertices[in_vertex_b * 3 + 2];

            float s = delta_b / (delta_b - delta_a);
            float cx = bx + (ax - bx) * s;
            float cy = by + (ay - by) * s;
            float cz = bz + (az - bz) * s;

            push_vertex(out_vertices_1, poly_1_vertex, cx, cy, cz);
            push_vertex(out_vertices_2, poly_2_vertex, cx, cy, cz);

            if (delta_a > 0) {
                push_vertex(out_vertices_1, poly_1_vertex, ax, ay, az);
            } else if (delta_a < 0) {
                push_vertex(out_vertices_2, poly_2_vertex, ax, ay, az);
            }
        } else {
            if (delta_a >= 0) {
                push_vertex(out_vertices_1, poly_1_vertex, ax, ay, az);
                if (delta_a != 0) continue;
            }
            push_vertex(out_vertices_2, poly_2_vertex, ax, ay, az);
        }
    }

    return {poly_1_vertex, poly_2_vertex};
}

struct CellPiece {
    int vertex_count;
    float min_y;
    float max_y;
};

static void include_cell_height(CellPiece &piece, float y) {
    if (piece.vertex_count == 0) {
        piece.min_y = y;
        piece.max_y = y;
    } else {
        piece.min_y = fminf(piece.min_y, y);
        piece.max_y = fmaxf(piece.max_y, y);
    }
    piece.vertex_count++;
}

static CellPiece divide_cell(const float *in_vertices, int in_vertex_count, float *out_far_vertices, int &far_vertex_count, float axis_offset) {
    float in_vertex_axis_delta[12];
    for (int in_vertex = 0; in_vertex < in_vertex_count; ++in_vertex) in_vertex_axis_delta[in_vertex] = axis_offset - in_vertices[in_vertex * 3];

    CellPiece piece = {0, 0, 0};
    far_vertex_count = 0;

    for (int in_vertex_a = 0, in_vertex_b = in_vertex_count - 1; in_vertex_a < in_vertex_count; in_vertex_b = in_vertex_a, ++in_vertex_a) {
        float delta_a = in_vertex_axis_delta[in_vertex_a];
        float delta_b = in_vertex_axis_delta[in_vertex_b];

        float ax = in_vertices[in_vertex_a * 3];
        float ay = in_vertices[in_vertex_a * 3 + 1];
        float az = in_vertices[in_vertex_a * 3 + 2];

        if ((delta_a >= 0) != (delta_b >= 0)) {
            float bx = in_vertices[in_vertex_b * 3];
            float by = in_vertices[in_vertex_b * 3 + 1];
            float bz = in_vertices[in_vertex_b * 3 + 2];

            float s = delta_b / (delta_b - delta_a);
            float cx = bx + (ax - bx) * s;
            float cy = by + (ay - by) * s;
            float cz = bz + (az - bz) * s;

            include_cell_height(piece, cy);
            push_vertex(out_far_vertices, far_vertex_count, cx, cy, cz);

            if (delta_a > 0) {
                include_cell_height(piece, ay);
            } else if (delta_a < 0) {
                push_vertex(out_far_vertices, far_vertex_count, ax, ay, az);
            }
        } else {
            if (delta_a >= 0) {
                include_cell_height(piece, ay);
                if (delta_a != 0) continue;
            }
            push_vertex(out_far_vertices, far_vertex_count, ax, ay, az);
        }
    }

    return piece;
}

static bool all_on_near_side(const float *vertices, int vertex_count, float axis_offset, int axis) {
    for (int vertex = 0; vertex < vertex_count; ++vertex) {
        if (!(axis_offset - vertices[vertex * 3 + axis] >= 0)) return false;
    }
    return true;
}

static bool rasterize_triangle(Vec3 v0, Vec3 v1, Vec3 v2, uint8_t area_id, Heightfield &heightfield, int flag_merge_threshold) {
    Vec3 triangle_min = vec3::min(vec3::min(v0, v1), v2);
    Vec3 triangle_max = vec3::max(vec3::max(v0, v1), v2);

    if (!box3::intersects({triangle_min, triangle_max}, heightfield.bounds)) return true;

    int w = heightfield.width;
    int h = heightfield.height;
    float by = heightfield.bounds.max.y - heightfield.bounds.min.y;
    float cell_size = heightfield.cell_size;
    float inverse_cell_size = 1.0f / cell_size;
    float inverse_cell_height = 1.0f / heightfield.cell_height;

    int z0 = (int)floorf((triangle_min.z - heightfield.bounds.min.z) * inverse_cell_size);
    int z1 = (int)floorf((triangle_max.z - heightfield.bounds.min.z) * inverse_cell_size);

    z0 = scalar::clamp(z0, -1, h - 1);
    z1 = scalar::clamp(z1, 0, h - 1);

    float buffer_a[7 * 3];
    float buffer_b[7 * 3];
    float buffer_c[7 * 3];
    float buffer_d[7 * 3];

    float *in_vertices = buffer_a;
    float *in_row = buffer_b;
    float *p1 = buffer_c;
    float *p2 = buffer_d;

    vec3::store(in_vertices, 0, v0);
    vec3::store(in_vertices, 3, v1);
    vec3::store(in_vertices, 6, v2);

    int in_vertex_count = 3;

    for (int z = z0; z <= z1; ++z) {
        float cell_z = heightfield.bounds.min.z + (float)z * cell_size;
        int row_vertex_count = 0;
        if (z == z1 && all_on_near_side(in_vertices, in_vertex_count, cell_z + cell_size, common::AXIS_Z)) {
            float *swap = in_row;
            in_row = in_vertices;
            in_vertices = swap;
            row_vertex_count = in_vertex_count;
            in_vertex_count = 0;
        } else {
            DividedPoly row = divide_poly(in_vertices, in_vertex_count, in_row, p1, cell_z + cell_size, common::AXIS_Z);

            float *swap = in_vertices;
            in_vertices = p1;
            p1 = swap;
            in_vertex_count = row.vertex_count_2;
            row_vertex_count = row.vertex_count_1;
        }

        if (row_vertex_count < 3) continue;
        if (z < 0) continue;

        float min_x = in_row[0];
        float max_x = in_row[0];
        for (int vertex = 1; vertex < row_vertex_count; ++vertex) {
            if (min_x > in_row[vertex * 3]) min_x = in_row[vertex * 3];
            if (max_x < in_row[vertex * 3]) max_x = in_row[vertex * 3];
        }

        int x0 = (int)floorf((min_x - heightfield.bounds.min.x) * inverse_cell_size);
        int x1 = (int)floorf((max_x - heightfield.bounds.min.x) * inverse_cell_size);
        if (x1 < 0 || x0 >= w) continue;
        x0 = scalar::clamp(x0, -1, w - 1);
        x1 = scalar::clamp(x1, 0, w - 1);

        int remaining_row_vertex_count = row_vertex_count;

        for (int x = x0; x <= x1; ++x) {
            float cx = heightfield.bounds.min.x + (float)x * cell_size;
            CellPiece piece;
            if (x == x1 && all_on_near_side(in_row, remaining_row_vertex_count, cx + cell_size, common::AXIS_X)) {
                piece = {0, 0, 0};
                for (int vertex = 0; vertex < remaining_row_vertex_count; ++vertex) include_cell_height(piece, in_row[vertex * 3 + 1]);
                remaining_row_vertex_count = 0;
            } else {
                int far_vertex_count = 0;
                piece = divide_cell(in_row, remaining_row_vertex_count, p2, far_vertex_count, cx + cell_size);

                float *swap_row = in_row;
                in_row = p2;
                p2 = swap_row;
                remaining_row_vertex_count = far_vertex_count;
            }

            if (piece.vertex_count < 3) continue;
            if (x < 0) continue;

            float span_min = piece.min_y;
            float span_max = piece.max_y;
            span_min -= heightfield.bounds.min.y;
            span_max -= heightfield.bounds.min.y;

            if (span_max < 0.0f) continue;
            if (span_min > by) continue;

            if (span_min < 0.0f) span_min = 0;
            if (span_max > by) span_max = by;

            int span_min_cell_index = scalar::clamp((int)floorf(span_min * inverse_cell_height), 0, SPAN_MAX_HEIGHT);
            int span_max_cell_index = scalar::clamp((int)ceilf(span_max * inverse_cell_height), span_min_cell_index + 1, SPAN_MAX_HEIGHT);

            if (!add_heightfield_span(heightfield, x, z, span_min_cell_index, span_max_cell_index, area_id, flag_merge_threshold)) return false;
        }
    }

    return true;
}

bool rasterize_triangles(BuildContext &context, Heightfield &heightfield, const InputTriangleMesh &mesh, Span<const uint8_t> tri_area_ids,
                         int flag_merge_threshold) {
    const float *vertices = mesh.positions.data;
    const uint32_t *indices = mesh.indices.data;
    uint32_t index_count = mesh.indices.capacity;
    ROAM_ASSERT(tri_area_ids.capacity >= index_count / 3);

    uint32_t triangle_count = index_count / 3;

    for (uint32_t triangle = 0; triangle < triangle_count; ++triangle) {
        Vec3 v0 = vec3::load(vertices, indices[triangle * 3 + 0] * 3);
        Vec3 v1 = vec3::load(vertices, indices[triangle * 3 + 1] * 3);
        Vec3 v2 = vec3::load(vertices, indices[triangle * 3 + 2] * 3);

        if (!rasterize_triangle(v0, v1, v2, tri_area_ids[triangle], heightfield, flag_merge_threshold)) {
            build_context::error(context, "Failed to rasterize triangle");
            return false;
        }
    }

    return true;
}

static HeightfieldSpan *span_at(Heightfield &heightfield, int32_t index) {
    return index == NULL_SPAN ? nullptr : &heightfield.span_pool[(uint32_t)index];
}

void filter_low_hanging_walkable_obstacles(Heightfield &heightfield, int walkable_climb) {
    int x_size = heightfield.width;
    int z_size = heightfield.height;

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            HeightfieldSpan *previous_span = nullptr;
            bool previous_was_walkable = false;
            uint8_t previous_area_id = NULL_AREA;

            HeightfieldSpan *span = span_at(heightfield, heightfield.spans[(uint32_t)(x + z * x_size)]);

            while (span != nullptr) {
                bool walkable = span->area != NULL_AREA;

                if (!walkable && previous_was_walkable && previous_span && span->max - previous_span->max <= walkable_climb) {
                    span->area = previous_area_id;
                }

                previous_was_walkable = walkable;
                previous_area_id = span->area;
                previous_span = span;
                span = span_at(heightfield, span->next);
            }
        }
    }
}

void filter_ledge_spans(Heightfield &heightfield, int walkable_height, int walkable_climb) {
    int x_size = heightfield.width;
    int z_size = heightfield.height;

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            HeightfieldSpan *span = span_at(heightfield, heightfield.spans[(uint32_t)(x + z * x_size)]);

            while (span != nullptr) {
                if (span->area == NULL_AREA) {
                    span = span_at(heightfield, span->next);
                    continue;
                }

                int floor = span->max;
                HeightfieldSpan *next_span = span_at(heightfield, span->next);
                int ceiling = next_span ? next_span->min : MAX_HEIGHTFIELD_HEIGHT;

                int lowest_neighbor_floor_difference = MAX_HEIGHTFIELD_HEIGHT;

                int lowest_traversable_neighbor_floor = span->max;
                int highest_traversable_neighbor_floor = span->max;

                for (int direction = 0; direction < 4; ++direction) {
                    int neighbor_x = x + common::get_dir_offset_x(direction);
                    int neighbor_z = z + common::get_dir_offset_y(direction);

                    if (neighbor_x < 0 || neighbor_z < 0 || neighbor_x >= x_size || neighbor_z >= z_size) {
                        lowest_neighbor_floor_difference = -walkable_climb - 1;
                        break;
                    }

                    HeightfieldSpan *neighbor_span = span_at(heightfield, heightfield.spans[(uint32_t)(neighbor_x + neighbor_z * x_size)]);

                    int neighbor_ceiling = neighbor_span ? neighbor_span->min : MAX_HEIGHTFIELD_HEIGHT;

                    if (scalar::min(ceiling, neighbor_ceiling) - floor >= walkable_height) {
                        lowest_neighbor_floor_difference = -walkable_climb - 1;
                        break;
                    }

                    while (neighbor_span != nullptr) {
                        int neighbor_floor = neighbor_span->max;
                        HeightfieldSpan *neighbor_next = span_at(heightfield, neighbor_span->next);
                        neighbor_ceiling = neighbor_next ? neighbor_next->min : MAX_HEIGHTFIELD_HEIGHT;

                        if (scalar::min(ceiling, neighbor_ceiling) - scalar::max(floor, neighbor_floor) < walkable_height) {
                            neighbor_span = neighbor_next;
                            continue;
                        }

                        int neighbor_floor_difference = neighbor_floor - floor;
                        lowest_neighbor_floor_difference = scalar::min(lowest_neighbor_floor_difference, neighbor_floor_difference);

                        if (scalar::abs(neighbor_floor_difference) <= walkable_climb) {
                            lowest_traversable_neighbor_floor = scalar::min(lowest_traversable_neighbor_floor, neighbor_floor);
                            highest_traversable_neighbor_floor = scalar::max(highest_traversable_neighbor_floor, neighbor_floor);
                        } else if (neighbor_floor_difference < -walkable_climb) {
                            break;
                        }

                        neighbor_span = neighbor_next;
                    }
                }

                if (lowest_neighbor_floor_difference < -walkable_climb) {
                    span->area = NULL_AREA;
                } else if (highest_traversable_neighbor_floor - lowest_traversable_neighbor_floor > walkable_climb) {
                    span->area = NULL_AREA;
                }

                span = next_span;
            }
        }
    }
}

void filter_walkable_low_height_spans(Heightfield &heightfield, int walkable_height) {
    int x_size = heightfield.width;
    int z_size = heightfield.height;

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            HeightfieldSpan *span = span_at(heightfield, heightfield.spans[(uint32_t)(x + z * x_size)]);

            while (span != nullptr) {
                HeightfieldSpan *next_span = span_at(heightfield, span->next);
                int floor = span->max;
                int ceiling = next_span ? next_span->min : MAX_HEIGHTFIELD_HEIGHT;

                if (ceiling - floor < walkable_height) span->area = NULL_AREA;

                span = next_span;
            }
        }
    }
}

static const float EPSILON = 0.00001f;
static const int BOX_EDGES[24] = {0, 1, 0, 2, 0, 4, 1, 3, 1, 5, 2, 3, 2, 6, 3, 7, 4, 5, 4, 6, 5, 7, 6, 7};

struct CellRect {
    float x_min;
    float z_min;
    float x_max;
    float z_max;
    float y_base;
};

struct YSpan {
    bool hit;
    float min;
    float max;
};

static const YSpan NO_Y_SPAN = {false, 0, 0};

struct Plane {
    Vec3 normal;
    float distance;
};

static YSpan merge_intersections(YSpan a, YSpan b) {
    if (!a.hit) return b;
    if (!b.hit) return a;
    return {true, fminf(a.min, b.min), fmaxf(a.max, b.max)};
}

static YSpan include_height(YSpan span, float y) {
    if (!span.hit) return {true, y, y};
    return {true, fminf(span.min, y), fmaxf(span.max, y)};
}

static bool overlap_bounds(Box3 a, Box3 b) {
    if (a.min.x > b.max.x || a.max.x < b.min.x) return false;
    if (a.min.y > b.max.y || a.max.y < b.min.y) return false;
    if (a.min.z > b.max.z || a.max.z < b.min.z) return false;
    return true;
}

static int round_half_up(float value) { return (int)floorf(value + 0.5f); }

template <typename Intersection>
static void rasterization_filled_shape(Heightfield &heightfield, Box3 bounds, uint8_t area, int flag_merge_threshold, Intersection intersection) {
    if (!overlap_bounds(heightfield.bounds, bounds)) return;

    bounds.max.x = fminf(bounds.max.x, heightfield.bounds.max.x);
    bounds.max.z = fminf(bounds.max.z, heightfield.bounds.max.z);
    bounds.min.x = fmaxf(bounds.min.x, heightfield.bounds.min.x);
    bounds.min.z = fmaxf(bounds.min.z, heightfield.bounds.min.z);

    if (bounds.max.x <= bounds.min.x || bounds.max.y <= bounds.min.y || bounds.max.z <= bounds.min.z) return;

    float inverse_cell_size = 1.0f / heightfield.cell_size;
    float inverse_cell_height = 1.0f / heightfield.cell_height;
    int x_min = round_half_up((bounds.min.x - heightfield.bounds.min.x) * inverse_cell_size);
    int z_min = round_half_up((bounds.min.z - heightfield.bounds.min.z) * inverse_cell_size);
    int x_max = scalar::min(heightfield.width - 1, round_half_up((bounds.max.x - heightfield.bounds.min.x) * inverse_cell_size));
    int z_max = scalar::min(heightfield.height - 1, round_half_up((bounds.max.z - heightfield.bounds.min.z) * inverse_cell_size));

    CellRect cell;
    cell.y_base = heightfield.bounds.min.y;
    for (int x = x_min; x <= x_max; x++) {
        for (int z = z_min; z <= z_max; z++) {
            cell.x_min = (float)x * heightfield.cell_size + heightfield.bounds.min.x;
            cell.z_min = (float)z * heightfield.cell_size + heightfield.bounds.min.z;
            cell.x_max = cell.x_min + heightfield.cell_size;
            cell.z_max = cell.z_min + heightfield.cell_size;

            YSpan span = intersection(cell);
            if (!span.hit) continue;

            int span_min = (int)floorf((span.min - heightfield.bounds.min.y) * inverse_cell_height);
            int span_max = (int)ceilf((span.max - heightfield.bounds.min.y) * inverse_cell_height);
            if (span_min == span_max) continue;

            int clamped_min = scalar::clamp(span_min, 0, SPAN_MAX_HEIGHT);
            int clamped_max = scalar::clamp(span_max, clamped_min + 1, SPAN_MAX_HEIGHT);
            add_heightfield_span(heightfield, x, z, clamped_min, clamped_max, area, flag_merge_threshold);
        }
    }
}

static YSpan intersect_sphere(const CellRect &cell, Vec3 center, float radius_sqr) {
    float x = fmaxf(cell.x_min, fminf(center.x, cell.x_max));
    float y = cell.y_base;
    float z = fmaxf(cell.z_min, fminf(center.z, cell.z_max));

    float mx = x - center.x;
    float my = y - center.y;
    float mz = z - center.z;

    float b = my;
    float c = mx * mx + my * my + mz * mz - radius_sqr;
    if (c > 0.0f && b > 0.0f) return NO_Y_SPAN;

    float discriminant = b * b - c;
    if (discriminant < 0.0f) return NO_Y_SPAN;

    float discriminant_sqrt = sqrtf(discriminant);
    float tmin = -b - discriminant_sqrt;
    float tmax = -b + discriminant_sqrt;

    if (tmin < 0.0f) tmin = 0.0f;
    return {true, y + tmin, y + tmax};
}

static YSpan ray_cylinder_intersection(Vec3 point, Vec3 start, Vec3 axis, float radius_sqr) {
    Vec3 d = axis;
    Vec3 m = vec3::subtract(point, start);
    float md = vec3::dot(m, d);
    float nd = axis.y;
    float dd = vec3::dot(d, d);

    float nn = 1;
    float mn = m.y;
    float a = dd - nd * nd;
    float k = vec3::dot(m, m) - radius_sqr;
    float c = dd * k - md * md;
    if (fabsf(a) < EPSILON) {
        if (c > 0.0f) return NO_Y_SPAN;
        float t1 = -mn / nn;
        float t2 = (nd - mn) / nn;
        return {true, point.y + fminf(t1, t2), point.y + fmaxf(t1, t2)};
    }
    float b = dd * mn - nd * md;
    float discriminant = b * b - a * c;
    if (discriminant < 0.0f) return NO_Y_SPAN;

    float discriminant_sqrt = sqrtf(discriminant);
    float t1 = (-b - discriminant_sqrt) / a;
    float t2 = (-b + discriminant_sqrt) / a;

    if (md + t1 * nd < 0.0f) {
        t1 = -md / nd;
        if (k + t1 * (2 * mn + t1 * nn) > 0.0f) return NO_Y_SPAN;
    } else if (md + t1 * nd > dd) {
        t1 = (dd - md) / nd;
        if (k + dd - 2 * md + t1 * (2 * (mn - nd) + t1 * nn) > 0.0f) return NO_Y_SPAN;
    }
    if (md + t2 * nd < 0.0f) {
        t2 = -md / nd;
        if (k + t2 * (2 * mn + t2 * nn) > 0.0f) return NO_Y_SPAN;
    } else if (md + t2 * nd > dd) {
        t2 = (dd - md) / nd;
        if (k + dd - 2 * md + t2 * (2 * (mn - nd) + t2 * nn) > 0.0f) return NO_Y_SPAN;
    }
    return {true, point.y + fminf(t1, t2), point.y + fmaxf(t1, t2)};
}

static Vec3 x_slab_ray_intersection(const CellRect &cell, Vec3 start, Vec3 direction, float x) {
    float t = (x - start.x) / direction.x;
    return {x, cell.y_base, scalar::clamp(start.z + t * direction.z, cell.z_min, cell.z_max)};
}

static Vec3 z_slab_ray_intersection(const CellRect &cell, Vec3 start, Vec3 direction, float z) {
    float t = (z - start.z) / direction.z;
    return {scalar::clamp(start.x + t * direction.x, cell.x_min, cell.x_max), cell.y_base, z};
}

static YSpan x_slab_cylinder_intersection(const CellRect &cell, Vec3 start, Vec3 axis, float radius_sqr, float x) {
    return ray_cylinder_intersection(x_slab_ray_intersection(cell, start, axis, x), start, axis, radius_sqr);
}

static YSpan z_slab_cylinder_intersection(const CellRect &cell, Vec3 start, Vec3 axis, float radius_sqr, float z) {
    return ray_cylinder_intersection(z_slab_ray_intersection(cell, start, axis, z), start, axis, radius_sqr);
}

static YSpan slabs_cylinder_intersection(const CellRect &cell, Vec3 start, Vec3 end, Vec3 axis, float radius_sqr, YSpan span) {
    if (fminf(start.x, end.x) < cell.x_min) span = merge_intersections(span, x_slab_cylinder_intersection(cell, start, axis, radius_sqr, cell.x_min));
    if (fmaxf(start.x, end.x) > cell.x_max) span = merge_intersections(span, x_slab_cylinder_intersection(cell, start, axis, radius_sqr, cell.x_max));
    if (fminf(start.z, end.z) < cell.z_min) span = merge_intersections(span, z_slab_cylinder_intersection(cell, start, axis, radius_sqr, cell.z_min));
    if (fmaxf(start.z, end.z) > cell.z_max) span = merge_intersections(span, z_slab_cylinder_intersection(cell, start, axis, radius_sqr, cell.z_max));
    return span;
}

static YSpan intersect_capsule(const CellRect &cell, Vec3 start, Vec3 end, Vec3 axis, float radius_sqr) {
    YSpan span = merge_intersections(intersect_sphere(cell, start, radius_sqr), intersect_sphere(cell, end, radius_sqr));
    float axis_length_2d_sqr = axis.x * axis.x + axis.z * axis.z;
    if (axis_length_2d_sqr > EPSILON) span = slabs_cylinder_intersection(cell, start, end, axis, radius_sqr, span);
    return span;
}

static YSpan cylinder_cap_intersection(Vec3 start, float radius_sqr, YSpan span, int i, const Vec3 *rectangle_on_plane) {
    int j = (i + 1) % 4;
    Vec3 m = vec3::subtract(rectangle_on_plane[i], start);
    Vec3 d = vec3::subtract(rectangle_on_plane[j], rectangle_on_plane[i]);
    float dl = vec3::dot(d, d);
    float b = vec3::dot(m, d) / dl;
    float c = (vec3::dot(m, m) - radius_sqr) / dl;
    float discriminant = b * b - c;
    if (discriminant > EPSILON) {
        float discriminant_sqrt = sqrtf(discriminant);
        float t1 = -b - discriminant_sqrt;
        float t2 = -b + discriminant_sqrt;
        if (t1 <= 1 && t2 >= 0) {
            t1 = fmaxf(0, t1);
            t2 = fminf(1, t2);
            float y1 = rectangle_on_plane[i].y + t1 * d.y;
            float y2 = rectangle_on_plane[i].y + t2 * d.y;
            span = merge_intersections(span, {true, fminf(y1, y2), fmaxf(y1, y2)});
        }
    }
    return span;
}

static YSpan intersect_cylinder(const CellRect &cell, Vec3 start, Vec3 end, Vec3 axis, float radius_sqr) {
    Vec3 start_point = {scalar::clamp(start.x, cell.x_min, cell.x_max), cell.y_base, scalar::clamp(start.z, cell.z_min, cell.z_max)};
    Vec3 end_point = {scalar::clamp(end.x, cell.x_min, cell.x_max), cell.y_base, scalar::clamp(end.z, cell.z_min, cell.z_max)};
    YSpan span = merge_intersections(ray_cylinder_intersection(start_point, start, axis, radius_sqr),
                                     ray_cylinder_intersection(end_point, start, axis, radius_sqr));

    float axis_length_2d_sqr = axis.x * axis.x + axis.z * axis.z;
    if (axis_length_2d_sqr > EPSILON) span = slabs_cylinder_intersection(cell, start, end, axis, radius_sqr, span);

    if (axis.y * axis.y > EPSILON) {
        Vec3 rectangle_on_start_plane[4];
        Vec3 rectangle_on_end_plane[4];
        float ds = vec3::dot(axis, start);
        float de = vec3::dot(axis, end);
        for (int i = 0; i < 4; i++) {
            float x = ((i + 1) & 2) ? cell.x_max : cell.x_min;
            float z = (i & 2) ? cell.z_max : cell.z_min;
            float dot_axis_a = axis.x * x + axis.y * cell.y_base + axis.z * z;
            float t = (ds - dot_axis_a) / axis.y;
            rectangle_on_start_plane[i] = {x, cell.y_base + t, z};
            t = (de - dot_axis_a) / axis.y;
            rectangle_on_end_plane[i] = {x, cell.y_base + t, z};
        }
        for (int i = 0; i < 4; i++) {
            span = cylinder_cap_intersection(start, radius_sqr, span, i, rectangle_on_start_plane);
            span = cylinder_cap_intersection(end, radius_sqr, span, i, rectangle_on_end_plane);
        }
    }
    return span;
}

static bool x_slab_segment_intersection(const CellRect &cell, float x, float y, float z, float dx, float dy, float dz, float slab_x,
                                        float &out_y) {
    float x2 = x + dx;
    if ((x < slab_x && x2 > slab_x) || (x > slab_x && x2 < slab_x)) {
        float t = (slab_x - x) / dx;
        float iz = z + dz * t;
        if (iz >= cell.z_min && iz <= cell.z_max) {
            out_y = y + dy * t;
            return true;
        }
    }
    return false;
}

static bool z_slab_segment_intersection(const CellRect &cell, float x, float y, float z, float dx, float dy, float dz, float slab_z,
                                        float &out_y) {
    float z2 = z + dz;
    if ((z < slab_z && z2 > slab_z) || (z > slab_z && z2 < slab_z)) {
        float t = (slab_z - z) / dz;
        float ix = x + dx * t;
        if (ix >= cell.x_min && ix <= cell.x_max) {
            out_y = y + dy * t;
            return true;
        }
    }
    return false;
}

static YSpan include_edge_slabs(YSpan span, const CellRect &cell, float x, float y, float z, float dx, float dy, float dz) {
    float iy;
    if (fabsf(dx) > EPSILON) {
        if (x_slab_segment_intersection(cell, x, y, z, dx, dy, dz, cell.x_min, iy)) span = include_height(span, iy);
        if (x_slab_segment_intersection(cell, x, y, z, dx, dy, dz, cell.x_max, iy)) span = include_height(span, iy);
    }
    if (fabsf(dz) > EPSILON) {
        if (z_slab_segment_intersection(cell, x, y, z, dx, dy, dz, cell.z_min, iy)) span = include_height(span, iy);
        if (z_slab_segment_intersection(cell, x, y, z, dx, dy, dz, cell.z_max, iy)) span = include_height(span, iy);
    }
    return span;
}

static YSpan intersect_box(const CellRect &cell, const float *vertices, const Plane *planes) {
    YSpan span = NO_Y_SPAN;
    for (int i = 0; i < 8; i++) {
        int vi = i * 3;
        if (vertices[vi] >= cell.x_min && vertices[vi] < cell.x_max && vertices[vi + 2] >= cell.z_min && vertices[vi + 2] < cell.z_max) {
            span = include_height(span, vertices[vi + 1]);
        }
    }

    for (int i = 0; i < 4; i++) {
        Vec3 point = {(i & 1) == 0 ? cell.x_min : cell.x_max, cell.y_base, (i & 2) == 0 ? cell.z_min : cell.z_max};
        for (int j = 0; j < 6; j++) {
            if (fabsf(planes[j].normal.y) <= EPSILON) continue;
            float t = (planes[j].distance - vec3::dot(planes[j].normal, point)) / planes[j].normal.y;
            float y = point.y + t;
            bool valid = true;
            for (int k = 0; k < 6; k++) {
                if (k == j) continue;
                if (point.x * planes[k].normal.x + y * planes[k].normal.y + point.z * planes[k].normal.z > planes[k].distance) {
                    valid = false;
                    break;
                }
            }
            if (valid) span = include_height(span, y);
        }
    }

    for (int i = 0; i < 24; i += 2) {
        int vi = BOX_EDGES[i] * 3;
        int vj = BOX_EDGES[i + 1] * 3;
        float x = vertices[vi];
        float y = vertices[vi + 1];
        float z = vertices[vi + 2];
        span = include_edge_slabs(span, cell, x, y, z, vertices[vj] - x, vertices[vj + 1] - y, vertices[vj + 2] - z);
    }

    return span;
}

struct TriangleBounds {
    float x_min;
    float z_min;
    float x_max;
    float z_max;
};

static bool ray_triangle_intersection(Vec3 point, uint32_t plane_index, const Plane *planes, float &out_y) {
    float t = (planes[plane_index].distance - vec3::dot(planes[plane_index].normal, point)) / planes[plane_index].normal.y;
    Vec3 s = {point.x, point.y + t, point.z};
    float u = vec3::dot(s, planes[plane_index + 1].normal) - planes[plane_index + 1].distance;
    if (u < 0.0f || u > 1.0f) return false;
    float v = vec3::dot(s, planes[plane_index + 2].normal) - planes[plane_index + 2].distance;
    if (v < 0.0f) return false;
    float w = 1 - u - v;
    if (w < 0.0f) return false;
    out_y = s.y;
    return true;
}

static YSpan intersect_convex(const CellRect &cell, const uint32_t *triangles, uint32_t triangle_index_count, const float *vertices,
                              const Plane *planes, const TriangleBounds *triangle_bounds) {
    YSpan span = NO_Y_SPAN;
    for (uint32_t tr = 0, tri = 0; tri < triangle_index_count; tr++, tri += 3) {
        const TriangleBounds &bounds = triangle_bounds[tr];
        if (bounds.x_min > cell.x_max || bounds.x_max < cell.x_min || bounds.z_min > cell.z_max || bounds.z_max < cell.z_min) continue;
        if (fabsf(planes[tri].normal.y) < EPSILON) continue;

        for (uint32_t i = 0; i < 3; i++) {
            uint32_t vi = triangles[tri + i] * 3;
            uint32_t vj = triangles[tri + (i + 1) % 3] * 3;
            float x = vertices[vi];
            float y = vertices[vi + 1];
            float z = vertices[vi + 2];
            if (x >= cell.x_min && x <= cell.x_max && z >= cell.z_min && z <= cell.z_max) span = include_height(span, y);
            span = include_edge_slabs(span, cell, x, y, z, vertices[vj] - x, vertices[vj + 1] - y, vertices[vj + 2] - z);
        }

        for (int i = 0; i < 4; i++) {
            Vec3 point = {(i & 1) == 0 ? cell.x_min : cell.x_max, cell.y_base, (i & 2) == 0 ? cell.z_min : cell.z_max};
            float y;
            if (ray_triangle_intersection(point, tri, planes, y)) span = include_height(span, y);
        }
    }
    return span;
}

void rasterize_sphere(Heightfield &heightfield, Vec3 center, float radius, uint8_t area, int flag_merge_threshold) {
    Box3 bounds = {vec3::subtract(center, {radius, radius, radius}), vec3::add(center, {radius, radius, radius})};
    float radius_sqr = radius * radius;
    rasterization_filled_shape(heightfield, bounds, area, flag_merge_threshold,
                               [&](const CellRect &cell) { return intersect_sphere(cell, center, radius_sqr); });
}

static Box3 segment_bounds(Vec3 start, Vec3 end, float radius) {
    Vec3 padding = {radius, radius, radius};
    return {vec3::subtract(vec3::min(start, end), padding), vec3::add(vec3::max(start, end), padding)};
}

void rasterize_capsule(Heightfield &heightfield, Vec3 start, Vec3 end, float radius, uint8_t area, int flag_merge_threshold) {
    Vec3 axis = vec3::subtract(end, start);
    float radius_sqr = radius * radius;
    rasterization_filled_shape(heightfield, segment_bounds(start, end, radius), area, flag_merge_threshold,
                               [&](const CellRect &cell) { return intersect_capsule(cell, start, end, axis, radius_sqr); });
}

void rasterize_cylinder(Heightfield &heightfield, Vec3 start, Vec3 end, float radius, uint8_t area, int flag_merge_threshold) {
    Vec3 axis = vec3::subtract(end, start);
    float radius_sqr = radius * radius;
    rasterization_filled_shape(heightfield, segment_bounds(start, end, radius), area, flag_merge_threshold,
                               [&](const CellRect &cell) { return intersect_cylinder(cell, start, end, axis, radius_sqr); });
}

void rasterize_box(Heightfield &heightfield, Vec3 center, const Vec3 half_edges[3], uint8_t area, int flag_merge_threshold) {
    Vec3 normals[3] = {vec3::normalize(half_edges[0]), vec3::normalize(half_edges[1]), vec3::normalize(half_edges[2])};

    float vertices[8 * 3];
    Box3 bounds = box3::empty();
    for (int i = 0; i < 8; ++i) {
        float s0 = (i & 1) != 0 ? 1.0f : -1.0f;
        float s1 = (i & 2) != 0 ? 1.0f : -1.0f;
        float s2 = (i & 4) != 0 ? 1.0f : -1.0f;
        Vec3 vertex = vec3::add(center, vec3::add(vec3::scale(half_edges[0], s0), vec3::add(vec3::scale(half_edges[1], s1), vec3::scale(half_edges[2], s2))));
        vec3::store(vertices, (uint32_t)i * 3, vertex);
        bounds = box3::expand_by_point(bounds, vertex);
    }

    Plane planes[6];
    for (int i = 0; i < 6; i++) {
        float m = i < 3 ? -1.0f : 1.0f;
        int vi = i < 3 ? 0 : 7;
        planes[i].normal = vec3::scale(normals[i % 3], m);
        planes[i].distance = vec3::dot(vec3::load(vertices, (uint32_t)vi * 3), planes[i].normal);
    }

    rasterization_filled_shape(heightfield, bounds, area, flag_merge_threshold,
                               [&](const CellRect &cell) { return intersect_box(cell, vertices, planes); });
}

static Plane plane_from_edges(Vec3 v1, Vec3 v2, Vec3 point) {
    Vec3 normal = vec3::cross(v1, v2);
    return {normal, vec3::dot(normal, point)};
}

static Plane scale_plane(Plane plane, float scale) { return {vec3::scale(plane.normal, scale), plane.distance * scale}; }

void rasterize_convex(Heightfield &heightfield, const InputTriangleMesh &mesh, uint8_t area, int flag_merge_threshold) {
    const float *vertices = mesh.positions.data;
    uint32_t vertex_float_count = mesh.positions.capacity;
    const uint32_t *triangles = mesh.indices.data;
    uint32_t triangle_index_count = mesh.indices.capacity;

    Box3 bounds = {vec3::load(vertices, 0), vec3::load(vertices, 0)};
    for (uint32_t i = 0; i < vertex_float_count; i += 3) bounds = box3::expand_by_point(bounds, vec3::load(vertices, i));

    uint32_t triangle_count = triangle_index_count / 3;
    Array<Plane> planes = {};
    Array<TriangleBounds> triangle_bounds = {};
    planes.resize(triangle_index_count);
    triangle_bounds.resize(triangle_count);

    for (uint32_t j = 0; j < triangle_index_count; j += 3) {
        Vec3 a = vec3::load(vertices, triangles[j] * 3);
        Vec3 b = vec3::load(vertices, triangles[j + 1] * 3);
        Vec3 c = vec3::load(vertices, triangles[j + 2] * 3);

        planes[j] = plane_from_edges(vec3::subtract(b, a), vec3::subtract(c, a), a);
        planes[j + 1] = plane_from_edges(planes[j].normal, vec3::subtract(c, b), b);
        planes[j + 2] = plane_from_edges(planes[j].normal, vec3::subtract(a, c), c);

        planes[j + 1] = scale_plane(planes[j + 1], 1.0f / (vec3::dot(a, planes[j + 1].normal) - planes[j + 1].distance));
        planes[j + 2] = scale_plane(planes[j + 2], 1.0f / (vec3::dot(b, planes[j + 2].normal) - planes[j + 2].distance));

        triangle_bounds[j / 3] = {fminf(a.x, fminf(b.x, c.x)), fminf(a.z, fminf(b.z, c.z)), fmaxf(a.x, fmaxf(b.x, c.x)),
                                  fmaxf(a.z, fmaxf(b.z, c.z))};
    }

    rasterization_filled_shape(heightfield, bounds, area, flag_merge_threshold, [&](const CellRect &cell) {
        return intersect_convex(cell, triangles, triangle_index_count, vertices, planes.data, triangle_bounds.data);
    });

    planes.release();
    triangle_bounds.release();
}

}

}
