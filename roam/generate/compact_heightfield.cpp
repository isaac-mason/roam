#include "roam/generate/compact_heightfield.h"

#include <math.h>

#include "roam/generate/common.h"
#include "roam/geometry/geometry.h"
#include "roam/math/scalar.h"
#include "roam/util/sort.h"

namespace roam {

namespace compact_heightfield {

static uint32_t get_heightfield_span_count(const Heightfield &heightfield) {
    uint32_t span_count = 0;
    for (int32_t first : heightfield.spans) {
        for (int32_t index = first; index != NULL_SPAN; index = heightfield.span_pool[(uint32_t)index].next) {
            if (heightfield.span_pool[(uint32_t)index].area != NULL_AREA) span_count++;
        }
    }
    return span_count;
}

CompactHeightfield build(BuildContext &context, const Heightfield &heightfield, int walkable_height_voxels, int walkable_climb_voxels) {
    int x_size = heightfield.width;
    int z_size = heightfield.height;
    uint32_t span_count = get_heightfield_span_count(heightfield);

    CompactHeightfield compact_heightfield = {};
    compact_heightfield.width = x_size;
    compact_heightfield.height = z_size;
    compact_heightfield.span_count = span_count;
    compact_heightfield.walkable_height_voxels = walkable_height_voxels;
    compact_heightfield.walkable_climb_voxels = walkable_climb_voxels;
    compact_heightfield.bounds = heightfield.bounds;
    compact_heightfield.cell_size = heightfield.cell_size;
    compact_heightfield.cell_height = heightfield.cell_height;
    compact_heightfield.cells.resize((uint32_t)(x_size * z_size), {0, 0});
    compact_heightfield.spans.resize(span_count, {0, 0, 0, 0});
    compact_heightfield.areas.resize(span_count, NULL_AREA);
    compact_heightfield.distances.resize(span_count, 0);

    compact_heightfield.bounds.max.y += (float)walkable_height_voxels * heightfield.cell_height;

    uint32_t current_cell_index = 0;
    uint32_t column_count = (uint32_t)(x_size * z_size);

    for (uint32_t column_index = 0; column_index < column_count; ++column_index) {
        int32_t index = heightfield.spans[column_index];
        if (index == NULL_SPAN) continue;

        CompactHeightfieldCell &cell = compact_heightfield.cells[column_index];
        cell.index = current_cell_index;
        cell.count = 0;

        while (index != NULL_SPAN) {
            const HeightfieldSpan &span = heightfield.span_pool[(uint32_t)index];
            if (span.area != NULL_AREA) {
                int bottom = span.max;
                int top = span.next != NULL_SPAN ? heightfield.span_pool[(uint32_t)span.next].min : MAX_HEIGHT;

                compact_heightfield.spans[current_cell_index].y = scalar::min(scalar::max(bottom, 0), 0xffff);
                compact_heightfield.spans[current_cell_index].h = scalar::min(scalar::max(top - bottom, 0), 0xff);
                compact_heightfield.areas[current_cell_index] = span.area;

                current_cell_index++;
                cell.count++;
            }
            index = span.next;
        }
    }

    int max_layer_index = 0;
    int z_stride = x_size;

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * z_stride)];

            for (uint32_t i = cell.index; i < cell.index + cell.count; ++i) {
                CompactHeightfieldSpan &span = compact_heightfield.spans[i];

                for (int direction = 0; direction < 4; ++direction) {
                    set_con(span, direction, NOT_CONNECTED);

                    int neighbor_x = x + DIR_OFFSETS[direction][0];
                    int neighbor_z = z + DIR_OFFSETS[direction][1];

                    if (neighbor_x < 0 || neighbor_z < 0 || neighbor_x >= x_size || neighbor_z >= z_size) continue;

                    CompactHeightfieldCell neighbor_cell = compact_heightfield.cells[(uint32_t)(neighbor_x + neighbor_z * z_stride)];

                    for (uint32_t k = neighbor_cell.index; k < neighbor_cell.index + neighbor_cell.count; ++k) {
                        const CompactHeightfieldSpan &neighbor_span = compact_heightfield.spans[k];
                        int bottom = scalar::max(span.y, neighbor_span.y);
                        int top = scalar::min(span.y + span.h, neighbor_span.y + neighbor_span.h);

                        if (top - bottom >= walkable_height_voxels && scalar::abs(neighbor_span.y - span.y) <= walkable_climb_voxels) {
                            int layer_index = (int)(k - neighbor_cell.index);
                            if (layer_index < 0 || layer_index > MAX_LAYERS) {
                                max_layer_index = scalar::max(max_layer_index, layer_index);
                                continue;
                            }
                            set_con(span, direction, layer_index);
                            break;
                        }
                    }
                }
            }
        }
    }

    if (max_layer_index > MAX_LAYERS) {
        build_context::warn(context, "buildCompactHeightfield: Heightfield has too many layers %d (max: %d)", max_layer_index, MAX_LAYERS);
    }

    return compact_heightfield;
}

void release(CompactHeightfield &compact_heightfield) {
    compact_heightfield.cells.release();
    compact_heightfield.spans.release();
    compact_heightfield.areas.release();
    compact_heightfield.distances.release();
}

static const uint8_t MAX_DISTANCE = 255;

static uint32_t neighbor_span_index(const CompactHeightfield &compact_heightfield, int x, int z, int direction, int connection) {
    int neighbor_x = x + DIR_OFFSETS[direction][0];
    int neighbor_z = z + DIR_OFFSETS[direction][1];
    return compact_heightfield.cells[(uint32_t)(neighbor_x + neighbor_z * compact_heightfield.width)].index + (uint32_t)connection;
}

static void relax_distance(uint8_t *distance_to_boundary, uint32_t span_index, uint32_t neighbor_index, int cost) {
    int new_distance = scalar::min(distance_to_boundary[neighbor_index] + cost, 255);
    if (new_distance < distance_to_boundary[span_index]) distance_to_boundary[span_index] = (uint8_t)new_distance;
}

static void relax_from_direction(const CompactHeightfield &compact_heightfield, uint8_t *distance_to_boundary, int x, int z,
                                 uint32_t span_index, int direction, int diagonal_direction) {
    const CompactHeightfieldSpan &span = compact_heightfield.spans[span_index];
    if (get_con(span, direction) == NOT_CONNECTED) return;

    int ax = x + DIR_OFFSETS[direction][0];
    int az = z + DIR_OFFSETS[direction][1];
    uint32_t a_index = neighbor_span_index(compact_heightfield, x, z, direction, get_con(span, direction));
    relax_distance(distance_to_boundary, span_index, a_index, 2);

    const CompactHeightfieldSpan &a_span = compact_heightfield.spans[a_index];
    if (get_con(a_span, diagonal_direction) == NOT_CONNECTED) return;

    uint32_t b_index = neighbor_span_index(compact_heightfield, ax, az, diagonal_direction, get_con(a_span, diagonal_direction));
    relax_distance(distance_to_boundary, span_index, b_index, 3);
}

static Array<uint8_t> compute_distance_to_boundary(const CompactHeightfield &compact_heightfield) {
    int x_size = compact_heightfield.width;
    int z_size = compact_heightfield.height;

    Array<uint8_t> distance_to_boundary = {};
    distance_to_boundary.resize(compact_heightfield.span_count, MAX_DISTANCE);

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * x_size)];
            for (uint32_t span_index = cell.index; span_index < cell.index + cell.count; ++span_index) {
                if (compact_heightfield.areas[span_index] == NULL_AREA) {
                    distance_to_boundary[span_index] = 0;
                    continue;
                }

                const CompactHeightfieldSpan &span = compact_heightfield.spans[span_index];

                int neighbor_count = 0;
                for (int direction = 0; direction < 4; ++direction) {
                    int neighbor_connection = get_con(span, direction);
                    if (neighbor_connection == NOT_CONNECTED) break;

                    uint32_t neighbor_index = neighbor_span_index(compact_heightfield, x, z, direction, neighbor_connection);
                    if (compact_heightfield.areas[neighbor_index] == NULL_AREA) break;
                    neighbor_count++;
                }

                if (neighbor_count != 4) distance_to_boundary[span_index] = 0;
            }
        }
    }

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * x_size)];
            for (uint32_t span_index = cell.index; span_index < cell.index + cell.count; ++span_index) {
                relax_from_direction(compact_heightfield, distance_to_boundary.data, x, z, span_index, 0, 3);
                relax_from_direction(compact_heightfield, distance_to_boundary.data, x, z, span_index, 3, 2);
            }
        }
    }

    for (int z = z_size - 1; z >= 0; --z) {
        for (int x = x_size - 1; x >= 0; --x) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * x_size)];
            for (uint32_t span_index = cell.index; span_index < cell.index + cell.count; ++span_index) {
                relax_from_direction(compact_heightfield, distance_to_boundary.data, x, z, span_index, 2, 1);
                relax_from_direction(compact_heightfield, distance_to_boundary.data, x, z, span_index, 1, 0);
            }
        }
    }

    return distance_to_boundary;
}

void erode_walkable_area(CompactHeightfield &compact_heightfield, int walkable_radius_voxels) {
    Array<uint8_t> distance_to_boundary = compute_distance_to_boundary(compact_heightfield);

    int min_boundary_distance = walkable_radius_voxels * 2;
    for (uint32_t span_index = 0; span_index < compact_heightfield.span_count; ++span_index) {
        if (distance_to_boundary[span_index] < min_boundary_distance) compact_heightfield.areas[span_index] = NULL_AREA;
    }

    distance_to_boundary.release();
}

void erode_and_mark_walkable_areas(CompactHeightfield &compact_heightfield, int base_walkable_radius_voxels,
                                   Span<const WalkableAreaThreshold> thresholds) {
    Array<uint8_t> distance_to_boundary = compute_distance_to_boundary(compact_heightfield);

    Array<WalkableAreaThreshold> sorted_thresholds = {};
    for (const WalkableAreaThreshold &threshold : thresholds) sorted_thresholds.push(threshold);
    sort::stable(sorted_thresholds.data, sorted_thresholds.length, [](const WalkableAreaThreshold &a, const WalkableAreaThreshold &b) {
        return a.walkable_radius_voxels < b.walkable_radius_voxels;
    });

    int base_min_distance = base_walkable_radius_voxels * 2;

    for (uint32_t span_index = 0; span_index < compact_heightfield.span_count; ++span_index) {
        int distance = distance_to_boundary[span_index];

        if (distance < base_min_distance) {
            compact_heightfield.areas[span_index] = NULL_AREA;
            continue;
        }

        for (const WalkableAreaThreshold &threshold : sorted_thresholds) {
            if (distance < threshold.walkable_radius_voxels * 2) {
                compact_heightfield.areas[span_index] = threshold.area_id;
                break;
            }
        }
    }

    sorted_thresholds.release();
    distance_to_boundary.release();
}

struct GridFootprint {
    bool inside;
    int min_x;
    int min_y;
    int min_z;
    int max_x;
    int max_y;
    int max_z;
};

static GridFootprint grid_footprint(const CompactHeightfield &compact_heightfield, Vec3 min, Vec3 max) {
    GridFootprint footprint;
    footprint.min_x = (int)floorf((min.x - compact_heightfield.bounds.min.x) / compact_heightfield.cell_size);
    footprint.min_y = (int)floorf((min.y - compact_heightfield.bounds.min.y) / compact_heightfield.cell_height);
    footprint.min_z = (int)floorf((min.z - compact_heightfield.bounds.min.z) / compact_heightfield.cell_size);
    footprint.max_x = (int)floorf((max.x - compact_heightfield.bounds.min.x) / compact_heightfield.cell_size);
    footprint.max_y = (int)floorf((max.y - compact_heightfield.bounds.min.y) / compact_heightfield.cell_height);
    footprint.max_z = (int)floorf((max.z - compact_heightfield.bounds.min.z) / compact_heightfield.cell_size);

    footprint.inside = !(footprint.max_x < 0 || footprint.min_x >= compact_heightfield.width || footprint.max_z < 0 ||
                         footprint.min_z >= compact_heightfield.height);

    if (footprint.min_x < 0) footprint.min_x = 0;
    if (footprint.max_x >= compact_heightfield.width) footprint.max_x = compact_heightfield.width - 1;
    if (footprint.min_z < 0) footprint.min_z = 0;
    if (footprint.max_z >= compact_heightfield.height) footprint.max_z = compact_heightfield.height - 1;

    return footprint;
}

static Vec3 cell_center(const CompactHeightfield &compact_heightfield, int x, int z) {
    return {compact_heightfield.bounds.min.x + ((float)x + 0.5f) * compact_heightfield.cell_size, 0,
            compact_heightfield.bounds.min.z + ((float)z + 0.5f) * compact_heightfield.cell_size};
}

static void mark_column(CompactHeightfield &compact_heightfield, int x, int z, int min_y, int max_y, uint8_t area_id) {
    CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * compact_heightfield.width)];
    for (uint32_t span_index = cell.index; span_index < cell.index + cell.count; ++span_index) {
        int y = compact_heightfield.spans[span_index].y;
        if (y < min_y || y > max_y) continue;
        if (compact_heightfield.areas[span_index] == NULL_AREA) continue;
        compact_heightfield.areas[span_index] = area_id;
    }
}

void mark_box_area(CompactHeightfield &compact_heightfield, Box3 bounds, uint8_t area_id) {
    GridFootprint footprint = grid_footprint(compact_heightfield, bounds.min, bounds.max);
    if (!footprint.inside) return;

    for (int z = footprint.min_z; z <= footprint.max_z; ++z) {
        for (int x = footprint.min_x; x <= footprint.max_x; ++x) mark_column(compact_heightfield, x, z, footprint.min_y, footprint.max_y, area_id);
    }
}

void mark_rotated_box_area(CompactHeightfield &compact_heightfield, Vec3 center, Vec3 half_extents, float angle_radians, uint8_t area_id) {
    float cos_angle = cosf(angle_radians);
    float sin_angle = sinf(angle_radians);

    float hx = half_extents.x;
    float hz = half_extents.z;
    const float corners[4][2] = {{-hx, -hz}, {hx, -hz}, {hx, hz}, {-hx, hz}};

    Vec3 min = {INFINITY, center.y - half_extents.y, INFINITY};
    Vec3 max = {-INFINITY, center.y + half_extents.y, -INFINITY};
    for (int i = 0; i < 4; i++) {
        float world_x = center.x + cos_angle * corners[i][0] - sin_angle * corners[i][1];
        float world_z = center.z + sin_angle * corners[i][0] + cos_angle * corners[i][1];
        min.x = fminf(min.x, world_x);
        max.x = fmaxf(max.x, world_x);
        min.z = fminf(min.z, world_z);
        max.z = fmaxf(max.z, world_z);
    }

    GridFootprint footprint = grid_footprint(compact_heightfield, min, max);
    if (!footprint.inside) return;

    for (int z = footprint.min_z; z <= footprint.max_z; ++z) {
        for (int x = footprint.min_x; x <= footprint.max_x; ++x) {
            Vec3 cell = cell_center(compact_heightfield, x, z);
            float dx = cell.x - center.x;
            float dz = cell.z - center.z;

            float local_x = cos_angle * dx - sin_angle * dz;
            float local_z = sin_angle * dx + cos_angle * dz;

            if (fabsf(local_x) > half_extents.x || fabsf(local_z) > half_extents.z) continue;

            mark_column(compact_heightfield, x, z, footprint.min_y, footprint.max_y, area_id);
        }
    }
}

void mark_convex_poly_area(CompactHeightfield &compact_heightfield, Span<const float> polygon, float min_y, float max_y, uint8_t area_id) {
    const float *vertices = polygon.data;
    uint32_t vertex_count = polygon.capacity / 3;
    Vec3 min = {vertices[0], min_y, vertices[2]};
    Vec3 max = {vertices[0], max_y, vertices[2]};
    for (uint32_t i = 1; i < vertex_count; ++i) {
        min.x = fminf(min.x, vertices[i * 3]);
        min.z = fminf(min.z, vertices[i * 3 + 2]);
        max.x = fmaxf(max.x, vertices[i * 3]);
        max.z = fmaxf(max.z, vertices[i * 3 + 2]);
    }

    GridFootprint footprint = grid_footprint(compact_heightfield, min, max);
    if (!footprint.inside) return;

    for (int z = footprint.min_z; z <= footprint.max_z; ++z) {
        for (int x = footprint.min_x; x <= footprint.max_x; ++x) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * compact_heightfield.width)];
            for (uint32_t span_index = cell.index; span_index < cell.index + cell.count; ++span_index) {
                if (compact_heightfield.areas[span_index] == NULL_AREA) continue;

                int y = compact_heightfield.spans[span_index].y;
                if (y < footprint.min_y || y > footprint.max_y) continue;

                if (geometry::point_in_poly(cell_center(compact_heightfield, x, z), vertices, (int)vertex_count)) {
                    compact_heightfield.areas[span_index] = area_id;
                }
            }
        }
    }
}

void mark_cylinder_area(CompactHeightfield &compact_heightfield, Vec3 position, float radius, float height, uint8_t area_id) {
    Vec3 min = {position.x - radius, position.y, position.z - radius};
    Vec3 max = {position.x + radius, position.y + height, position.z + radius};

    GridFootprint footprint = grid_footprint(compact_heightfield, min, max);
    if (!footprint.inside) return;

    float radius_sqr = radius * radius;

    for (int z = footprint.min_z; z <= footprint.max_z; ++z) {
        for (int x = footprint.min_x; x <= footprint.max_x; ++x) {
            Vec3 cell = cell_center(compact_heightfield, x, z);
            float delta_x = cell.x - position.x;
            float delta_z = cell.z - position.z;

            if (delta_x * delta_x + delta_z * delta_z >= radius_sqr) continue;

            mark_column(compact_heightfield, x, z, footprint.min_y, footprint.max_y, area_id);
        }
    }
}

static void insert_sort(uint8_t *values, int length) {
    for (int i = 1; i < length; ++i) {
        uint8_t key = values[i];
        int j = i - 1;
        while (j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            j--;
        }
        values[j + 1] = key;
    }
}

bool median_filter_walkable_area(CompactHeightfield &compact_heightfield) {
    int x_size = compact_heightfield.width;
    int z_size = compact_heightfield.height;

    Array<uint8_t> areas = {};
    areas.resize(compact_heightfield.span_count, 0xff);

    for (int z = 0; z < z_size; ++z) {
        for (int x = 0; x < x_size; ++x) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + z * x_size)];

            for (uint32_t span_index = cell.index; span_index < cell.index + cell.count; ++span_index) {
                const CompactHeightfieldSpan &span = compact_heightfield.spans[span_index];

                if (compact_heightfield.areas[span_index] == NULL_AREA) {
                    areas[span_index] = compact_heightfield.areas[span_index];
                    continue;
                }

                uint8_t neighbor_areas[9];
                for (int neighbor_index = 0; neighbor_index < 9; ++neighbor_index) neighbor_areas[neighbor_index] = compact_heightfield.areas[span_index];

                for (int direction = 0; direction < 4; ++direction) {
                    if (get_con(span, direction) == NOT_CONNECTED) continue;

                    int ax = x + DIR_OFFSETS[direction][0];
                    int az = z + DIR_OFFSETS[direction][1];
                    uint32_t a_index = neighbor_span_index(compact_heightfield, x, z, direction, get_con(span, direction));

                    if (compact_heightfield.areas[a_index] != NULL_AREA) neighbor_areas[direction * 2 + 0] = compact_heightfield.areas[a_index];

                    const CompactHeightfieldSpan &a_span = compact_heightfield.spans[a_index];
                    int direction_2 = (direction + 1) & 0x3;
                    int neighbor_connection_2 = get_con(a_span, direction_2);

                    if (neighbor_connection_2 != NOT_CONNECTED) {
                        uint32_t b_index = neighbor_span_index(compact_heightfield, ax, az, direction_2, neighbor_connection_2);
                        if (compact_heightfield.areas[b_index] != NULL_AREA) neighbor_areas[direction * 2 + 1] = compact_heightfield.areas[b_index];
                    }
                }

                insert_sort(neighbor_areas, 9);
                areas[span_index] = neighbor_areas[4];
            }
        }
    }

    for (uint32_t i = 0; i < compact_heightfield.span_count; ++i) compact_heightfield.areas[i] = areas[i];

    areas.release();
    return true;
}

}

}
