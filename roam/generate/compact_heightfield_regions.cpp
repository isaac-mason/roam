#include "roam/generate/compact_heightfield_regions.h"

#include <stddef.h>
#include <string.h>

#include "roam/generate/common.h"
#include "roam/math/scalar.h"

namespace roam {

namespace compact_heightfield_regions {

static const int LOG_NB_STACKS = 3;
static const int NB_STACKS = 1 << LOG_NB_STACKS;
static const int EXPAND_ITERS = 8;
static const int NULL_NEI = 0xffff;

using compact_heightfield::get_con;

static uint32_t neighbor_index(const CompactHeightfield &compact_heightfield, int x, int y, int direction, const CompactHeightfieldSpan &span) {
    int ax = x + common::DIR_OFFSETS[direction][0];
    int ay = y + common::DIR_OFFSETS[direction][1];
    return compact_heightfield.cells[(uint32_t)(ax + ay * compact_heightfield.width)].index + (uint32_t)get_con(span, direction);
}

static size_t neighbor_span_index(const CompactHeightfield &compact_heightfield, int x, int y, int direction, const CompactHeightfieldSpan &span) {
    size_t cell = (size_t)(x + common::DIR_OFFSETS[direction][0]) + (size_t)(y + common::DIR_OFFSETS[direction][1]) * (size_t)compact_heightfield.width;
    return (size_t)compact_heightfield.cells.data[cell].index + (size_t)get_con(span, direction);
}

static void relax_distance(const CompactHeightfield &compact_heightfield, uint16_t *distances, int x, int y, size_t i, int direction,
                           int diagonal_direction) {
    const CompactHeightfieldSpan &span = compact_heightfield.spans.data[i];
    if (get_con(span, direction) == NOT_CONNECTED) return;

    size_t ai = neighbor_span_index(compact_heightfield, x, y, direction, span);
    if (distances[ai] + 2 < distances[i]) distances[i] = (uint16_t)(distances[ai] + 2);

    const CompactHeightfieldSpan &a_span = compact_heightfield.spans.data[ai];
    if (get_con(a_span, diagonal_direction) == NOT_CONNECTED) return;

    size_t aai = neighbor_span_index(compact_heightfield, x + common::DIR_OFFSETS[direction][0], y + common::DIR_OFFSETS[direction][1],
                                     diagonal_direction, a_span);
    if (distances[aai] + 3 < distances[i]) distances[i] = (uint16_t)(distances[aai] + 3);
}

static int calculate_distance_field(const CompactHeightfield &compact_heightfield, uint16_t *distances) {
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;
    const CompactHeightfieldCell *cells = compact_heightfield.cells.data;
    const CompactHeightfieldSpan *spans = compact_heightfield.spans.data;
    const uint8_t *areas = compact_heightfield.areas.data;
    size_t span_count = compact_heightfield.spans.length;

    for (size_t i = 0; i < span_count; i++) distances[i] = 0xffff;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            CompactHeightfieldCell cell = cells[(size_t)x + (size_t)y * (size_t)w];
            for (size_t i = cell.index, end = (size_t)cell.index + cell.count; i < end; ++i) {
                const CompactHeightfieldSpan &span = spans[i];
                uint8_t area = areas[i];

                int neighbor_count = 0;
                for (int direction = 0; direction < 4; ++direction) {
                    if (get_con(span, direction) == NOT_CONNECTED) continue;
                    if (area == areas[neighbor_span_index(compact_heightfield, x, y, direction, span)]) neighbor_count++;
                }
                if (neighbor_count != 4) distances[i] = 0;
            }
        }
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            CompactHeightfieldCell cell = cells[(size_t)x + (size_t)y * (size_t)w];
            for (size_t i = cell.index, end = (size_t)cell.index + cell.count; i < end; ++i) {
                relax_distance(compact_heightfield, distances, x, y, i, 0, 3);
                relax_distance(compact_heightfield, distances, x, y, i, 3, 2);
            }
        }
    }

    for (int y = h - 1; y >= 0; --y) {
        for (int x = w - 1; x >= 0; --x) {
            CompactHeightfieldCell cell = cells[(size_t)x + (size_t)y * (size_t)w];
            for (size_t i = cell.index, end = (size_t)cell.index + cell.count; i < end; ++i) {
                relax_distance(compact_heightfield, distances, x, y, i, 2, 1);
                relax_distance(compact_heightfield, distances, x, y, i, 1, 0);
            }
        }
    }

    int max_distance = 0;
    for (size_t i = 0; i < span_count; ++i) max_distance = scalar::max((int)distances[i], max_distance);

    return max_distance;
}

static void box_blur(const CompactHeightfield &compact_heightfield, int threshold, const uint16_t *source_distances, uint16_t *destination_distances) {
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;
    const CompactHeightfieldCell *cells = compact_heightfield.cells.data;
    const CompactHeightfieldSpan *spans = compact_heightfield.spans.data;

    int scaled_threshold = threshold * 2;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            CompactHeightfieldCell cell = cells[(size_t)x + (size_t)y * (size_t)w];
            for (size_t i = cell.index, end = (size_t)cell.index + cell.count; i < end; ++i) {
                const CompactHeightfieldSpan &span = spans[i];
                int cd = source_distances[i];

                if (cd <= scaled_threshold) {
                    destination_distances[i] = (uint16_t)cd;
                    continue;
                }

                int d = cd;
                for (int direction = 0; direction < 4; ++direction) {
                    if (get_con(span, direction) == NOT_CONNECTED) {
                        d += cd * 2;
                        continue;
                    }

                    size_t ai = neighbor_span_index(compact_heightfield, x, y, direction, span);
                    d += source_distances[ai];

                    const CompactHeightfieldSpan &a_span = spans[ai];
                    int direction_2 = (direction + 1) & 0x3;
                    if (get_con(a_span, direction_2) != NOT_CONNECTED) {
                        d += source_distances[neighbor_span_index(compact_heightfield, x + common::DIR_OFFSETS[direction][0],
                                                                  y + common::DIR_OFFSETS[direction][1],
                                                                direction_2, a_span)];
                    } else {
                        d += cd;
                    }
                }
                destination_distances[i] = (uint16_t)((d + 5) / 9);
            }
        }
    }
}

void build_distance_field(CompactHeightfield &compact_heightfield) {
    Array<uint16_t> blurred = {};
    blurred.resize(compact_heightfield.spans.length);

    compact_heightfield.max_distance = calculate_distance_field(compact_heightfield, compact_heightfield.distances.data);

    box_blur(compact_heightfield, 1, compact_heightfield.distances.data, blurred.data);

    Array<uint16_t> unblurred = compact_heightfield.distances;
    compact_heightfield.distances = blurred;
    unblurred.release();
}

struct LevelStackEntry {
    int x;
    int y;
    int index;
};

using LevelStack = Array<LevelStackEntry>;

static void paint_rect_region(int min_x, int max_x, int min_y, int max_y, int region_id, const CompactHeightfield &compact_heightfield,
                              uint16_t *source_regions) {
    int w = compact_heightfield.width;
    for (int y = min_y; y < max_y; y++) {
        for (int x = min_x; x < max_x; x++) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + y * w)];
            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                if (compact_heightfield.areas[i] != NULL_AREA) source_regions[i] = (uint16_t)region_id;
            }
        }
    }
}

static int paint_border_regions(int border_size, int region_id, const CompactHeightfield &compact_heightfield, uint16_t *source_regions) {
    if (border_size <= 0) return region_id;

    int w = compact_heightfield.width;
    int h = compact_heightfield.height;
    int bw = scalar::min(w, border_size);
    int bh = scalar::min(h, border_size);

    paint_rect_region(0, bw, 0, h, region_id | BORDER_REG, compact_heightfield, source_regions);
    region_id++;
    paint_rect_region(w - bw, w, 0, h, region_id | BORDER_REG, compact_heightfield, source_regions);
    region_id++;
    paint_rect_region(0, w, 0, bh, region_id | BORDER_REG, compact_heightfield, source_regions);
    region_id++;
    paint_rect_region(0, w, h - bh, h, region_id | BORDER_REG, compact_heightfield, source_regions);
    region_id++;

    return region_id;
}

static void gather_unassigned_spans(const CompactHeightfield &compact_heightfield, const uint16_t *source_regions, LevelStack &unassigned) {
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;
    unassigned.clear();
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + y * w)];
            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                if (compact_heightfield.areas[i] == NULL_AREA || source_regions[i] != 0) continue;
                unassigned.push({x, y, (int)i});
            }
        }
    }
}

static void sort_cells_by_level(int start_level, const CompactHeightfield &compact_heightfield, const uint16_t *source_regions, int stack_count,
                                LevelStack *stacks, int log_levels_per_stack, LevelStack &unassigned) {
    const uint16_t *distances = compact_heightfield.distances.data;
    int adjusted_start_level = start_level >> log_levels_per_stack;

    for (int j = 0; j < stack_count; j++) stacks[j].clear();

    uint32_t kept = 0;
    for (uint32_t k = 0; k < unassigned.length; k++) {
        LevelStackEntry entry = unassigned[k];
        if (source_regions[entry.index] != 0) continue;
        unassigned[kept++] = entry;

        int level = distances[entry.index] >> log_levels_per_stack;
        int stack_id = adjusted_start_level - level;
        if (stack_id >= stack_count) continue;
        if (stack_id < 0) stack_id = 0;

        stacks[stack_id].push(entry);
    }
    unassigned.resize(kept);
}

static void append_stacks(const LevelStack &source_stack, LevelStack &destination_stack, const uint16_t *source_regions) {
    for (const LevelStackEntry &entry : source_stack) {
        if (entry.index < 0 || source_regions[entry.index] != 0) continue;
        destination_stack.push(entry);
    }
}

static bool flood_region(int x, int y, int i, int level, int r, const CompactHeightfield &compact_heightfield, uint16_t *source_regions,
                         uint16_t *source_distances, LevelStack &stack) {
    const CompactHeightfieldSpan *spans = compact_heightfield.spans.data;
    const uint8_t *areas = compact_heightfield.areas.data;
    const uint16_t *distances = compact_heightfield.distances.data;
    uint8_t area = areas[(size_t)i];

    stack.clear();
    stack.push({x, y, i});
    source_regions[i] = (uint16_t)r;
    source_distances[i] = 0;

    int lev = level >= 2 ? level - 2 : 0;
    int count = 0;

    while (stack.length > 0) {
        LevelStackEntry current = stack.pop();
        int cx = current.x;
        int cy = current.y;
        int ci = current.index;

        const CompactHeightfieldSpan &span = spans[(size_t)ci];

        int ar = 0;
        for (int direction = 0; direction < 4; direction++) {
            if (get_con(span, direction) == NOT_CONNECTED) continue;

            int ax = cx + common::DIR_OFFSETS[direction][0];
            int ay = cy + common::DIR_OFFSETS[direction][1];
            size_t ai = neighbor_span_index(compact_heightfield, cx, cy, direction, span);

            if (areas[ai] != area) continue;

            int nr = source_regions[ai];
            if (nr & BORDER_REG) continue;
            if (nr != 0 && nr != r) {
                ar = nr;
                break;
            }

            const CompactHeightfieldSpan &a_span = spans[ai];
            int direction_2 = (direction + 1) & 0x3;
            if (get_con(a_span, direction_2) != NOT_CONNECTED) {
                size_t ai2 = neighbor_span_index(compact_heightfield, ax, ay, direction_2, a_span);

                if (areas[ai2] != area) continue;

                int nr2 = source_regions[ai2];
                if (nr2 != 0 && nr2 != r) {
                    ar = nr2;
                    break;
                }
            }
        }

        if (ar != 0) {
            source_regions[ci] = 0;
            continue;
        }

        count++;

        for (int direction = 0; direction < 4; direction++) {
            if (get_con(span, direction) == NOT_CONNECTED) continue;

            int ax = cx + common::DIR_OFFSETS[direction][0];
            int ay = cy + common::DIR_OFFSETS[direction][1];
            size_t ai = neighbor_span_index(compact_heightfield, cx, cy, direction, span);

            if (areas[ai] != area) continue;
            if (distances[ai] >= lev && source_regions[ai] == 0) {
                source_regions[ai] = (uint16_t)r;
                source_distances[ai] = 0;
                stack.push({ax, ay, (int)ai});
            }
        }
    }

    return count > 0;
}

struct DirtyEntry {
    int index;
    int region;
    int distance_2;
};

static void expand_regions(int max_iterations, int level, const CompactHeightfield &compact_heightfield, uint16_t *source_regions,
                           uint16_t *source_distances, LevelStack &stack, bool fill_stack, Array<DirtyEntry> &dirty_entries) {
    const CompactHeightfieldCell *cells = compact_heightfield.cells.data;
    const CompactHeightfieldSpan *spans = compact_heightfield.spans.data;
    const uint8_t *areas = compact_heightfield.areas.data;
    const uint16_t *distances = compact_heightfield.distances.data;
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;

    if (fill_stack) {
        stack.clear();
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactHeightfieldCell cell = cells[(size_t)x + (size_t)y * (size_t)w];
                for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                    if (distances[i] >= level && source_regions[i] == 0 && areas[i] != NULL_AREA) {
                        stack.push({x, y, (int)i});
                    }
                }
            }
        }
    } else {
        for (LevelStackEntry &entry : stack) {
            if (source_regions[entry.index] != 0) entry.index = -1;
        }
    }

    dirty_entries.clear();
    int iteration = 0;

    while (stack.length > 0) {
        uint32_t kept = 0;
        uint32_t resolved = 0;
        dirty_entries.clear();

        for (uint32_t k = 0; k < stack.length; k++) {
            LevelStackEntry entry = stack[k];
            int x = entry.x;
            int y = entry.y;
            int i = entry.index;

            if (i < 0) continue;

            int r = source_regions[i];
            int d2 = 0xffff;
            uint8_t area = areas[(size_t)i];
            const CompactHeightfieldSpan &span = spans[(size_t)i];

            for (int direction = 0; direction < 4; direction++) {
                if (get_con(span, direction) == NOT_CONNECTED) continue;

                size_t ai = neighbor_span_index(compact_heightfield, x, y, direction, span);

                if (areas[ai] != area) continue;

                if (source_regions[ai] > 0 && (source_regions[ai] & BORDER_REG) == 0) {
                    if (source_distances[ai] + 2 < d2) {
                        r = source_regions[ai];
                        d2 = source_distances[ai] + 2;
                    }
                }
            }

            if (r) {
                resolved++;
                dirty_entries.push({i, r, d2});
            } else {
                stack[kept++] = entry;
            }
        }
        stack.resize(kept);

        for (const DirtyEntry &entry : dirty_entries) {
            source_regions[entry.index] = (uint16_t)entry.region;
            source_distances[entry.index] = (uint16_t)entry.distance_2;
        }

        if (resolved == 0) break;

        if (level > 0) {
            iteration++;
            if (iteration >= max_iterations) break;
        }
    }
}

struct Region {
    int span_count;
    int id;
    uint8_t area_type;
    bool remap;
    bool visited;
    bool overlap;
    bool connects_to_border;
    int ymin;
    int ymax;
    Array<int> connections;
    Array<int> floors;
};

static Array<Region> create_regions(int region_count) {
    Array<Region> regions = {};
    for (int i = 0; i < region_count; i++) {
        Region region = {};
        region.id = i;
        region.ymin = 0xffff;
        regions.push(region);
    }
    return regions;
}

static void release_regions(Array<Region> &regions) {
    for (Region &region : regions) {
        region.connections.release();
        region.floors.release();
    }
    regions.release();
}

static void add_unique_floor_region(Region &region, int n) {
    for (int floor : region.floors) {
        if (floor == n) return;
    }
    region.floors.push(n);
}

static bool is_region_connected_to_border(const Region &region) {
    for (int connection : region.connections) {
        if (connection == 0) return true;
    }
    return false;
}

static bool can_merge_with_region(const Region &region_a, const Region &region_b) {
    if (region_a.area_type != region_b.area_type) return false;
    int n = 0;
    for (int connection : region_a.connections) {
        if (connection == region_b.id) n++;
    }
    if (n > 1) return false;
    for (int floor : region_a.floors) {
        if (floor == region_b.id) return false;
    }
    return true;
}

static void remove_adjacent_duplicates(Array<int> &values) {
    for (uint32_t i = 0; i < values.length && values.length > 1;) {
        uint32_t ni = (i + 1) % values.length;
        if (values[i] == values[ni]) {
            for (uint32_t j = i; j < values.length - 1; j++) values[j] = values[j + 1];
            values.pop();
        } else {
            i++;
        }
    }
}

static bool merge_regions(Region &region_a, Region &region_b, Array<int> &merged) {
    int aid = region_a.id;
    int bid = region_b.id;

    const Array<int> &a_connections = region_a.connections;
    const Array<int> &b_connections = region_b.connections;

    int insert_a = -1;
    for (uint32_t i = 0; i < region_a.connections.length; i++) {
        if (region_a.connections[i] == bid) {
            insert_a = (int)i;
            break;
        }
    }
    if (insert_a == -1) return false;

    int insert_b = -1;
    for (uint32_t i = 0; i < b_connections.length; i++) {
        if (b_connections[i] == aid) {
            insert_b = (int)i;
            break;
        }
    }
    if (insert_b == -1) return false;

    merged.clear();
    for (uint32_t i = 0; i < a_connections.length - 1; i++) merged.push(a_connections[((uint32_t)insert_a + 1 + i) % a_connections.length]);
    for (uint32_t i = 0; i < b_connections.length - 1; i++) merged.push(b_connections[((uint32_t)insert_b + 1 + i) % b_connections.length]);
    region_a.connections.resize(merged.length);
    memcpy(region_a.connections.data, merged.data, merged.length * sizeof(int));

    remove_adjacent_duplicates(region_a.connections);

    for (int floor : region_b.floors) add_unique_floor_region(region_a, floor);
    region_a.span_count += region_b.span_count;
    region_b.span_count = 0;
    region_b.connections.release();

    return true;
}

static void replace_neighbor(Region &region, int old_id, int new_id) {
    bool neighbors_changed = false;
    for (int &connection : region.connections) {
        if (connection == old_id) {
            connection = new_id;
            neighbors_changed = true;
        }
    }
    for (int &floor : region.floors) {
        if (floor == old_id) floor = new_id;
    }
    if (neighbors_changed) remove_adjacent_duplicates(region.connections);
}

static int neighbor_region(const CompactHeightfield &compact_heightfield, const uint16_t *source_regions, int x, int y, uint32_t i, int direction) {
    const CompactHeightfieldSpan &span = compact_heightfield.spans[i];
    if (get_con(span, direction) == NOT_CONNECTED) return 0;
    return source_regions[neighbor_index(compact_heightfield, x, y, direction, span)];
}

static bool is_solid_edge(const CompactHeightfield &compact_heightfield, const uint16_t *source_regions, int x, int y, uint32_t i, int direction) {
    return neighbor_region(compact_heightfield, source_regions, x, y, i, direction) != source_regions[i];
}

static void walk_contour(BuildContext &context, int x, int y, uint32_t i, int direction, const CompactHeightfield &compact_heightfield,
                         const uint16_t *source_regions, Array<int> &contour) {
    int start_direction = direction;
    uint32_t start_i = i;

    int current_region = neighbor_region(compact_heightfield, source_regions, x, y, i, direction);
    contour.push(current_region);

    int iteration = 0;
    int current_x = x;
    int current_y = y;
    uint32_t current_i = i;
    int current_direction = direction;

    while (++iteration < 40000) {
        const CompactHeightfieldSpan &span = compact_heightfield.spans[current_i];

        if (is_solid_edge(compact_heightfield, source_regions, current_x, current_y, current_i, current_direction)) {
            int r = neighbor_region(compact_heightfield, source_regions, current_x, current_y, current_i, current_direction);
            if (r != current_region) {
                current_region = r;
                contour.push(current_region);
            }

            current_direction = (current_direction + 1) & 0x3;
        } else {
            if (get_con(span, current_direction) == NOT_CONNECTED) {
                build_context::warn(context, "walkContour: encountered unexpected disconnected neighbour at (%d, %d)", current_x, current_y);
                return;
            }
            uint32_t ni = neighbor_index(compact_heightfield, current_x, current_y, current_direction, span);
            current_x += common::DIR_OFFSETS[current_direction][0];
            current_y += common::DIR_OFFSETS[current_direction][1];
            current_i = ni;
            current_direction = (current_direction + 3) & 0x3;
        }

        if (start_i == current_i && start_direction == current_direction) break;
    }

    if (contour.length > 1) remove_adjacent_duplicates(contour);
}

static void compress_region_ids(Array<Region> &regions, int &out_max_regions) {
    uint32_t region_count = regions.length;
    for (Region &region : regions) {
        region.remap = false;
        if (region.id == 0) continue;
        if (region.id & BORDER_REG) continue;
        region.remap = true;
    }

    int region_id_generator = 0;
    for (uint32_t i = 0; i < region_count; i++) {
        if (!regions[i].remap) continue;
        int old_id = regions[i].id;
        int new_id = ++region_id_generator;
        for (uint32_t j = i; j < region_count; j++) {
            if (regions[j].id == old_id) {
                regions[j].id = new_id;
                regions[j].remap = false;
            }
        }
    }
    out_max_regions = region_id_generator;
}

static void remap_source_regions(const CompactHeightfield &compact_heightfield, const Array<Region> &regions, uint16_t *source_regions) {
    for (uint32_t i = 0; i < compact_heightfield.spans.length; i++) {
        if ((source_regions[i] & BORDER_REG) == 0) source_regions[i] = (uint16_t)regions[(uint32_t)source_regions[i]].id;
    }
}

static bool merge_and_filter_regions(BuildContext &context, int min_region_area, int merge_region_size, CompactHeightfield &compact_heightfield,
                                     uint16_t *source_regions, Array<int> &overlaps) {
    Array<int> merged_connections = {};
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;
    int region_count = compact_heightfield.max_regions + 1;

    Array<Region> regions = create_regions(region_count);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + y * w)];
            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                int r = source_regions[i];
                if (r == 0 || r >= region_count) continue;

                Region &region = regions[(uint32_t)r];
                region.span_count++;

                for (uint32_t j = cell.index; j < cell.index + cell.count; j++) {
                    if (i == j) continue;
                    int floor_id = source_regions[j];
                    if (floor_id == 0 || floor_id >= region_count) continue;
                    if (floor_id == r) region.overlap = true;
                    add_unique_floor_region(region, floor_id);
                }

                if (region.connections.length > 0) continue;

                region.area_type = compact_heightfield.areas[i];

                int neighbor_direction = -1;
                for (int direction = 0; direction < 4; direction++) {
                    if (is_solid_edge(compact_heightfield, source_regions, x, y, i, direction)) {
                        neighbor_direction = direction;
                        break;
                    }
                }

                if (neighbor_direction != -1) walk_contour(context, x, y, i, neighbor_direction, compact_heightfield, source_regions,
                                                           region.connections);
            }
        }
    }

    Array<int> stack = {};
    Array<int> trace = {};

    for (int i = 0; i < region_count; i++) {
        Region &region = regions[(uint32_t)i];
        if (region.id == 0 || region.id & BORDER_REG) continue;
        if (region.span_count == 0) continue;
        if (region.visited) continue;

        bool connects_to_border = false;
        int span_count = 0;
        stack.clear();
        trace.clear();

        region.visited = true;
        stack.push(i);

        while (stack.length > 0) {
            int ri = stack.pop();
            Region &current_region = regions[(uint32_t)ri];

            span_count += current_region.span_count;
            trace.push(ri);

            for (int connection : current_region.connections) {
                if (connection & BORDER_REG) {
                    connects_to_border = true;
                    continue;
                }
                Region &neighbor_region = regions[(uint32_t)connection];
                if (neighbor_region.visited) continue;
                if (neighbor_region.id == 0 || neighbor_region.id & BORDER_REG) continue;

                stack.push(neighbor_region.id);
                neighbor_region.visited = true;
            }
        }

        if (span_count < min_region_area && !connects_to_border) {
            for (int traced : trace) {
                regions[(uint32_t)traced].span_count = 0;
                regions[(uint32_t)traced].id = 0;
            }
        }
    }

    int merge_count = 0;
    do {
        merge_count = 0;
        for (int i = 0; i < region_count; i++) {
            Region &region = regions[(uint32_t)i];
            if (region.id == 0 || region.id & BORDER_REG) continue;
            if (region.overlap) continue;
            if (region.span_count == 0) continue;

            if (region.span_count > merge_region_size && is_region_connected_to_border(region)) continue;

            int smallest = 0xfffffff;
            int merge_id = region.id;
            for (int connection : region.connections) {
                if (connection & BORDER_REG) continue;
                const Region &merge_region = regions[(uint32_t)connection];
                if (merge_region.id == 0 || merge_region.id & BORDER_REG || merge_region.overlap) continue;
                if (merge_region.span_count < smallest && can_merge_with_region(region, merge_region) && can_merge_with_region(merge_region, region)) {
                    smallest = merge_region.span_count;
                    merge_id = merge_region.id;
                }
            }

            if (merge_id != region.id) {
                int old_id = region.id;
                Region &target = regions[(uint32_t)merge_id];

                if (merge_regions(target, region, merged_connections)) {
                    for (int j = 0; j < region_count; j++) {
                        Region &other = regions[(uint32_t)j];
                        if (other.id == 0 || other.id & BORDER_REG) continue;
                        if (other.id == old_id) other.id = merge_id;
                        replace_neighbor(other, old_id, merge_id);
                    }
                    merge_count++;
                }
            }
        }
    } while (merge_count > 0);

    compress_region_ids(regions, compact_heightfield.max_regions);
    remap_source_regions(compact_heightfield, regions, source_regions);

    for (const Region &region : regions) {
        if (region.overlap) overlaps.push(region.id);
    }

    stack.release();
    trace.release();
    release_regions(regions);
    merged_connections.release();
    return true;
}

static void store_regions(CompactHeightfield &compact_heightfield, const uint16_t *source_regions) {
    for (uint32_t i = 0; i < compact_heightfield.spans.length; i++) compact_heightfield.spans[i].region = source_regions[i];
}

bool build_regions(BuildContext &context, CompactHeightfield &compact_heightfield, int border_size, int min_region_area,
                   int merge_region_area) {
    Array<uint16_t> source_regions = {};
    Array<uint16_t> source_distances = {};
    source_regions.resize(compact_heightfield.spans.length, 0);
    source_distances.resize(compact_heightfield.spans.length, 0);

    int region_id = 1;
    int level = (compact_heightfield.max_distance + 1) & ~1;

    LevelStack level_stacks[NB_STACKS] = {};
    LevelStack stack = {};
    LevelStack unassigned = {};
    Array<DirtyEntry> dirty_entries = {};

    region_id = paint_border_regions(border_size, region_id, compact_heightfield, source_regions.data);
    compact_heightfield.border_size = border_size;
    gather_unassigned_spans(compact_heightfield, source_regions.data, unassigned);

    bool ok = true;
    int stack_id = -1;
    while (ok && level > 0) {
        level = level >= 2 ? level - 2 : 0;
        stack_id = (stack_id + 1) & (NB_STACKS - 1);

        if (stack_id == 0) {
            sort_cells_by_level(level, compact_heightfield, source_regions.data, NB_STACKS, level_stacks, 1, unassigned);
        } else {
            append_stacks(level_stacks[stack_id - 1], level_stacks[stack_id], source_regions.data);
        }

        expand_regions(EXPAND_ITERS, level, compact_heightfield, source_regions.data, source_distances.data, level_stacks[stack_id], false,
                       dirty_entries);

        for (uint32_t j = 0; j < level_stacks[stack_id].length; j++) {
            LevelStackEntry current = level_stacks[stack_id][j];
            if (current.index < 0 || source_regions[(uint32_t)current.index] != 0) continue;

            if (flood_region(current.x, current.y, current.index, level, region_id, compact_heightfield, source_regions.data, source_distances.data,
                             stack)) {
                if (region_id == 0xffff) {
                    build_context::error(context, "Region ID overflow");
                    ok = false;
                    break;
                }
                region_id++;
            }
        }
    }

    if (ok) {
        expand_regions(EXPAND_ITERS * 8, 0, compact_heightfield, source_regions.data, source_distances.data, stack, true, dirty_entries);

        Array<int> overlaps = {};
        compact_heightfield.max_regions = region_id;

        if (!merge_and_filter_regions(context, min_region_area, merge_region_area, compact_heightfield, source_regions.data, overlaps)) {
            build_context::error(context, "Failed to merge and filter regions");
            ok = false;
        } else {
            store_regions(compact_heightfield, source_regions.data);
        }
        overlaps.release();
    }

    for (LevelStack &level_stack : level_stacks) level_stack.release();
    stack.release();
    unassigned.release();
    dirty_entries.release();
    source_regions.release();
    source_distances.release();
    return ok;
}

struct SweepSpan {
    int rid;
    int id;
    int ns;
    int nei;
};

static int sweep_monotone_rows(const CompactHeightfield &compact_heightfield, int border_size, int id, uint16_t *source_regions) {
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;

    Array<SweepSpan> sweeps = {};
    sweeps.resize((uint32_t)scalar::max(w, h), {0, 0, 0, 0});

    Array<int> previous = {};
    previous.resize(256, 0);

    for (int y = border_size; y < h - border_size; y++) {
        if (previous.length < (uint32_t)id + 1) previous.resize((uint32_t)id + 1, 0);
        for (int i = 0; i < id; i++) previous[(uint32_t)i] = 0;
        int rid = 1;

        for (int x = border_size; x < w - border_size; x++) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + y * w)];

            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                const CompactHeightfieldSpan &span = compact_heightfield.spans[i];
                if (compact_heightfield.areas[i] == NULL_AREA) continue;

                int previous_id = 0;
                if (get_con(span, 0) != NOT_CONNECTED) {
                    uint32_t ai = neighbor_index(compact_heightfield, x, y, 0, span);
                    if ((source_regions[ai] & BORDER_REG) == 0 && compact_heightfield.areas[i] == compact_heightfield.areas[ai]) {
                        previous_id = source_regions[ai];
                    }
                }

                if (!previous_id) {
                    previous_id = rid++;
                    sweeps[(uint32_t)previous_id].rid = previous_id;
                    sweeps[(uint32_t)previous_id].ns = 0;
                    sweeps[(uint32_t)previous_id].nei = 0;
                }

                if (get_con(span, 3) != NOT_CONNECTED) {
                    uint32_t ai = neighbor_index(compact_heightfield, x, y, 3, span);
                    if (source_regions[ai] && (source_regions[ai] & BORDER_REG) == 0 && compact_heightfield.areas[i] == compact_heightfield.areas[ai]) {
                        int nr = source_regions[ai];
                        SweepSpan &sweep = sweeps[(uint32_t)previous_id];
                        if (!sweep.nei || sweep.nei == nr) {
                            sweep.nei = nr;
                            sweep.ns++;
                            previous[(uint32_t)nr]++;
                        } else {
                            sweep.nei = NULL_NEI;
                        }
                    }
                }

                source_regions[i] = (uint16_t)previous_id;
            }
        }

        for (int i = 1; i < rid; i++) {
            SweepSpan &sweep = sweeps[(uint32_t)i];
            if (sweep.nei != NULL_NEI && sweep.nei != 0 && previous[(uint32_t)sweep.nei] == sweep.ns) {
                sweep.id = sweep.nei;
            } else {
                sweep.id = id++;
            }
        }

        for (int x = border_size; x < w - border_size; x++) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + y * w)];
            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                if (source_regions[i] > 0 && source_regions[i] < rid) source_regions[i] = (uint16_t)sweeps[(uint32_t)source_regions[i]].id;
            }
        }
    }

    sweeps.release();
    previous.release();
    return id;
}

bool build_regions_monotone(BuildContext &context, CompactHeightfield &compact_heightfield, int border_size, int min_region_area,
                            int merge_region_area) {
    Array<uint16_t> source_regions = {};
    source_regions.resize(compact_heightfield.spans.length, 0);

    int id = paint_border_regions(border_size, 1, compact_heightfield, source_regions.data);
    compact_heightfield.border_size = border_size;

    id = sweep_monotone_rows(compact_heightfield, border_size, id, source_regions.data);
    if (id > 0xffff) {
        build_context::error(context, "Region ID overflow");
        source_regions.release();
        return false;
    }

    Array<int> overlaps = {};
    compact_heightfield.max_regions = id;

    bool ok = merge_and_filter_regions(context, min_region_area, merge_region_area, compact_heightfield, source_regions.data, overlaps);
    if (ok) store_regions(compact_heightfield, source_regions.data);

    overlaps.release();
    source_regions.release();
    return ok;
}

static void add_unique_connection(Region &region, int n) {
    for (int connection : region.connections) {
        if (connection == n) return;
    }
    region.connections.push(n);
}

static bool merge_and_filter_layer_regions(int min_region_area, const CompactHeightfield &compact_heightfield, uint16_t *source_regions,
                                           int &max_region_id) {
    int w = compact_heightfield.width;
    int h = compact_heightfield.height;
    int region_count = max_region_id + 1;

    Array<Region> regions = create_regions(region_count);

    Array<int> layer_regions = {};
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            CompactHeightfieldCell cell = compact_heightfield.cells[(uint32_t)(x + y * w)];
            layer_regions.clear();

            for (uint32_t i = cell.index; i < cell.index + cell.count; i++) {
                const CompactHeightfieldSpan &span = compact_heightfield.spans[i];
                int ri = source_regions[i];
                if (ri == 0 || ri >= region_count) continue;
                Region &region = regions[(uint32_t)ri];

                region.span_count++;
                region.area_type = compact_heightfield.areas[i];
                region.ymin = scalar::min(region.ymin, span.y);
                region.ymax = scalar::max(region.ymax, span.y);

                layer_regions.push(ri);

                for (int direction = 0; direction < 4; direction++) {
                    if (get_con(span, direction) == NOT_CONNECTED) continue;
                    int rai = source_regions[neighbor_index(compact_heightfield, x, y, direction, span)];
                    if (rai > 0 && rai < region_count && rai != ri) add_unique_connection(region, rai);
                    if (rai & BORDER_REG) region.connects_to_border = true;
                }
            }

            for (uint32_t i = 0; i + 1 < layer_regions.length; i++) {
                for (uint32_t j = i + 1; j < layer_regions.length; j++) {
                    if (layer_regions[i] == layer_regions[j]) continue;
                    add_unique_floor_region(regions[(uint32_t)layer_regions[i]], layer_regions[j]);
                    add_unique_floor_region(regions[(uint32_t)layer_regions[j]], layer_regions[i]);
                }
            }
        }
    }
    layer_regions.release();

    int layer_id = 1;

    for (Region &region : regions) region.id = 0;

    Array<int> queue = {};
    for (int i = 1; i < region_count; i++) {
        Region &root = regions[(uint32_t)i];
        if (root.id != 0) continue;

        root.id = layer_id;
        queue.clear();
        queue.push(i);

        for (uint32_t head = 0; head < queue.length; head++) {
            Region &region = regions[(uint32_t)queue[head]];

            for (uint32_t j = 0; j < region.connections.length; j++) {
                int neighbor = region.connections[j];
                Region &neighbor_region = regions[(uint32_t)neighbor];
                if (neighbor_region.id != 0) continue;
                if (region.area_type != neighbor_region.area_type) continue;

                bool overlap = false;
                for (int floor : root.floors) {
                    if (floor == neighbor) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) continue;

                queue.push(neighbor);

                neighbor_region.id = layer_id;
                for (uint32_t k = 0; k < neighbor_region.floors.length; k++) add_unique_floor_region(root, neighbor_region.floors[k]);
                root.ymin = scalar::min(root.ymin, neighbor_region.ymin);
                root.ymax = scalar::max(root.ymax, neighbor_region.ymax);
                root.span_count += neighbor_region.span_count;
                neighbor_region.span_count = 0;
                root.connects_to_border = root.connects_to_border || neighbor_region.connects_to_border;
            }
        }

        layer_id++;
    }
    queue.release();

    for (int i = 0; i < region_count; i++) {
        Region &region = regions[(uint32_t)i];
        if (region.span_count > 0 && region.span_count < min_region_area && !region.connects_to_border) {
            int layer = region.id;
            for (Region &other : regions) {
                if (other.id == layer) other.id = 0;
            }
        }
    }

    compress_region_ids(regions, max_region_id);
    remap_source_regions(compact_heightfield, regions, source_regions);

    release_regions(regions);
    return true;
}

bool build_layer_regions(BuildContext &context, CompactHeightfield &compact_heightfield, int border_size, int min_region_area) {
    Array<uint16_t> source_regions = {};
    source_regions.resize(compact_heightfield.spans.length, 0);

    int id = paint_border_regions(border_size, 1, compact_heightfield, source_regions.data);
    compact_heightfield.border_size = border_size;

    id = sweep_monotone_rows(compact_heightfield, border_size, id, source_regions.data);
    if (id > 0xffff) {
        build_context::error(context, "Region ID overflow");
        source_regions.release();
        return false;
    }

    compact_heightfield.max_regions = id;
    int max_region_id = compact_heightfield.max_regions;

    bool ok = merge_and_filter_layer_regions(min_region_area, compact_heightfield, source_regions.data, max_region_id);
    if (ok) {
        compact_heightfield.max_regions = max_region_id;
        store_regions(compact_heightfield, source_regions.data);
    }

    source_regions.release();
    return ok;
}

}

}
