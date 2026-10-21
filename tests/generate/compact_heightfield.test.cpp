#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/generate/heightfield.h"
#include "roam/math/scalar.h"
#include "tests/harness/test.h"

using namespace roam;

struct GridPosition {
    int x;
    int z;
};

static CompactHeightfield build_compact(Heightfield &heightfield, int walkable_height_voxels, int walkable_climb_voxels) {
    BuildContext context = {};
    CompactHeightfield compact = compact_heightfield::build(context, heightfield, walkable_height_voxels, walkable_climb_voxels);
    build_context::release(context);
    heightfield::release(heightfield);
    return compact;
}

static CompactHeightfield create_grid_with_spans(int size) {
    Heightfield heightfield = heightfield::create(size, size, {{0, 0, 0}, {(float)size, 10, (float)size}}, 1.0f, 1.0f);
    for (int x = 0; x < size; x++) {
        for (int z = 0; z < size; z++) {
            heightfield::add_heightfield_span(heightfield, x, z, 0, 10, 1, 1);
        }
    }
    return build_compact(heightfield, 5, 3);
}

static uint32_t count_marked(const CompactHeightfield &compact, uint8_t area_id) {
    uint32_t count = 0;
    for (uint8_t area : compact.areas) {
        if (area == area_id) count++;
    }
    return count;
}

static Array<GridPosition> get_marked_positions(const CompactHeightfield &compact, uint8_t area_id, int width) {
    Array<GridPosition> positions = {};
    for (uint32_t i = 0; i < compact.cells.length; i++) {
        const CompactHeightfieldCell &cell = compact.cells[i];
        if (cell.count > 0 && compact.areas[cell.index] == area_id) {
            positions.push({(int)i % width, (int)i / width});
        }
    }
    return positions;
}

static bool contains_position(const Array<GridPosition> &positions, int x, int z) {
    for (const GridPosition &position : positions) {
        if (position.x == x && position.z == z) return true;
    }
    return false;
}

static int count_unique_x(const Array<GridPosition> &positions) {
    int count = 0;
    for (uint32_t i = 0; i < positions.length; i++) {
        bool seen = false;
        for (uint32_t j = 0; j < i; j++) {
            if (positions[j].x == positions[i].x) seen = true;
        }
        if (!seen) count++;
    }
    return count;
}

static int count_unique_z(const Array<GridPosition> &positions) {
    int count = 0;
    for (uint32_t i = 0; i < positions.length; i++) {
        bool seen = false;
        for (uint32_t j = 0; j < i; j++) {
            if (positions[j].z == positions[i].z) seen = true;
        }
        if (!seen) count++;
    }
    return count;
}

static void check_positions(const Array<GridPosition> &positions, const GridPosition *expected, uint32_t expected_count) {
    REQUIRE(positions.length == expected_count);
    for (uint32_t i = 0; i < expected_count; i++) {
        CHECK(positions[i].x == expected[i].x);
        CHECK(positions[i].z == expected[i].z);
    }
}

static const GridPosition ALIGNED_3X3_BOX[] = {
    {2, 2}, {3, 2}, {4, 2}, {2, 3}, {3, 3}, {4, 3}, {2, 4}, {3, 4}, {4, 4},
};

TEST_CASE("sets and gets connection data for a single direction") {
    CompactHeightfieldSpan span = {};
    compact_heightfield::set_con(span, 0, 5);
    CHECK(compact_heightfield::get_con(span, 0) == 5);
    CHECK(compact_heightfield::get_con(span, 1) == 0);
    CHECK(compact_heightfield::get_con(span, 2) == 0);
    CHECK(compact_heightfield::get_con(span, 3) == 0);
}

TEST_CASE("sets and gets connection data for all four directions") {
    CompactHeightfieldSpan span = {};
    compact_heightfield::set_con(span, 0, 1);
    compact_heightfield::set_con(span, 1, 2);
    compact_heightfield::set_con(span, 2, 3);
    compact_heightfield::set_con(span, 3, 4);
    CHECK(compact_heightfield::get_con(span, 0) == 1);
    CHECK(compact_heightfield::get_con(span, 1) == 2);
    CHECK(compact_heightfield::get_con(span, 2) == 3);
    CHECK(compact_heightfield::get_con(span, 3) == 4);
}

TEST_CASE("handles maximum layer index") {
    CompactHeightfieldSpan span = {};
    const int max_layer = 0x3f;
    compact_heightfield::set_con(span, 0, max_layer);
    CHECK(compact_heightfield::get_con(span, 0) == max_layer);
}

TEST_CASE("preserves other direction data when setting one direction") {
    CompactHeightfieldSpan span = {};
    compact_heightfield::set_con(span, 0, 5);
    compact_heightfield::set_con(span, 1, 10);
    compact_heightfield::set_con(span, 2, 15);
    compact_heightfield::set_con(span, 3, 20);
    compact_heightfield::set_con(span, 1, 25);
    CHECK(compact_heightfield::get_con(span, 0) == 5);
    CHECK(compact_heightfield::get_con(span, 1) == 25);
    CHECK(compact_heightfield::get_con(span, 2) == 15);
    CHECK(compact_heightfield::get_con(span, 3) == 20);
}

TEST_CASE("creates compact heightfield with correct dimensions") {
    Heightfield heightfield = heightfield::create(5, 5, {{0, 0, 0}, {5, 5, 5}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    CompactHeightfield compact = build_compact(heightfield, 2, 2);

    CHECK(compact.width == 5);
    CHECK(compact.height == 5);
    CHECK(compact.cells.length == 25u);
    compact_heightfield::release(compact);
}

TEST_CASE("counts only walkable spans") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 2, 0, 10, NULL_AREA, 1);
    CompactHeightfield compact = build_compact(heightfield, 2, 2);

    CHECK(compact.spans.length == 1u);
    compact_heightfield::release(compact);
}

TEST_CASE("adjusts upper bounds for walkable height") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 0.5f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    const int walkable_height_voxels = 4;
    CompactHeightfield compact = build_compact(heightfield, walkable_height_voxels, 2);

    CHECK(compact.bounds.max.y == 3 + walkable_height_voxels * 0.5f);
    compact_heightfield::release(compact);
}

TEST_CASE("converts heightfield spans to compact spans correctly") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 5, 15, 1, 1);
    CompactHeightfield compact = build_compact(heightfield, 2, 2);

    const CompactHeightfieldCell &cell = compact.cells[1 + 1 * 3];
    CHECK(cell.count == 1u);

    const CompactHeightfieldSpan &span = compact.spans[cell.index];
    CHECK(span.y == 15);
    CHECK(span.h > 0u);
    CHECK(compact.areas[cell.index] == 1);
    compact_heightfield::release(compact);
}

TEST_CASE("establishes neighbor connections in cardinal directions") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 0, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 0, 10, 1, 1);
    CompactHeightfield compact = build_compact(heightfield, 5, 3);

    const CompactHeightfieldCell &middle_cell = compact.cells[1 + 1 * 3];
    const CompactHeightfieldSpan &middle_span = compact.spans[middle_cell.index];

    CHECK(compact_heightfield::get_con(middle_span, 0) != NOT_CONNECTED);
    CHECK(compact_heightfield::get_con(middle_span, 2) != NOT_CONNECTED);
    compact_heightfield::release(compact);
}

TEST_CASE("only connects neighbors within walkable height gap") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 0, 10, 1, 1);
    const int walkable_height_voxels = 5;
    CompactHeightfield compact = build_compact(heightfield, walkable_height_voxels, 10);

    const CompactHeightfieldCell &cell = compact.cells[1 + 1 * 3];
    const CompactHeightfieldSpan &span = compact.spans[cell.index];

    CHECK(compact_heightfield::get_con(span, 2) != NOT_CONNECTED);
    compact_heightfield::release(compact);
}

TEST_CASE("only connects neighbors within walkable climb distance") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 20, 30, 1, 1);
    const int walkable_climb_voxels = 5;
    CompactHeightfield compact = build_compact(heightfield, 5, walkable_climb_voxels);

    const CompactHeightfieldCell &cell = compact.cells[1 + 1 * 3];
    const CompactHeightfieldSpan &span = compact.spans[cell.index];

    CHECK(compact_heightfield::get_con(span, 2) == NOT_CONNECTED);
    compact_heightfield::release(compact);
}

TEST_CASE("handles cells with no walkable spans") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    CompactHeightfield compact = build_compact(heightfield, 2, 2);

    const CompactHeightfieldCell &empty_cell = compact.cells[0];
    CHECK(empty_cell.index == 0u);
    CHECK(empty_cell.count == 0u);
    compact_heightfield::release(compact);
}

TEST_CASE("handles multiple spans in same column") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 2, 1);
    CompactHeightfield compact = build_compact(heightfield, 2, 2);

    const CompactHeightfieldCell &cell = compact.cells[1 + 1 * 3];
    REQUIRE(cell.count == 2u);

    CHECK(compact.spans[cell.index].y == 10);
    CHECK(compact.spans[cell.index + 1].y == 30);
    compact_heightfield::release(compact);
}

TEST_CASE("handles edge cells with missing neighbors") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 3, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 0, 0, 0, 10, 1, 1);
    CompactHeightfield compact = build_compact(heightfield, 5, 3);

    const CompactHeightfieldCell &corner_cell = compact.cells[0];
    const CompactHeightfieldSpan &corner_span = compact.spans[corner_cell.index];

    CHECK(compact_heightfield::get_con(corner_span, 0) == NOT_CONNECTED);
    CHECK(compact_heightfield::get_con(corner_span, 3) == NOT_CONNECTED);
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within axis aligned box bounds") {
    CompactHeightfield compact = create_grid_with_spans(5);

    compact_heightfield::mark_box_area(compact, {{1, 0, 1}, {3, 10, 3}}, 2);

    uint32_t marked_count = count_marked(compact, 2);
    CHECK(marked_count > 0u);
    CHECK(marked_count < compact.spans.length);
    CHECK(compact.areas[compact.cells[2 + 2 * 5].index] == 2);
    CHECK(compact.areas[compact.cells[0].index] == 1);
    compact_heightfield::release(compact);
}

TEST_CASE("mark box area respects y bounds and skips null area") {
    Heightfield heightfield = heightfield::create(3, 3, {{0, 0, 0}, {3, 10, 3}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 5, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 15, 20, 2, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 0, 5, NULL_AREA, 1);
    CompactHeightfield compact = build_compact(heightfield, 5, 3);

    compact_heightfield::mark_box_area(compact, {{0, 0, 0}, {3, 7, 3}}, 3);

    const CompactHeightfieldCell &cell = compact.cells[1 + 1 * 3];
    CHECK(compact.areas[cell.index] == 3);
    CHECK(compact.areas[cell.index + 1] == 2);
    CHECK(count_marked(compact, 3) == 1u);
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within convex polygon bounds") {
    CompactHeightfield compact = create_grid_with_spans(7);

    const float vertices[] = {2.0f, 0, 2.0f, 5.0f, 0, 3.5f, 3.5f, 0, 5.0f};
    compact_heightfield::mark_convex_poly_area(compact, vertices, 0, 10, 4);

    uint32_t marked_count = count_marked(compact, 4);
    CHECK(marked_count > 0u);
    CHECK(marked_count < compact.spans.length);
    CHECK(compact.areas[compact.cells[3 + 3 * 7].index] == 4);
    compact_heightfield::release(compact);
}

TEST_CASE("mark convex poly area respects y bounds and handles edge cases") {
    Heightfield heightfield = heightfield::create(5, 5, {{0, 0, 0}, {5, 10, 5}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 2, 2, 0, 5, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 2, 15, 20, 2, 1);
    CompactHeightfield compact = build_compact(heightfield, 5, 3);

    const float vertices[] = {1.0f, 0, 1.0f, 4.0f, 0, 1.0f, 4.0f, 0, 4.0f, 1.0f, 0, 4.0f};
    compact_heightfield::mark_convex_poly_area(compact, vertices, 0, 7, 5);

    const CompactHeightfieldCell &cell = compact.cells[2 + 2 * 5];
    CHECK(compact.areas[cell.index] == 5);
    CHECK(compact.areas[cell.index + 1] == 2);
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within cylinder radius") {
    CompactHeightfield compact = create_grid_with_spans(7);

    compact_heightfield::mark_cylinder_area(compact, {3.5f, 0, 3.5f}, 1.5f, 10, 6);

    uint32_t marked_count = count_marked(compact, 6);
    CHECK(marked_count > 0u);
    CHECK(marked_count < compact.spans.length);
    CHECK(compact.areas[compact.cells[3 + 3 * 7].index] == 6);
    CHECK(compact.areas[compact.cells[0].index] == 1);
    compact_heightfield::release(compact);
}

TEST_CASE("mark cylinder area respects y bounds and radius precisely") {
    Heightfield heightfield = heightfield::create(5, 5, {{0, 0, 0}, {5, 10, 5}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 2, 2, 0, 5, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 2, 15, 20, 2, 1);
    CompactHeightfield compact = build_compact(heightfield, 5, 3);

    compact_heightfield::mark_cylinder_area(compact, {2.5f, 0, 2.5f}, 1.0f, 7, 7);

    const CompactHeightfieldCell &cell = compact.cells[2 + 2 * 5];
    CHECK(compact.areas[cell.index] == 7);
    CHECK(compact.areas[cell.index + 1] == 2);
    CHECK(count_marked(compact, 7) < 5u);
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within rotated box at 0 degrees") {
    CompactHeightfield compact = create_grid_with_spans(7);

    compact_heightfield::mark_rotated_box_area(compact, {3.5f, 5, 3.5f}, {1.5f, 5, 1.5f}, 0, 2);

    Array<GridPosition> marked = get_marked_positions(compact, 2, 7);
    check_positions(marked, ALIGNED_3X3_BOX, 9);
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within rotated box at 45 degrees") {
    CompactHeightfield compact = create_grid_with_spans(9);

    compact_heightfield::mark_rotated_box_area(compact, {4.5f, 5, 4.5f}, {1.0f, 5, 1.0f}, scalar::PI / 4, 3);

    Array<GridPosition> marked = get_marked_positions(compact, 3, 9);
    const GridPosition expected[] = {{4, 3}, {3, 4}, {4, 4}, {5, 4}, {4, 5}};
    check_positions(marked, expected, 5);
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within rotated box at 90 degrees") {
    CompactHeightfield compact = create_grid_with_spans(9);

    compact_heightfield::mark_rotated_box_area(compact, {4.5f, 5, 4.5f}, {2.0f, 5, 1.0f}, scalar::PI / 2, 4);

    Array<GridPosition> marked = get_marked_positions(compact, 4, 9);
    CHECK(marked.length == 13u);

    int unique_x = count_unique_x(marked);
    int unique_z = count_unique_z(marked);
    CHECK(unique_z > unique_x);
    CHECK(unique_z >= 5);
    CHECK(unique_x <= 3);
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("marks spans within rotated box at 180 degrees") {
    CompactHeightfield compact = create_grid_with_spans(7);

    compact_heightfield::mark_rotated_box_area(compact, {3.5f, 5, 3.5f}, {1.5f, 5, 1.5f}, scalar::PI, 5);

    Array<GridPosition> marked = get_marked_positions(compact, 5, 7);
    check_positions(marked, ALIGNED_3X3_BOX, 9);
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("marks rectangular box rotated at 30 degrees") {
    CompactHeightfield compact = create_grid_with_spans(11);

    compact_heightfield::mark_rotated_box_area(compact, {5.5f, 5, 5.5f}, {2.5f, 5, 1.0f}, scalar::PI / 6, 6);

    Array<GridPosition> marked = get_marked_positions(compact, 6, 11);
    CHECK(marked.length >= 10u);
    CHECK(marked.length <= 18u);
    CHECK(contains_position(marked, 5, 5));
    CHECK(count_unique_x(marked) >= 4);
    CHECK(count_unique_z(marked) >= 3);
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("marks rectangular box rotated at minus 45 degrees") {
    CompactHeightfield compact = create_grid_with_spans(11);

    compact_heightfield::mark_rotated_box_area(compact, {5.5f, 5, 5.5f}, {2.0f, 5, 1.0f}, -scalar::PI / 4, 7);

    Array<GridPosition> marked = get_marked_positions(compact, 7, 11);
    CHECK(marked.length >= 5u);
    CHECK(marked.length <= 9u);
    CHECK(contains_position(marked, 5, 5));

    bool has_top_left = false;
    bool has_bottom_right = false;
    for (const GridPosition &position : marked) {
        if (position.x < 5 && position.z < 5) has_top_left = true;
        if (position.x > 5 && position.z > 5) has_bottom_right = true;
    }
    CHECK(has_top_left);
    CHECK(has_bottom_right);
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("mark rotated box area respects y bounds and skips null area spans") {
    Heightfield heightfield = heightfield::create(5, 5, {{0, 0, 0}, {5, 10, 5}}, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 2, 2, 0, 5, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 2, 15, 20, 2, 1);
    heightfield::add_heightfield_span(heightfield, 3, 2, 0, 5, NULL_AREA, 1);
    CompactHeightfield compact = build_compact(heightfield, 5, 3);

    compact_heightfield::mark_rotated_box_area(compact, {2.5f, 3.5f, 2.5f}, {1.0f, 3.5f, 1.0f}, scalar::PI / 4, 8);

    const CompactHeightfieldCell &cell22 = compact.cells[2 + 2 * 5];
    CHECK(compact.areas[cell22.index] == 8);
    CHECK(compact.areas[cell22.index + 1] == 2);

    const CompactHeightfieldCell &cell32 = compact.cells[3 + 2 * 5];
    if (cell32.count > 0) {
        CHECK(compact.areas[cell32.index] == NULL_AREA);
    }
    compact_heightfield::release(compact);
}

TEST_CASE("handles box partially outside grid bounds") {
    CompactHeightfield compact = create_grid_with_spans(5);

    compact_heightfield::mark_rotated_box_area(compact, {1.0f, 5, 1.0f}, {2.0f, 5, 2.0f}, scalar::PI / 4, 9);

    Array<GridPosition> marked = get_marked_positions(compact, 9, 5);
    CHECK(marked.length > 0u);
    CHECK(marked.length < compact.spans.length);
    for (const GridPosition &position : marked) {
        CHECK(position.x >= 0);
        CHECK(position.x < 5);
        CHECK(position.z >= 0);
        CHECK(position.z < 5);
    }
    marked.release();
    compact_heightfield::release(compact);
}

TEST_CASE("handles box completely outside grid bounds with early exit") {
    CompactHeightfield compact = create_grid_with_spans(5);

    compact_heightfield::mark_rotated_box_area(compact, {50.0f, 5, 50.0f}, {1.0f, 5, 1.0f}, 0, 10);

    CHECK(count_marked(compact, 10) == 0u);
    CHECK(count_marked(compact, 1) == compact.spans.length);
    compact_heightfield::release(compact);
}

TEST_CASE("small box rotated 360 degrees in 8 steps") {
    uint32_t counts[8];
    bool center_marked[8];

    for (int i = 0; i < 8; i++) {
        CompactHeightfield compact = create_grid_with_spans(9);
        float angle = ((float)i * scalar::PI) / 4;

        compact_heightfield::mark_rotated_box_area(compact, {4.5f, 5, 4.5f}, {1.0f, 5, 1.0f}, angle, 2);

        Array<GridPosition> marked = get_marked_positions(compact, 2, 9);
        counts[i] = marked.length;
        center_marked[i] = contains_position(marked, 4, 4);
        marked.release();
        compact_heightfield::release(compact);
    }

    for (int i = 0; i < 8; i++) {
        CAPTURE(i);
        CHECK(counts[i] >= 5u);
        CHECK(counts[i] <= 9u);
        CHECK(center_marked[i]);
        CHECK(counts[i] > 0u);
    }

    CHECK(counts[1] < counts[0]);
    CHECK(counts[3] < counts[2]);
}

TEST_CASE("thin rectangle rotating shows clear orientation changes") {
    const float angles[] = {0, scalar::PI / 2};
    int widths[2];
    int heights[2];

    for (int i = 0; i < 2; i++) {
        CompactHeightfield compact = create_grid_with_spans(11);

        compact_heightfield::mark_rotated_box_area(compact, {5.5f, 5, 5.5f}, {2.5f, 5, 0.5f}, angles[i], 2);

        Array<GridPosition> marked = get_marked_positions(compact, 2, 11);
        REQUIRE(marked.length > 0u);
        int min_x = marked[0].x;
        int max_x = marked[0].x;
        int min_z = marked[0].z;
        int max_z = marked[0].z;
        for (const GridPosition &position : marked) {
            min_x = scalar::min(min_x, position.x);
            max_x = scalar::max(max_x, position.x);
            min_z = scalar::min(min_z, position.z);
            max_z = scalar::max(max_z, position.z);
        }
        widths[i] = max_x - min_x + 1;
        heights[i] = max_z - min_z + 1;
        marked.release();
        compact_heightfield::release(compact);
    }

    CHECK(widths[0] > heights[0]);
    CHECK(heights[1] > widths[1]);
}
