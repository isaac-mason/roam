#include "roam/generate/common.h"
#include "roam/generate/heightfield.h"
#include "tests/harness/test.h"

using namespace roam;

static const Box3 SMALL_BOUNDS = {{0, 0, 0}, {3, 3, 3}};
static const Box3 LARGE_BOUNDS = {{0, 0, 0}, {10, 10, 10}};

static HeightfieldSpan *first_span(Heightfield &heightfield, int x, int z) {
    int32_t index = heightfield.spans[(uint32_t)(x + z * heightfield.width)];
    return index == NULL_SPAN ? nullptr : &heightfield.span_pool[(uint32_t)index];
}

static HeightfieldSpan *next_span(Heightfield &heightfield, const HeightfieldSpan *span) {
    return span->next == NULL_SPAN ? nullptr : &heightfield.span_pool[(uint32_t)span->next];
}

static uint32_t count_non_empty_columns(const Heightfield &heightfield) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < heightfield.spans.length; i++) {
        if (heightfield.spans[i] != NULL_SPAN) count++;
    }
    return count;
}

static bool rasterize(Heightfield &heightfield, Span<const float> positions, Span<const uint32_t> indices, Span<const uint8_t> tri_area_ids,
                      int flag_merge_threshold = 1) {
    BuildContext context = {};
    bool result = heightfield::rasterize_triangles(context, heightfield, {positions, indices}, tri_area_ids, flag_merge_threshold);
    build_context::release(context);
    return result;
}

TEST_CASE("calculate grid size for simple square bounds") {
    GridSize grid_size = heightfield::calculate_grid_size({{0, 0, 0}, {10, 10, 10}}, 1.0f);
    CHECK(grid_size.width == 10);
    CHECK(grid_size.height == 10);
}

TEST_CASE("calculate grid size handles non uniform bounds correctly") {
    GridSize grid_size = heightfield::calculate_grid_size({{-5, 0, -5}, {15, 10, 5}}, 0.5f);
    CHECK(grid_size.width == 40);
    CHECK(grid_size.height == 20);
}

TEST_CASE("calculate grid size rounds correctly with fractional cell sizes") {
    GridSize grid_size = heightfield::calculate_grid_size({{0, 0, 0}, {10.3f, 5, 10.7f}}, 1.0f);
    CHECK(grid_size.width == 10);
    CHECK(grid_size.height == 11);
}

TEST_CASE("create heightfield with correct dimensions") {
    Heightfield heightfield = heightfield::create(10, 20, {{0, 0, 0}, {10, 5, 20}}, 1.0f, 0.5f);
    CHECK(heightfield.width == 10);
    CHECK(heightfield.height == 20);
    CHECK(heightfield.spans.length == 200);
    heightfield::release(heightfield);
}

TEST_CASE("create heightfield initializes all spans to null") {
    Heightfield heightfield = heightfield::create(5, 5, {{0, 0, 0}, {5, 5, 5}}, 1.0f, 0.5f);
    for (uint32_t i = 0; i < heightfield.spans.length; i++) CHECK(heightfield.spans[i] == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("create heightfield stores bounds and cell parameters correctly") {
    Box3 bounds = {{0, 0, 0}, {10, 5, 20}};
    Heightfield heightfield = heightfield::create(10, 20, bounds, 1.0f, 0.5f);
    CHECK(heightfield.bounds.min.x == bounds.min.x);
    CHECK(heightfield.bounds.min.y == bounds.min.y);
    CHECK(heightfield.bounds.min.z == bounds.min.z);
    CHECK(heightfield.bounds.max.x == bounds.max.x);
    CHECK(heightfield.bounds.max.y == bounds.max.y);
    CHECK(heightfield.bounds.max.z == bounds.max.z);
    CHECK(heightfield.cell_size == 1.0f);
    CHECK(heightfield.cell_height == 0.5f);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span to empty column") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->min == 0);
    CHECK(span->max == 10);
    CHECK(span->area == 1);
    CHECK(span->next == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span adds non overlapping span before existing span") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    HeightfieldSpan *span1 = first_span(heightfield, 1, 1);
    REQUIRE(span1 != nullptr);
    HeightfieldSpan *span2 = next_span(heightfield, span1);
    REQUIRE(span2 != nullptr);
    CHECK(span1->min == 0);
    CHECK(span1->max == 10);
    CHECK(span2->min == 20);
    CHECK(span2->max == 30);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span adds non overlapping span after existing span") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    HeightfieldSpan *span1 = first_span(heightfield, 1, 1);
    REQUIRE(span1 != nullptr);
    HeightfieldSpan *span2 = next_span(heightfield, span1);
    REQUIRE(span2 != nullptr);
    CHECK(span1->min == 0);
    CHECK(span1->max == 10);
    CHECK(span2->min == 20);
    CHECK(span2->max == 30);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span merges completely overlapping spans") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 12, 18, 2, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->min == 10);
    CHECK(span->max == 20);
    CHECK(span->next == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span merges partially overlapping spans") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 15, 25, 2, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->min == 10);
    CHECK(span->max == 25);
    CHECK(span->next == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span extends span when new span is taller") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 30, 2, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->min == 10);
    CHECK(span->max == 30);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span extends span when new span starts lower") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 5, 20, 2, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->min == 5);
    CHECK(span->max == 20);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span merges multiple consecutive spans") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 40, 50, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 5, 45, 2, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->min == 0);
    CHECK(span->max == 50);
    CHECK(span->next == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span respects area priority when spans have similar max height within threshold") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 5, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 10, 1);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == 10);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span keeps original area when max heights differ beyond threshold") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 5, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 25, 10, 2);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == 10);
    CHECK(span->max == 25);
    heightfield::release(heightfield);
}

TEST_CASE("add heightfield span maintains span ordering after multiple insertions") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 5, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 40, 50, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 15, 1, 1);
    const int expected_mins[] = {0, 10, 20, 40};
    uint32_t count = 0;
    for (HeightfieldSpan *span = first_span(heightfield, 1, 1); span != nullptr; span = next_span(heightfield, span)) {
        REQUIRE(count < 4);
        CHECK(span->min == expected_mins[count]);
        count++;
    }
    CHECK(count == 4);
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles rasterizes single horizontal triangle") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {2, 2, 2, 4, 2, 2, 3, 2, 4};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    CHECK(count_non_empty_columns(heightfield) > 0);
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles rasterizes single vertical triangle") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {2, 0, 2, 4, 5, 2, 3, 0, 4};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    CHECK(count_non_empty_columns(heightfield) > 0);
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles skips triangle completely outside heightfield bounds") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {20, 2, 20, 22, 2, 20, 21, 2, 22};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    for (uint32_t i = 0; i < heightfield.spans.length; i++) CHECK(heightfield.spans[i] == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles clips triangle partially outside heightfield bounds") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {8, 2, 8, 12, 2, 8, 10, 2, 12};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    CHECK(count_non_empty_columns(heightfield) > 0);
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles handles triangle touching heightfield boundary") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {0, 2, 0, 2, 2, 0, 1, 2, 2};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles rasterizes multiple overlapping triangles") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {2, 2, 2, 4, 2, 2, 3, 2, 4, 2, 5, 2, 4, 5, 2, 3, 5, 4};
    const uint32_t indices[] = {0, 1, 2, 3, 4, 5};
    const uint8_t tri_area_ids[] = {1, 2};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    CHECK(count_non_empty_columns(heightfield) > 0);
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles handles degenerate triangles gracefully") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {2, 2, 2, 3, 2, 2, 4, 2, 2};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles respects flag merge threshold for overlapping geometry") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {2, 2.0f, 2, 4, 2.0f, 2, 3, 2.0f, 4, 2, 2.5f, 2, 4, 2.5f, 2, 3, 2.5f, 4};
    const uint32_t indices[] = {0, 1, 2, 3, 4, 5};
    const uint8_t tri_area_ids[] = {5, 10};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids, 1));
    heightfield::release(heightfield);
}

TEST_CASE("rasterize triangles handles triangles spanning multiple cells") {
    Heightfield heightfield = heightfield::create(10, 10, LARGE_BOUNDS, 1.0f, 1.0f);
    const float positions[] = {1, 2, 1, 8, 2, 1, 4, 2, 8};
    const uint32_t indices[] = {0, 1, 2};
    const uint8_t tri_area_ids[] = {1};
    CHECK(rasterize(heightfield, positions, indices, tri_area_ids));
    CHECK(count_non_empty_columns(heightfield) > 5);
    heightfield::release(heightfield);
}

TEST_CASE("filter low hanging walkable obstacles marks span as walkable when obstacle is within walkable climb") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 12, 20, NULL_AREA, 1);
    heightfield::filter_low_hanging_walkable_obstacles(heightfield, 15);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    span = next_span(heightfield, span);
    REQUIRE(span != nullptr);
    CHECK(span->area != NULL_AREA);
    CHECK(span->area == 1);
    heightfield::release(heightfield);
}

TEST_CASE("filter low hanging walkable obstacles does not modify span when obstacle exceeds walkable climb") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, NULL_AREA, 1);
    heightfield::filter_low_hanging_walkable_obstacles(heightfield, 3);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    span = next_span(heightfield, span);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter low hanging walkable obstacles handles multiple consecutive non walkable spans correctly") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 12, 15, NULL_AREA, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 17, 20, NULL_AREA, 1);
    heightfield::filter_low_hanging_walkable_obstacles(heightfield, 6);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    span = next_span(heightfield, span);
    REQUIRE(span != nullptr);
    CHECK(span->area != NULL_AREA);
    span = next_span(heightfield, span);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter low hanging walkable obstacles preserves original walkable spans") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 0, 10, 5, 1);
    heightfield::filter_low_hanging_walkable_obstacles(heightfield, 3);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == 5);
    heightfield::release(heightfield);
}

TEST_CASE("filter low hanging walkable obstacles handles empty columns") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::filter_low_hanging_walkable_obstacles(heightfield, 3);
    for (uint32_t i = 0; i < heightfield.spans.length; i++) CHECK(heightfield.spans[i] == NULL_SPAN);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans marks span as unwalkable when neighbor drop exceeds walkable climb") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 5, 15, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 2, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans keeps span walkable when all neighbors are within walkable climb") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 0, 1, 22, 32, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 18, 28, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 0, 21, 31, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 2, 19, 29, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 2, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == 1);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans marks span as unwalkable when slope is too steep") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 0, 1, 22, 32, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 15, 25, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 2, 3);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans handles edge of heightfield correctly") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 0, 0, 20, 30, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 2, 5);
    HeightfieldSpan *span = first_span(heightfield, 0, 0);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans handles corners with multiple steep neighbors") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 0, 0, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 0, 5, 15, 1, 1);
    heightfield::add_heightfield_span(heightfield, 0, 1, 5, 15, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 2, 5);
    HeightfieldSpan *span = first_span(heightfield, 0, 0);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans considers ceiling height for traversability") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 31, 40, 1, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 22, 32, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 5, 5);
    CHECK(first_span(heightfield, 1, 1) != nullptr);
    heightfield::release(heightfield);
}

TEST_CASE("filter ledge spans skips non walkable spans") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 20, 30, NULL_AREA, 1);
    heightfield::add_heightfield_span(heightfield, 2, 1, 5, 15, 1, 1);
    heightfield::filter_ledge_spans(heightfield, 2, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter walkable low height spans marks span as unwalkable when ceiling is too low") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 22, 30, 1, 1);
    heightfield::filter_walkable_low_height_spans(heightfield, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    heightfield::release(heightfield);
}

TEST_CASE("filter walkable low height spans keeps span walkable when ceiling height is sufficient") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 30, 40, 1, 1);
    heightfield::filter_walkable_low_height_spans(heightfield, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == 1);
    heightfield::release(heightfield);
}

TEST_CASE("filter walkable low height spans handles spans with no ceiling") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::filter_walkable_low_height_spans(heightfield, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == 1);
    heightfield::release(heightfield);
}

TEST_CASE("filter walkable low height spans processes all spans in column") {
    Heightfield heightfield = heightfield::create(3, 3, SMALL_BOUNDS, 1.0f, 1.0f);
    heightfield::add_heightfield_span(heightfield, 1, 1, 10, 20, 1, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 22, 30, 2, 1);
    heightfield::add_heightfield_span(heightfield, 1, 1, 50, 60, 3, 1);
    heightfield::filter_walkable_low_height_spans(heightfield, 5);
    HeightfieldSpan *span = first_span(heightfield, 1, 1);
    REQUIRE(span != nullptr);
    CHECK(span->area == NULL_AREA);
    span = next_span(heightfield, span);
    REQUIRE(span != nullptr);
    CHECK(span->area == 2);
    span = next_span(heightfield, span);
    REQUIRE(span != nullptr);
    CHECK(span->area == 3);
    heightfield::release(heightfield);
}
