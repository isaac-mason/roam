#include "roam/blocks/agents/path_corridor.h"
#include "tests/harness/test.h"

using namespace roam;

static Array<NodeRef> refs(Span<const NodeRef> values) {
    Array<NodeRef> array = {};
    for (NodeRef value : values) array.push(value);
    return array;
}

static bool equals(const Array<NodeRef> &actual, Span<const NodeRef> expected) {
    if (actual.length != expected.capacity) return false;
    for (uint32_t i = 0; i < actual.length; i++) {
        if (actual[i] != expected[i]) return false;
    }
    return true;
}

static bool merge_start_moved_equals(Span<const NodeRef> path_values, Span<const NodeRef> visited_values, Span<const NodeRef> expected) {
    Array<NodeRef> path = refs(path_values);
    Array<NodeRef> visited = refs(visited_values);
    path_corridor::merge_start_moved(path, visited);
    bool result = equals(path, expected);
    visited.release();
    path.release();
    return result;
}

TEST_CASE("merge start moved should handle empty input") {
    CHECK(merge_start_moved_equals({}, {}, {}));
}

TEST_CASE("merge start moved should handle empty visited") {
    const NodeRef path[] = {1};
    const NodeRef expected[] = {1};
    CHECK(merge_start_moved_equals(path, {}, expected));
}

TEST_CASE("merge start moved should strip visited points from path except last") {
    const NodeRef path[] = {1, 2};
    const NodeRef visited[] = {1, 2};
    const NodeRef expected[] = {2};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should add visited points not present in path in reverse order") {
    const NodeRef path[] = {1, 2};
    const NodeRef visited[] = {1, 2, 3, 4};
    const NodeRef expected[] = {4, 3, 2};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should not change path if there is no intersection with visited") {
    const NodeRef path[] = {1, 2};
    const NodeRef visited[] = {3, 4};
    const NodeRef expected[] = {1, 2};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should save unvisited path points") {
    const NodeRef path[] = {1, 2};
    const NodeRef visited[] = {1, 3};
    const NodeRef expected[] = {3, 1, 2};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle complex merge scenario") {
    const NodeRef path[] = {1, 2, 3, 4};
    const NodeRef visited[] = {1, 5, 6, 2};
    const NodeRef expected[] = {2, 3, 4};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle when visited ends at path start") {
    const NodeRef path[] = {1, 2, 3};
    const NodeRef visited[] = {4, 5, 1};
    const NodeRef expected[] = {1, 2, 3};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle single element path and visited with same element") {
    const NodeRef path[] = {1};
    const NodeRef visited[] = {1};
    const NodeRef expected[] = {1};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle visited path that completely replaces current path") {
    const NodeRef path[] = {1, 2};
    const NodeRef visited[] = {3, 4, 5, 2};
    const NodeRef expected[] = {2};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle multiple common polygons and use furthest") {
    const NodeRef path[] = {1, 2, 3, 2, 4};
    const NodeRef visited[] = {1, 5, 2};
    const NodeRef expected[] = {2, 4};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle long visited path") {
    const NodeRef path[] = {1, 2, 3};
    const NodeRef visited[] = {1, 10, 11, 12, 13, 14, 15, 3};
    const NodeRef expected[] = {3};
    CHECK(merge_start_moved_equals(path, visited, expected));
}

TEST_CASE("merge start moved should handle visited starting in middle of path") {
    const NodeRef path[] = {1, 2, 3, 4, 5};
    const NodeRef visited[] = {3, 6, 7, 8};
    const NodeRef expected[] = {8, 7, 6, 3, 4, 5};
    CHECK(merge_start_moved_equals(path, visited, expected));
}
