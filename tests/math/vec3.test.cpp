#include "roam/math/box3.h"
#include "roam/math/vec3.h"
#include "tests/harness/test.h"

using namespace roam;

TEST(vec3_arithmetic) {
    Vec3 a = {1.0f, 2.0f, 3.0f};
    Vec3 b = {4.0f, 5.0f, 6.0f};
    CHECK(vec3::exactly_equals(vec3::add(a, b), {5.0f, 7.0f, 9.0f}));
    CHECK(vec3::exactly_equals(vec3::subtract(b, a), {3.0f, 3.0f, 3.0f}));
    CHECK(vec3::exactly_equals(vec3::scale_and_add(a, b, 2.0f), {9.0f, 12.0f, 15.0f}));
    CHECK(vec3::dot(a, b) == 32.0f);
    CHECK(vec3::exactly_equals(vec3::cross({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), {0.0f, 0.0f, 1.0f}));
}

TEST(vec3_normalize_keeps_zero_at_zero) {
    CHECK(vec3::exactly_equals(vec3::normalize({0.0f, 0.0f, 0.0f}), {0.0f, 0.0f, 0.0f}));
    CHECK_NEAR(vec3::length(vec3::normalize({3.0f, 4.0f, 12.0f})), 1.0, 1e-6);
}

TEST(vec3_equals_is_relative) {
    CHECK(vec3::equals({1000000.0f, 0.0f, 0.0f}, {1000000.5f, 0.0f, 0.0f}));
    CHECK(!vec3::equals({1.0f, 0.0f, 0.0f}, {1.001f, 0.0f, 0.0f}));
}

TEST(vec3_load_and_store_use_float_offsets) {
    float buffer[6] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    CHECK(vec3::exactly_equals(vec3::load(buffer, 3), {3.0f, 4.0f, 5.0f}));
    vec3::store(buffer, 0, {9.0f, 8.0f, 7.0f});
    CHECK(buffer[0] == 9.0f && buffer[2] == 7.0f);
}

TEST(box3_intersects_and_expands) {
    Box3 box = box3::expand_by_point(box3::empty(), {0.0f, 0.0f, 0.0f});
    box = box3::expand_by_point(box, {1.0f, 1.0f, 1.0f});
    CHECK(box3::intersects(box, {{0.5f, 0.5f, 0.5f}, {2.0f, 2.0f, 2.0f}}));
    CHECK(!box3::intersects(box, {{1.5f, 0.0f, 0.0f}, {2.0f, 1.0f, 1.0f}}));
    CHECK(box3::contains_point(box, {1.0f, 0.0f, 0.5f}));
}
