#include "roam/util/map.h"
#include "tests/harness/test.h"

using namespace roam;

TEST_CASE("map get on empty is null") {
    Map<uint64_t, int> map = {};
    CHECK(map.get(42) == nullptr);
    CHECK(!map.remove(42));
    map.release();
}

TEST_CASE("map set then get") {
    Map<uint64_t, int> map = {};
    map.set(7, 70);
    map.set(9, 90);
    REQUIRE(map.get(7) != nullptr);
    CHECK(*map.get(7) == 70);
    CHECK(*map.get(9) == 90);
    CHECK(map.length == 2);
    map.release();
}

TEST_CASE("map set overwrites existing key") {
    Map<uint32_t, int> map = {};
    map.set(1, 10);
    map.set(1, 11);
    CHECK(map.length == 1);
    CHECK(*map.get(1) == 11);
    map.release();
}

TEST_CASE("map survives growth and removal") {
    Map<uint64_t, uint64_t> map = {};
    for (uint64_t i = 0; i < 10000; i++) map.set(i * 7919, i);
    for (uint64_t i = 0; i < 10000; i += 2) CHECK(map.remove(i * 7919));
    CHECK(map.length == 5000);
    for (uint64_t i = 0; i < 10000; i++) {
        const uint64_t *value = map.get(i * 7919);
        if (i % 2 == 0) {
            CHECK(value == nullptr);
        } else {
            REQUIRE(value != nullptr);
            CHECK(*value == i);
        }
    }
    map.release();
}

TEST_CASE("map reuses removed slots without growing forever") {
    Map<uint32_t, int> map = {};
    for (int round = 0; round < 1000; round++) {
        map.set((uint32_t)round, round);
        map.remove((uint32_t)round);
    }
    CHECK(map.length == 0);
    CHECK(map.capacity <= 64);
    map.release();
}

TEST_CASE("map supports negative signed keys") {
    Map<int32_t, int> map = {};
    map.set(-5, 1);
    map.set(5, 2);
    CHECK(*map.get(-5) == 1);
    CHECK(*map.get(5) == 2);
    map.release();
}
