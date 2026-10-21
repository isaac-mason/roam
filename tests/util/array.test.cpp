#include "roam/util/array.h"
#include "tests/harness/test.h"

using namespace roam;

TEST_CASE("array starts empty when zeroed") {
    Array<int> array = {};
    CHECK(array.length == 0);
    CHECK(array.capacity == 0);
    CHECK(array.data == nullptr);
    array.release();
}

TEST_CASE("array push grows and keeps values") {
    Array<int> array = {};
    for (int i = 0; i < 100; i++) array.push(i * 3);
    REQUIRE(array.length == 100);
    for (uint32_t i = 0; i < array.length; i++) CHECK(array[i] == (int)i * 3);
    array.release();
}

TEST_CASE("array push of own element survives growth") {
    Array<int> array = {};
    for (int i = 0; i < 8; i++) array.push(i);
    REQUIRE(array.length == array.capacity);
    array.push(array[0]);
    CHECK(array.back() == 0);
    array.release();
}

TEST_CASE("array pop returns last") {
    Array<int> array = {};
    array.push(1);
    array.push(2);
    CHECK(array.pop() == 2);
    CHECK(array.length == 1);
    array.release();
}

TEST_CASE("array resize fills new elements") {
    Array<int> array = {};
    array.push(5);
    array.resize(4, -1);
    REQUIRE(array.length == 4);
    CHECK(array[0] == 5);
    CHECK(array[3] == -1);
    array.release();
}

TEST_CASE("array remove swap moves last into place") {
    Array<int> array = {};
    for (int i = 0; i < 4; i++) array.push(i);
    array.remove_swap(1);
    REQUIRE(array.length == 3);
    CHECK(array[1] == 3);
    array.release();
}

TEST_CASE("array iterates with range for") {
    Array<int> array = {};
    for (int i = 1; i <= 4; i++) array.push(i);
    int sum = 0;
    for (int value : array) sum += value;
    CHECK(sum == 10);
    array.release();
}
