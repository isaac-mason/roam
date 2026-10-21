#include "roam/util/sort.h"
#include "tests/harness/test.h"

using namespace roam;

struct Keyed {
    int key;
    int order;
};

TEST_CASE("sort stable orders by key") {
    int values[200];
    for (int i = 0; i < 200; i++) values[i] = (i * 7919) % 200;
    sort::stable(values, 200, [](int a, int b) { return a < b; });
    for (int i = 1; i < 200; i++) CHECK(values[i - 1] <= values[i]);
}

TEST_CASE("sort stable keeps equal keys in order") {
    Keyed items[300];
    for (int i = 0; i < 300; i++) items[i] = {(i * 31) % 5, i};
    sort::stable(items, 300, [](const Keyed &a, const Keyed &b) { return a.key < b.key; });
    for (int i = 1; i < 300; i++) {
        CHECK(items[i - 1].key <= items[i].key);
        if (items[i - 1].key == items[i].key) CHECK(items[i - 1].order < items[i].order);
    }
}

TEST_CASE("sort stable handles tiny inputs") {
    int empty[1] = {0};
    sort::stable(empty, 0, [](int a, int b) { return a < b; });
    int pair[2] = {2, 1};
    sort::stable(pair, 2, [](int a, int b) { return a < b; });
    CHECK(pair[0] == 1);
    CHECK(pair[1] == 2);
}
