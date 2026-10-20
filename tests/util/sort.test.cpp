#include "roam/util/sort.h"
#include "tests/harness/test.h"

using namespace roam;

struct Keyed {
    int key;
    int order;
};

TEST(sort_stable_orders_by_key) {
    int values[200];
    for (int i = 0; i < 200; i++) values[i] = (i * 7919) % 200;
    sort::stable(values, 200, [](int a, int b) { return a < b; });
    for (int i = 1; i < 200; i++) CHECK(values[i - 1] <= values[i]);
}

TEST(sort_stable_keeps_equal_keys_in_order) {
    Keyed items[300];
    for (int i = 0; i < 300; i++) items[i] = {(i * 31) % 5, i};
    sort::stable(items, 300, [](const Keyed &a, const Keyed &b) { return a.key < b.key; });
    for (int i = 1; i < 300; i++) {
        CHECK(items[i - 1].key <= items[i].key);
        if (items[i - 1].key == items[i].key) CHECK(items[i - 1].order < items[i].order);
    }
}

TEST(sort_stable_handles_tiny_inputs) {
    int empty[1] = {0};
    sort::stable(empty, 0, [](int a, int b) { return a < b; });
    int pair[2] = {2, 1};
    sort::stable(pair, 2, [](int a, int b) { return a < b; });
    CHECK(pair[0] == 1);
    CHECK(pair[1] == 2);
}
