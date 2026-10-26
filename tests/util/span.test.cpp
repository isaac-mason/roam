#include "roam/util/array.h"
#include "roam/util/span.h"
#include "tests/harness/test.h"

using namespace roam;

static uint32_t sum(Span<const int> values) {
    uint32_t total = 0;
    for (int value : values) total += (uint32_t)value;
    return total;
}

static void fill(Span<int> values) {
    for (uint32_t i = 0; i < values.length; i++) values[i] = (int)i + 1;
}

TEST_CASE("span comes from arrays and c arrays") {
    int fixed[3];
    fill(fixed);
    CHECK(fixed[2] == 3);
    CHECK(sum(fixed) == 6);

    Array<int> values = {};
    values.reserve(16);
    values.resize(4);
    Span<int> written = values;
    CHECK(written.length == 16);
    Span<const int> elements = values;
    CHECK(elements.length == 4);
    fill(values);
    CHECK(sum(values) == 10);

    const Array<int> &read_only = values;
    CHECK(sum(read_only) == 10);
    CHECK(sum(elements) == 10);
    CHECK(sum({fixed, 2}) == 3);
    CHECK(sum({}) == 0);

    values.release();
}
