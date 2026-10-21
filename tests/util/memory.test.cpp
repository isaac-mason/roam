#include "roam/util/array.h"
#include "roam/util/memory.h"
#include "roam/util/scoped.h"
#include "tests/harness/test.h"

using namespace roam;

static void release_array(Array<int> &array) { array.release(); }

TEST_CASE("memory tracking counts outstanding allocations") {
    if (!memory::tracking_enabled()) return;

    memory::MemoryStats before = memory::outstanding();
    Array<int> values = {};
    values.push(1);
    memory::MemoryStats during = memory::outstanding();
    CHECK(during.allocations == before.allocations + 1);
    CHECK(during.bytes > before.bytes);

    values.release();
    memory::MemoryStats after = memory::outstanding();
    CHECK(after.allocations == before.allocations);
    CHECK(after.bytes == before.bytes);
}

TEST_CASE("release is safe to repeat") {
    Array<int> values = {};
    values.release();
    values.push(3);
    values.release();
    values.release();
    CHECK(values.data == nullptr);
    CHECK(values.length == 0);
}

TEST_CASE("scoped releases at end of scope") {
    if (!memory::tracking_enabled()) return;

    memory::MemoryStats before = memory::outstanding();
    {
        Scoped<Array<int>> values{release_array};
        values->push(1);
        values->push(2);
        CHECK((*values).length == 2);
        CHECK(memory::outstanding().allocations == before.allocations + 1);
    }
    CHECK(memory::outstanding().allocations == before.allocations);
}

static Scoped<Array<int>> make_values() {
    Scoped<Array<int>> values{release_array};
    values->push(7);
    return values;
}

TEST_CASE("scoped moves ownership") {
    if (!memory::tracking_enabled()) return;

    memory::MemoryStats before = memory::outstanding();
    {
        Scoped<Array<int>> values = make_values();
        CHECK(values->length == 1);
        CHECK(values.value[0] == 7);
    }
    CHECK(memory::outstanding().allocations == before.allocations);
}
