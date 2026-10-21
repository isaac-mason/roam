#include "roam/util/array.h"
#include "roam/util/memory.h"
#include "roam/util/scoped.h"
#include "tests/harness/test.h"

using namespace roam;

static void release_array(Array<int> &array) { array.release(); }

TEST(memory_tracking_counts_outstanding_allocations) {
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

TEST(release_is_safe_to_repeat) {
    Array<int> values = {};
    values.release();
    values.push(3);
    values.release();
    values.release();
    CHECK(values.data == nullptr);
    CHECK(values.length == 0);
}

TEST(scoped_releases_at_end_of_scope) {
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

TEST(scoped_moves_ownership) {
    if (!memory::tracking_enabled()) return;

    memory::MemoryStats before = memory::outstanding();
    {
        Scoped<Array<int>> values = make_values();
        CHECK(values->length == 1);
        CHECK(values.value[0] == 7);
    }
    CHECK(memory::outstanding().allocations == before.allocations);
}
