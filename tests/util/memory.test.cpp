#include <stdlib.h>

#include "roam/util/array.h"
#include "roam/util/memory.h"
#include "roam/util/scoped.h"
#include "tests/harness/test.h"

using namespace roam;

static void release_array(Array<int> &array) { array.release(); }

TEST_CASE("memory tracking counts outstanding allocations") {
    if (!memory::tracking_enabled()) return;

    MemoryStats before = memory::outstanding();
    Array<int> values = {};
    values.push(1);
    MemoryStats during = memory::outstanding();
    CHECK(during.allocations == before.allocations + 1);
    CHECK(during.bytes > before.bytes);

    values.release();
    MemoryStats after = memory::outstanding();
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

    MemoryStats before = memory::outstanding();
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

    MemoryStats before = memory::outstanding();
    {
        Scoped<Array<int>> values = make_values();
        CHECK(values->length == 1);
        CHECK(values.value[0] == 7);
    }
    CHECK(memory::outstanding().allocations == before.allocations);
}

struct RecordingAllocator {
    size_t last_allocate_alignment;
    size_t last_free_alignment;
    int64_t live;
};

static void *recording_allocate(size_t size, size_t alignment, void *user_data) {
    RecordingAllocator &recorder = *static_cast<RecordingAllocator *>(user_data);
    recorder.last_allocate_alignment = alignment;
    recorder.live++;
    return malloc(size);
}

static void recording_free(void *block, size_t, size_t alignment, void *user_data) {
    RecordingAllocator &recorder = *static_cast<RecordingAllocator *>(user_data);
    recorder.last_free_alignment = alignment;
    recorder.live--;
    free(block);
}

TEST_CASE("a custom allocator is told each block's alignment") {
    RecordingAllocator recorder = {};
    memory::set_allocator({recording_allocate, recording_free, &recorder});

    Array<double> doubles = {};
    doubles.push(1);
    CHECK(recorder.last_allocate_alignment == alignof(double));
    doubles.release();
    CHECK(recorder.last_free_alignment == alignof(double));

    Array<uint8_t> bytes = {};
    bytes.push(1);
    CHECK(recorder.last_allocate_alignment == alignof(uint8_t));
    bytes.release();

    void *raw = memory::allocate(24);
    CHECK(recorder.last_allocate_alignment == memory::DEFAULT_ALIGNMENT);
    memory::free(raw, 24);
    CHECK(recorder.last_free_alignment == memory::DEFAULT_ALIGNMENT);
    CHECK(recorder.live == 0);

    memory::reset_allocator();
}
