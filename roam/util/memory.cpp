#include "roam/util/memory.h"

#include <stdio.h>
#include <stdlib.h>

#if !defined(NDEBUG) || defined(ROAM_TRACK_MEMORY)
#define ROAM_MEMORY_TRACKING 1
#else
#define ROAM_MEMORY_TRACKING 0
#endif

#if ROAM_MEMORY_TRACKING && defined(_MSC_VER)
#include <intrin.h>
#endif

namespace roam {

namespace memory {

static void *system_allocate(size_t size, void *) { return malloc(size); }

static void system_free(void *block, size_t, void *) { ::free(block); }

static Allocator current = {system_allocate, system_free, nullptr};

#if ROAM_MEMORY_TRACKING
static volatile int64_t outstanding_allocations = 0;
static volatile int64_t outstanding_bytes = 0;

static void track(int64_t allocations, int64_t bytes) {
#if defined(_MSC_VER)
    _InterlockedExchangeAdd64(&outstanding_allocations, allocations);
    _InterlockedExchangeAdd64(&outstanding_bytes, bytes);
#else
    __atomic_fetch_add(&outstanding_allocations, allocations, __ATOMIC_RELAXED);
    __atomic_fetch_add(&outstanding_bytes, bytes, __ATOMIC_RELAXED);
#endif
}
#endif

void set_allocator(const Allocator &allocator) { current = allocator; }

void reset_allocator() { current = {system_allocate, system_free, nullptr}; }

void *allocate(size_t size) {
    if (size == 0) return nullptr;
    void *block = current.allocate(size, current.user_data);
    if (block == nullptr) {
        fprintf(stderr, "roam: out of memory allocating %zu bytes\n", size);
        abort();
    }
#if ROAM_MEMORY_TRACKING
    track(1, (int64_t)size);
#endif
    return block;
}

void free(void *block, size_t size) {
    if (block == nullptr) return;
#if ROAM_MEMORY_TRACKING
    track(-1, -(int64_t)size);
#endif
    current.free(block, size, current.user_data);
}

bool tracking_enabled() { return ROAM_MEMORY_TRACKING != 0; }

MemoryStats outstanding() {
#if ROAM_MEMORY_TRACKING
#if defined(_MSC_VER)
    return {_InterlockedExchangeAdd64(&outstanding_allocations, 0), _InterlockedExchangeAdd64(&outstanding_bytes, 0)};
#else
    return {__atomic_load_n(&outstanding_allocations, __ATOMIC_RELAXED), __atomic_load_n(&outstanding_bytes, __ATOMIC_RELAXED)};
#endif
#else
    return {0, 0};
#endif
}

}

}
