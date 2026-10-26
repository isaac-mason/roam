#pragma once

#include <stddef.h>
#include <stdint.h>

namespace roam {

struct Allocator {
    void *(*allocate)(size_t size, size_t alignment, void *user_data);
    void (*free)(void *block, size_t size, size_t alignment, void *user_data);
    void *user_data;
};

struct MemoryStats {
    int64_t allocations;
    int64_t bytes;
    int64_t total_allocations;
};

namespace memory {

constexpr size_t DEFAULT_ALIGNMENT = alignof(max_align_t);

void set_allocator(const Allocator &allocator);
void reset_allocator();

void *allocate(size_t size, size_t alignment = DEFAULT_ALIGNMENT);
void free(void *block, size_t size, size_t alignment = DEFAULT_ALIGNMENT);

bool tracking_enabled();
MemoryStats outstanding();

template <typename T>
T *allocate_array(size_t count) {
    return static_cast<T *>(allocate(count * sizeof(T), alignof(T)));
}

template <typename T>
void free_array(T *block, size_t count) {
    free(block, count * sizeof(T), alignof(T));
}

}

}
