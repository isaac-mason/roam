#pragma once

#include <stddef.h>
#include <stdint.h>

namespace roam {

namespace memory {

struct Allocator {
    void *(*allocate)(size_t size, void *user_data);
    void (*free)(void *block, size_t size, void *user_data);
    void *user_data;
};

struct MemoryStats {
    int64_t allocations;
    int64_t bytes;
};

void set_allocator(const Allocator &allocator);
void reset_allocator();

void *allocate(size_t size);
void free(void *block, size_t size);

bool tracking_enabled();
MemoryStats outstanding();

template <typename T>
T *allocate_array(size_t count) {
    return static_cast<T *>(allocate(count * sizeof(T)));
}

template <typename T>
void free_array(T *block, size_t count) {
    free(block, count * sizeof(T));
}

}

}
