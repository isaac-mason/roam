#include "roam/util/memory.h"

#include <stdio.h>
#include <stdlib.h>

namespace roam {

namespace memory {

static void *system_allocate(size_t size, void *) { return malloc(size); }

static void system_free(void *block, size_t, void *) { ::free(block); }

static Allocator current = {system_allocate, system_free, nullptr};

void set_allocator(const Allocator &allocator) { current = allocator; }

void reset_allocator() { current = {system_allocate, system_free, nullptr}; }

void *allocate(size_t size) {
    if (size == 0) return nullptr;
    void *block = current.allocate(size, current.user_data);
    if (block == nullptr) {
        fprintf(stderr, "roam: out of memory allocating %zu bytes\n", size);
        abort();
    }
    return block;
}

void free(void *block, size_t size) {
    if (block == nullptr) return;
    current.free(block, size, current.user_data);
}

}  // namespace memory

}  // namespace roam
