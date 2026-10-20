#pragma once

#include <stdint.h>

#include "roam/util/memory.h"

namespace roam {

namespace sort {

template <typename T, typename Less>
void merge_runs(T *items, T *scratch, uint32_t start, uint32_t middle, uint32_t end, Less less) {
    uint32_t left = start;
    uint32_t right = middle;
    uint32_t out = start;
    while (left < middle && right < end) {
        if (less(items[right], items[left])) {
            scratch[out++] = items[right++];
        } else {
            scratch[out++] = items[left++];
        }
    }
    while (left < middle) scratch[out++] = items[left++];
    while (right < end) scratch[out++] = items[right++];
    for (uint32_t i = start; i < end; i++) items[i] = scratch[i];
}

template <typename T, typename Less>
void insertion(T *items, uint32_t start, uint32_t end, Less less) {
    for (uint32_t i = start + 1; i < end; i++) {
        T item = items[i];
        uint32_t j = i;
        while (j > start && less(item, items[j - 1])) {
            items[j] = items[j - 1];
            j--;
        }
        items[j] = item;
    }
}

template <typename T, typename Less>
void stable(T *items, uint32_t count, Less less) {
    const uint32_t run = 16;
    for (uint32_t start = 0; start < count; start += run) {
        uint32_t end = start + run < count ? start + run : count;
        insertion(items, start, end, less);
    }
    if (count <= run) return;

    T *scratch = memory::allocate_array<T>(count);
    for (uint32_t width = run; width < count; width *= 2) {
        for (uint32_t start = 0; start < count; start += width * 2) {
            uint32_t middle = start + width;
            if (middle >= count) break;
            uint32_t end = middle + width < count ? middle + width : count;
            merge_runs(items, scratch, start, middle, end, less);
        }
    }
    memory::free_array(scratch, count);
}

}  // namespace sort

}  // namespace roam
