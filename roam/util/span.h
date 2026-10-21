#pragma once

#include <stddef.h>
#include <stdint.h>

#include "roam/util/array.h"

namespace roam {

template <typename T>
struct Span {
    T *data;
    uint32_t capacity;

    Span() : data(nullptr), capacity(0) {}
    Span(T *data, uint32_t capacity) : data(data), capacity(capacity) {}

    template <size_t N>
    Span(T (&array)[N]) : data(array), capacity((uint32_t)N) {}

    Span(Array<T> &array) : data(array.data), capacity(array.length) {}

    T &operator[](uint32_t index) const {
        ROAM_ASSERT(index < capacity);
        return data[index];
    }

    T *begin() const { return data; }
    T *end() const { return data + capacity; }
};

}
