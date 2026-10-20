#pragma once

#include <stdint.h>
#include <string.h>

#include "roam/util/assert.h"
#include "roam/util/memory.h"

namespace roam {

template <typename T>
struct Array {
    static_assert(__is_trivially_copyable(T));

    T *data;
    uint32_t length;
    uint32_t capacity;

    T &operator[](uint32_t index) {
        ROAM_ASSERT(index < length);
        return data[index];
    }

    const T &operator[](uint32_t index) const {
        ROAM_ASSERT(index < length);
        return data[index];
    }

    T *begin() { return data; }
    T *end() { return data + length; }
    const T *begin() const { return data; }
    const T *end() const { return data + length; }

    T &back() {
        ROAM_ASSERT(length > 0);
        return data[length - 1];
    }

    void reserve(uint32_t wanted) {
        if (wanted <= capacity) return;
        T *grown = memory::allocate_array<T>(wanted);
        if (length > 0) memcpy(grown, data, length * sizeof(T));
        memory::free_array(data, capacity);
        data = grown;
        capacity = wanted;
    }

    void push(const T &value) {
        if (length == capacity) {
            T copy = value;
            reserve(capacity == 0 ? 8 : capacity * 2);
            data[length++] = copy;
            return;
        }
        data[length++] = value;
    }

    T pop() {
        ROAM_ASSERT(length > 0);
        return data[--length];
    }

    void resize(uint32_t new_length) {
        reserve(new_length);
        length = new_length;
    }

    void resize(uint32_t new_length, const T &fill) {
        reserve(new_length);
        for (uint32_t i = length; i < new_length; i++) data[i] = fill;
        length = new_length;
    }

    void remove_swap(uint32_t index) {
        ROAM_ASSERT(index < length);
        data[index] = data[--length];
    }

    void clear() { length = 0; }

    void release() {
        memory::free_array(data, capacity);
        data = nullptr;
        length = 0;
        capacity = 0;
    }
};

}
