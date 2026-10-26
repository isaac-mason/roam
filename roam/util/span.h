#pragma once

#include <stddef.h>
#include <stdint.h>

#include "roam/util/array.h"

namespace roam {

template <typename T>
struct WithoutConst {
    using Type = T;
};

template <typename T>
struct WithoutConst<const T> {
    using Type = T;
};

template <typename T>
constexpr bool IS_CONST = false;

template <typename T>
constexpr bool IS_CONST<const T> = true;

template <typename A, typename B>
constexpr bool IS_SAME = false;

template <typename A>
constexpr bool IS_SAME<A, A> = true;

template <typename T>
struct Span {
    using Element = typename WithoutConst<T>::Type;

    T *data;
    uint32_t length;

    Span() : data(nullptr), length(0) {}
    Span(T *data, uint32_t length) : data(data), length(length) {}

    template <size_t N>
    Span(T (&array)[N]) : data(array), length((uint32_t)N) {}

    Span(Array<Element> &array)
        requires(!IS_CONST<T>)
        : data(array.data), length(array.capacity) {}

    Span(const Array<Element> &array)
        requires IS_CONST<T>
        : data(array.data), length(array.length) {}

    template <typename U>
        requires IS_CONST<T> && IS_SAME<U, Element>
    Span(const Span<U> &span) : data(span.data), length(span.length) {}

    T &operator[](uint32_t index) const {
        ROAM_ASSERT(index < length);
        return data[index];
    }

    T *begin() const { return data; }
    T *end() const { return data + length; }
};

}
