#pragma once

namespace roam {

template <typename T>
struct Scoped {
    T value;
    void (*release_value)(T &value);

    explicit Scoped(void (*release_function)(T &value)) : value{}, release_value(release_function) {}

    Scoped(const Scoped &) = delete;
    Scoped &operator=(const Scoped &) = delete;

    Scoped(Scoped &&other) : value(other.value), release_value(other.release_value) {
        other.value = {};
        other.release_value = nullptr;
    }

    Scoped &operator=(Scoped &&other) = delete;

    ~Scoped() {
        if (release_value) release_value(value);
    }

    T &operator*() { return value; }
    const T &operator*() const { return value; }
    T *operator->() { return &value; }
    const T *operator->() const { return &value; }
};

}
