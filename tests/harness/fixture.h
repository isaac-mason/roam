#pragma once

#include <stdint.h>

#include "roam/util/array.h"

namespace fixture {

enum ValueType : uint32_t {
    VALUE_TYPE_F64 = 0,
    VALUE_TYPE_I32 = 1,
};

struct Entry {
    char name[128];
    ValueType type;
    uint32_t count;
    void *data;
};

struct Fixture {
    roam::Array<Entry> entries;
    bool loaded;
};

struct F64 {
    const double *data;
    uint32_t count;
};

struct I32 {
    const int32_t *data;
    uint32_t count;
};

Fixture load(const char *name);
void release(Fixture &fixture);

F64 f64(const Fixture &fixture, const char *name);
I32 i32(const Fixture &fixture, const char *name);
double number(const Fixture &fixture, const char *name);

}  // namespace fixture
