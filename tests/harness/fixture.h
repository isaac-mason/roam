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

void check_i32(const char *file, int line, const char *label, const int32_t *actual, uint32_t actual_count, I32 expected);
void check_f64(const char *file, int line, const char *label, const float *actual, uint32_t actual_count, F64 expected, double tolerance);

}

#define CHECK_I32_ARRAY(label, actual, actual_count, expected) fixture::check_i32(__FILE__, __LINE__, label, actual, actual_count, expected)
#define CHECK_F64_ARRAY(label, actual, actual_count, expected, tolerance) \
    fixture::check_f64(__FILE__, __LINE__, label, actual, actual_count, expected, tolerance)
