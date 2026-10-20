#pragma once

#include "roam/util/array.h"
#include "tests/harness/fixture.h"

struct NavTestInput {
    roam::Array<float> positions;
    roam::Array<uint32_t> indices;
};

inline NavTestInput load_nav_test_input(const fixture::Fixture &nav_test) {
    NavTestInput input = {};
    fixture::F64 positions = fixture::f64(nav_test, "input.positions");
    fixture::I32 indices = fixture::i32(nav_test, "input.indices");
    for (uint32_t i = 0; i < positions.count; i++) input.positions.push((float)positions.data[i]);
    for (uint32_t i = 0; i < indices.count; i++) input.indices.push((uint32_t)indices.data[i]);
    return input;
}

inline void release(NavTestInput &input) {
    input.positions.release();
    input.indices.release();
}
