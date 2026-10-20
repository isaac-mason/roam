#pragma once

#include <stdint.h>

#include "roam/util/array.h"

namespace roam {

struct IndexPool {
    Array<uint32_t> free;
    uint32_t counter;
};

namespace index_pool {

inline uint32_t request(IndexPool &pool) {
    if (pool.free.length > 0) return pool.free.pop();
    return pool.counter++;
}

inline void release_index(IndexPool &pool, uint32_t index) { pool.free.push(index); }

inline void release(IndexPool &pool) {
    pool.free.release();
    pool.counter = 0;
}

}  // namespace index_pool

}  // namespace roam
