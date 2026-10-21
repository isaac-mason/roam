#pragma once

#include <stdint.h>

namespace test {

struct Random {
    uint64_t state;
};

inline uint32_t next_u32(Random &random) {
    random.state += 0x9e3779b97f4a7c15ull;
    uint64_t z = random.state;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return (uint32_t)((z ^ (z >> 31)) >> 32);
}

inline uint64_t next_u64(Random &random) { return ((uint64_t)next_u32(random) << 32) | next_u32(random); }

inline uint32_t next_below(Random &random, uint32_t count) { return count == 0 ? 0 : next_u32(random) % count; }

inline float next_float(Random &random, float min, float max) { return min + (max - min) * ((float)(next_u32(random) >> 8) / 16777216.0f); }

inline bool one_in(Random &random, uint32_t count) { return next_below(random, count) == 0; }

}
