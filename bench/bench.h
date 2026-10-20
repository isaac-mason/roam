#pragma once

#include <stdint.h>

struct BenchCase {
    const char *name;
    void (*prepare)();
    void (*run)();
    void (*finish)();
};

struct BenchResult {
    double median_milliseconds;
    double min_milliseconds;
    uint32_t samples;
};

namespace bench {

double now_milliseconds();
BenchResult run(const BenchCase &bench_case, double min_total_milliseconds = 500);

}
