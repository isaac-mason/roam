#include "bench/bench.h"

#include "roam/util/array.h"
#include "roam/util/sort.h"

#if defined(__APPLE__)
#include <mach/mach_time.h>
#elif defined(_WIN32)
#include <windows.h>
#else
#include <time.h>
#endif

using namespace roam;

namespace bench {

double now_milliseconds() {
#if defined(__APPLE__)
    static mach_timebase_info_data_t timebase = {0, 0};
    if (timebase.denom == 0) mach_timebase_info(&timebase);
    return (double)mach_absolute_time() * timebase.numer / timebase.denom / 1e6;
#elif defined(_WIN32)
    LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart * 1000.0 / (double)frequency.QuadPart;
#else
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (double)time.tv_sec * 1000.0 + (double)time.tv_nsec / 1e6;
#endif
}

static double sample(const BenchCase &bench_case) {
    if (bench_case.prepare) bench_case.prepare();
    double start = now_milliseconds();
    bench_case.run();
    double elapsed = now_milliseconds() - start;
    if (bench_case.finish) bench_case.finish();
    return elapsed;
}

BenchResult run(const BenchCase &bench_case, double min_total_milliseconds) {
    const uint32_t WARMUP_SAMPLES = 3;
    const uint32_t MIN_SAMPLES = 10;
    const uint32_t MAX_SAMPLES = 2000;

    for (uint32_t i = 0; i < WARMUP_SAMPLES; i++) sample(bench_case);

    Array<double> times = {};
    double total = 0;
    while (times.length < MAX_SAMPLES && (times.length < MIN_SAMPLES || total < min_total_milliseconds)) {
        double elapsed = sample(bench_case);
        times.push(elapsed);
        total += elapsed;
    }

    sort::stable(times.data, times.length, [](double a, double b) { return a < b; });
    BenchResult result = {times[times.length / 2], times[0], times.length};
    times.release();
    return result;
}

}
