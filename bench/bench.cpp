#include "bench/bench.h"

#include "roam/util/array.h"
#include "roam/util/sort.h"

#if defined(__APPLE__)
#include <mach/mach_time.h>
#include <time.h>
#elif defined(_WIN32)
#include <windows.h>
#else
#include <time.h>
#endif

using namespace roam;

namespace bench {

static bool use_cpu_time = false;
static int fixed_samples = -1;

void measure_cpu_time(bool enabled) { use_cpu_time = enabled; }

void use_fixed_samples(int samples) { fixed_samples = samples; }

static double cpu_milliseconds() {
#if defined(_WIN32)
    FILETIME creation, exit, kernel, user;
    GetThreadTimes(GetCurrentThread(), &creation, &exit, &kernel, &user);
    ULARGE_INTEGER total;
    total.LowPart = user.dwLowDateTime;
    total.HighPart = user.dwHighDateTime;
    return (double)total.QuadPart / 1e4;
#else
    timespec time;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time);
    return (double)time.tv_sec * 1000.0 + (double)time.tv_nsec / 1e6;
#endif
}

static double clock_milliseconds() { return use_cpu_time ? cpu_milliseconds() : now_milliseconds(); }

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
    double start = clock_milliseconds();
    bench_case.run();
    double elapsed = clock_milliseconds() - start;
    if (bench_case.finish) bench_case.finish();
    return elapsed;
}

BenchResult run(const BenchCase &bench_case, double min_total_milliseconds) {
    const uint32_t WARMUP_SAMPLES = 3;
    const uint32_t MIN_SAMPLES = 10;
    const uint32_t MAX_SAMPLES = 2000;

    Array<double> times = {};
    if (fixed_samples >= 0) {
        for (int i = 0; i < fixed_samples; i++) times.push(sample(bench_case));
        if (times.length == 0) return {0, 0, 0};
        sort::stable(times.data, times.length, [](double a, double b) { return a < b; });
        BenchResult result = {times[times.length / 2], times[0], times.length};
        times.release();
        return result;
    }

    for (uint32_t i = 0; i < WARMUP_SAMPLES; i++) sample(bench_case);

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
