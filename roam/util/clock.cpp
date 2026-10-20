#include "roam/util/clock.h"

#if defined(_WIN32)
#include <windows.h>
#else
#include <time.h>
#endif

namespace roam {

namespace clock {

#if defined(_WIN32)

double now_milliseconds() {
    LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart * 1000.0 / (double)frequency.QuadPart;
}

#else

double now_milliseconds() {
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (double)time.tv_sec * 1000.0 + (double)time.tv_nsec / 1000000.0;
}

#endif

}

}
