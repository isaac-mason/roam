#include "tests/harness/test.h"

#include <stdio.h>
#include <string.h>

#include "roam/util/memory.h"

namespace test {

struct Case {
    const char *name;
    const char *file;
    void (*run)();
};

static const int MAX_CASES = 4096;
static Case cases[MAX_CASES];
static int case_count = 0;
static int current_failures = 0;
static const char *current_name = nullptr;

void add(const char *name, const char *file, void (*run)()) {
    if (case_count == MAX_CASES) {
        fprintf(stderr, "too many tests, raise MAX_CASES\n");
        return;
    }
    cases[case_count++] = {name, file, run};
}

void fail(const char *file, int line, const char *message) {
    if (current_failures == 0) printf("FAIL %s\n", current_name);
    printf("  %s:%d: %s\n", file, line, message);
    current_failures++;
}

void fail_near(const char *file, int line, const char *expression, double actual, double expected, double tolerance) {
    char message[512];
    snprintf(message, sizeof(message), "%s is %.9g, expected %.9g within %.3g", expression, actual, expected, tolerance);
    fail(file, line, message);
}

static bool matches(const Case &test_case, const char *filter) {
    if (filter == nullptr) return true;
    return strstr(test_case.name, filter) != nullptr || strstr(test_case.file, filter) != nullptr;
}

int run_all(const char *filter) {
    int ran = 0;
    int failed = 0;
    for (int i = 0; i < case_count; i++) {
        if (!matches(cases[i], filter)) continue;
        current_name = cases[i].name;
        current_failures = 0;
        roam::memory::MemoryStats before = roam::memory::outstanding();
        cases[i].run();
        roam::memory::MemoryStats after = roam::memory::outstanding();
        if (current_failures == 0 && (after.allocations != before.allocations || after.bytes != before.bytes)) {
            char message[256];
            snprintf(message, sizeof(message), "leaked %lld allocations (%lld bytes)", (long long)(after.allocations - before.allocations),
                     (long long)(after.bytes - before.bytes));
            fail(cases[i].file, 0, message);
        }
        ran++;
        if (current_failures > 0) failed++;
    }
    printf("%d passed, %d failed\n", ran - failed, failed);
    return failed == 0 ? 0 : 1;
}

}
