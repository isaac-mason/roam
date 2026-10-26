#include "tests/harness/test.h"

#include <stdio.h>

#include "roam/util/memory.h"

namespace test {

static int leaking_tests = 0;

void fail(const char *file, int line, const char *message) { ADD_FAIL_CHECK_AT(file, line, doctest::String(message)); }

int leak_count() { return leaking_tests; }

struct LeakListener : doctest::IReporter {
    roam::MemoryStats before = {};
    const doctest::TestCaseData *test_case = nullptr;

    LeakListener(const doctest::ContextOptions &) {}

    void report_query(const doctest::QueryData &) override {}
    void test_run_start() override {}
    void test_run_end(const doctest::TestRunStats &) override {}
    void test_case_start(const doctest::TestCaseData &data) override {
        test_case = &data;
        before = roam::memory::outstanding();
    }
    void test_case_reenter(const doctest::TestCaseData &) override {}
    void test_case_end(const doctest::CurrentTestCaseStats &stats) override {
        roam::MemoryStats after = roam::memory::outstanding();
        if (stats.failure_flags != 0) return;
        if (after.allocations == before.allocations && after.bytes == before.bytes) return;
        printf("%s:%d: %s leaked %lld allocations (%lld bytes)\n", test_case->m_file.c_str(), test_case->m_line, test_case->m_name,
               (long long)(after.allocations - before.allocations), (long long)(after.bytes - before.bytes));
        leaking_tests++;
    }
    void test_case_exception(const doctest::TestCaseException &) override {}
    void subcase_start(const doctest::SubcaseSignature &) override {}
    void subcase_end() override {}
    void log_assert(const doctest::AssertData &) override {}
    void log_message(const doctest::MessageData &) override {}
    void test_case_skipped(const doctest::TestCaseData &) override {}
};

DOCTEST_REGISTER_LISTENER("leaks", 1, LeakListener);

}
