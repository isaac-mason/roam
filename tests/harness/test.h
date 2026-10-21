#pragma once

#include <doctest/doctest.h>

namespace test {

void fail(const char *file, int line, const char *message);

}

#define CHECK_NEAR(actual, expected, tolerance)                                                                   \
    do {                                                                                                          \
        double test_actual = (double)(actual);                                                                    \
        double test_expected = (double)(expected);                                                                \
        double test_tolerance = (double)(tolerance);                                                              \
        INFO(#actual " is ", test_actual, ", expected ", test_expected, " within ", test_tolerance);              \
        CHECK((test_actual - test_expected <= test_tolerance && test_expected - test_actual <= test_tolerance)); \
    } while (0)
