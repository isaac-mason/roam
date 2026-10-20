#pragma once

namespace test {

void add(const char *name, const char *file, void (*run)());
void fail(const char *file, int line, const char *message);
void fail_near(const char *file, int line, const char *expression, double actual, double expected, double tolerance);

struct Registration {
    Registration(const char *name, const char *file, void (*run)()) { add(name, file, run); }
};

int run_all(const char *filter);

}

#define TEST(name)                                                         \
    static void name();                                                    \
    static test::Registration name##_registration(#name, __FILE__, name); \
    static void name()

#define CHECK(expression)                                                     \
    do {                                                                      \
        if (!(expression)) test::fail(__FILE__, __LINE__, #expression);      \
    } while (0)

#define REQUIRE(expression)                                                   \
    do {                                                                      \
        if (!(expression)) {                                                  \
            test::fail(__FILE__, __LINE__, #expression);                     \
            return;                                                           \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                                                         \
    do {                                                                                                                \
        double test_actual = (double)(actual);                                                                          \
        double test_expected = (double)(expected);                                                                      \
        double test_difference = test_actual - test_expected;                                                           \
        if (!(test_difference <= (tolerance) && -test_difference <= (tolerance)))                                       \
            test::fail_near(__FILE__, __LINE__, #actual, test_actual, test_expected, (tolerance));                      \
    } while (0)
