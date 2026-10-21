#define DOCTEST_CONFIG_IMPLEMENT
#include "tests/harness/test.h"

namespace test {

int leak_count();

}

int main(int argument_count, char **arguments) {
    doctest::Context context(argument_count, arguments);
    int result = context.run();
    if (context.shouldExit()) return result;
    if (result == 0 && test::leak_count() > 0) return 1;
    return result;
}
