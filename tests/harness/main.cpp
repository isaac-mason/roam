#include "tests/harness/test.h"

int main(int argument_count, char **arguments) { return test::run_all(argument_count > 1 ? arguments[1] : nullptr); }
