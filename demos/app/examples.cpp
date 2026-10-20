#include "app/examples.h"

const Example *const EXAMPLES[] = {
    &EXAMPLE_SOLO_NAV_MESH,
    &EXAMPLE_TILED_NAV_MESH,
    &EXAMPLE_FIND_PATH,
    &EXAMPLE_CROWD_SIMULATION,
};

const int EXAMPLE_COUNT = (int)(sizeof(EXAMPLES) / sizeof(EXAMPLES[0]));
