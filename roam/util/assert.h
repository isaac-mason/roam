#pragma once

#include <stdio.h>
#include <stdlib.h>

#if defined(NDEBUG) && !defined(ROAM_ENABLE_ASSERTS)
#define ROAM_ASSERT(condition) ((void)sizeof(!(condition)))
#else
#define ROAM_ASSERT(condition)                                                              \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__, #condition); \
            abort();                                                                        \
        }                                                                                   \
    } while (0)
#endif
