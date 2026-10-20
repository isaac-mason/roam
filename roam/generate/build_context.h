#pragma once

#include "roam/util/array.h"

namespace roam {

enum BuildContextLogType : uint32_t {
    BUILD_CONTEXT_LOG_TYPE_INFO = 0,
    BUILD_CONTEXT_LOG_TYPE_WARNING = 1,
    BUILD_CONTEXT_LOG_TYPE_ERROR = 2,
};

struct BuildContextLog {
    BuildContextLogType type;
    char message[256];
};

struct BuildContextTime {
    const char *name;
    double duration;
};

struct BuildContextStart {
    const char *name;
    double time;
};

struct BuildContext {
    Array<BuildContextLog> logs;
    Array<BuildContextTime> times;
    Array<BuildContextStart> starts;
};

namespace build_context {

void start(BuildContext &context, const char *name);
void end(BuildContext &context, const char *name);

void info(BuildContext &context, const char *format, ...);
void warn(BuildContext &context, const char *format, ...);
void error(BuildContext &context, const char *format, ...);

void release(BuildContext &context);

}

}
