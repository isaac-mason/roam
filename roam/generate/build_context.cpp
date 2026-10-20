#include "roam/generate/build_context.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "roam/util/clock.h"

namespace roam {

namespace build_context {

void start(BuildContext &context, const char *name) { context.starts.push({name, clock::now_milliseconds()}); }

void end(BuildContext &context, const char *name) {
    double now = clock::now_milliseconds();
    for (uint32_t i = context.starts.length; i-- > 0;) {
        if (strcmp(context.starts[i].name, name) != 0) continue;
        context.times.push({name, now - context.starts[i].time});
        context.starts.remove_swap(i);
        return;
    }
}

static void log(BuildContext &context, BuildContextLogType type, const char *format, va_list arguments) {
    BuildContextLog entry;
    entry.type = type;
    vsnprintf(entry.message, sizeof(entry.message), format, arguments);
    context.logs.push(entry);
}

void info(BuildContext &context, const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    log(context, BUILD_CONTEXT_LOG_TYPE_INFO, format, arguments);
    va_end(arguments);
}

void warn(BuildContext &context, const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    log(context, BUILD_CONTEXT_LOG_TYPE_WARNING, format, arguments);
    va_end(arguments);
}

void error(BuildContext &context, const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    log(context, BUILD_CONTEXT_LOG_TYPE_ERROR, format, arguments);
    va_end(arguments);
}

void release(BuildContext &context) {
    context.logs.release();
    context.times.release();
    context.starts.release();
}

}

}
