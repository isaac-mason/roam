#include "roam/generate/build_context.h"

#include <stdarg.h>
#include <stdio.h>

namespace roam {

namespace build_context {

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
}

}

}
