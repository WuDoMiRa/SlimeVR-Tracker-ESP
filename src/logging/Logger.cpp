#include "Logger.h"
#include <stdarg.h>

namespace SlimeVR {
    Logger::Logger(HardwareSerial& serial, const char* prefix, const char* category)
        : serial(serial)
        , prefix(prefix)
        , category(category) {}

    void Logger::print(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buf[128];
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        serial.print("[");
        serial.print(prefix);
        serial.print("] ");
        if (category) {
            serial.print("[");
            serial.print(category);
            serial.print("] >> ");
        }
        serial.println(buf);
    }

    void Logger::warn(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buf[128];
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        serial.print("[WARN] ");
        serial.print("[");
        serial.print(prefix);
        serial.print("] ");
        if (category) {
            serial.print("[");
            serial.print(category);
            serial.print("] >> ");
        }
        serial.println(buf);
    }

    void Logger::error(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buf[128];
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        serial.print("[ERROR] ");
        serial.print("[");
        serial.print(prefix);
        serial.print("] ");
        if (category) {
            serial.print("[");
            serial.print(category);
            serial.print("] >> ");
        }
        serial.println(buf);
    }

    void Logger::debug(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buf[128];
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        serial.print("[DEBUG] ");
        serial.print("[");
        serial.print(prefix);
        serial.print("] ");
        if (category) {
            serial.print("[");
            serial.print(category);
            serial.print("] >> ");
        }
        serial.println(buf);
    }

    void Logger::info(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buf[128];
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        serial.print("[INFO] ");
        serial.print("[");
        serial.print(prefix);
        serial.print("] ");
        if (category) {
            serial.print("[");
            serial.print(category);
            serial.print("] >> ");
        }
        serial.println(buf);
    }

    void Logger::logStackTrace() {
        // Implement stack trace logging if needed
    }
}  // namespace SlimeVR