#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

class Logger {
public:
    Logger(HardwareSerial& serial, const char* prefix, const char* category = nullptr);

    void print(const char* format, ...);
    void warn(const char* format, ...);
    void error(const char* format, ...);
    void debug(const char* format, ...);
    void info(const char* format, ...);

private:
    HardwareSerial& serial;
    const char* prefix;
    const char* category;

    void logStackTrace();
};

#endif // LOGGER_H