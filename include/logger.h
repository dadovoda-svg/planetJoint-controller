#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

enum class LogLevel : uint8_t {
  OFF   = 0,
  ERROR = 1,
  INFO  = 2,
  DEBUG = 3
};

namespace Logger {

inline LogLevel currentLevel = LogLevel::INFO;

inline void setLevel(uint8_t level)
{
  if (level > static_cast<uint8_t>(LogLevel::DEBUG)) {
    level = static_cast<uint8_t>(LogLevel::DEBUG);
  }

  currentLevel = static_cast<LogLevel>(level);
}

inline uint8_t getLevel()
{
  return static_cast<uint8_t>(currentLevel);
}

inline bool enabled(LogLevel level)
{
  return static_cast<uint8_t>(currentLevel) >= static_cast<uint8_t>(level);
}

inline void vlog(LogLevel level, const char* prefix, const char* fmt, va_list args)
{
  if (!enabled(level)) {
    return;
  }

  // For diagnostic logs only: if USB CDC is not connected, avoid pushing
  // unnecessary data into the Serial path. Command replies and trace output
  // intentionally bypass this logger.
  if (!Serial) {
    return;
  }

  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), fmt, args);

  Serial.print(prefix);
  Serial.print(' ');
  Serial.print(buffer);
}

inline void error(const char* fmt, ...)
{
  if (!enabled(LogLevel::ERROR)) {
    return;
  }

  va_list args;
  va_start(args, fmt);
  vlog(LogLevel::ERROR, "[ERR]", fmt, args);
  va_end(args);
}

inline void info(const char* fmt, ...)
{
  if (!enabled(LogLevel::INFO)) {
    return;
  }

  va_list args;
  va_start(args, fmt);
  vlog(LogLevel::INFO, "[NFO]", fmt, args);
  va_end(args);
}

inline void debug(const char* fmt, ...)
{
  if (!enabled(LogLevel::DEBUG)) {
    return;
  }

  va_list args;
  va_start(args, fmt);
  vlog(LogLevel::DEBUG, "[DBG]", fmt, args);
  va_end(args);
}

} // namespace Logger

#define LOG_ERR(...)  Logger::error(__VA_ARGS__)
#define LOG_NFO(...)  Logger::info(__VA_ARGS__)
#define LOG_DBG(...)  Logger::debug(__VA_ARGS__)
