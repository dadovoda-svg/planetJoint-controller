#include "logger.h"

#include <cstdio>

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); \
    ++failures; \
  } \
} while (0)

void checkLevel(uint8_t level,
                bool errorEnabled,
                bool infoEnabled,
                bool debugEnabled)
{
  Logger::setLevel(level);
  CHECK(Logger::getLevel() == level);
  CHECK(Logger::enabled(LogLevel::ERROR) == errorEnabled);
  CHECK(Logger::enabled(LogLevel::INFO) == infoEnabled);
  CHECK(Logger::enabled(LogLevel::DEBUG) == debugEnabled);
}

} // namespace

int main()
{
  checkLevel(0, false, false, false);
  checkLevel(1, true, false, false);
  checkLevel(2, true, true, false);
  checkLevel(3, true, true, true);

  Logger::setLevel(255);
  CHECK(Logger::getLevel() == 3);

  if (failures != 0) {
    std::printf("logger tests failed: %d\n", failures);
    return 1;
  }

  std::printf("logger tests passed\n");
  return 0;
}
