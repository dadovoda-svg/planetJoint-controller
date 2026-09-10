#pragma once

#include <stdint.h>

namespace HobbyServoMotion {

enum class MoveResult : uint8_t {
  Accepted,
  Disabled,
  InvalidArgument,
  InvalidConfig,
  PwmError
};

static constexpr uint32_t PWM_FREQUENCY_HZ = 50U;
static constexpr uint32_t PWM_PERIOD_US = 1000000U / PWM_FREQUENCY_HZ;
// ESP32-S3 LEDC timers support at most 14-bit duty resolution. Using the
// hardware maximum still gives substantially finer steps than a hobby servo
// can resolve across a typical 1000..2000 us range.
static constexpr uint8_t PWM_RESOLUTION_BITS = 14U;
static constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_RESOLUTION_BITS) - 1UL;
static constexpr uint32_t MAX_FULL_RANGE_DURATION_MS = 5000U;

inline bool validConfig(uint32_t zeroUs, uint32_t minUs, uint32_t maxUs)
{
  return minUs > 0U && minUs < maxUs && maxUs < PWM_PERIOD_US &&
         zeroUs >= minUs && zeroUs <= maxUs;
}

inline uint32_t positionToPulseUs(uint16_t position, uint32_t minUs, uint32_t maxUs)
{
  const uint32_t span = maxUs - minUs;
  return minUs + (span * static_cast<uint32_t>(position) + 499U) / 999U;
}

inline uint32_t pulseUsToDuty(uint32_t pulseUs)
{
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(pulseUs) * PWM_MAX_DUTY + PWM_PERIOD_US / 2U) /
      PWM_PERIOD_US);
}

inline uint32_t moveDurationMs(uint32_t fromUs,
                               uint32_t toUs,
                               uint32_t minUs,
                               uint32_t maxUs,
                               uint8_t speed)
{
  if (speed >= 10U || fromUs == toUs || minUs >= maxUs) {
    return 0U;
  }

  const uint32_t fullRangeMs =
      (MAX_FULL_RANGE_DURATION_MS * static_cast<uint32_t>(10U - speed) + 4U) / 9U;
  const uint32_t distance = fromUs > toUs ? fromUs - toUs : toUs - fromUs;
  const uint32_t span = maxUs - minUs;
  return (fullRangeMs * distance + span / 2U) / span;
}

inline uint32_t interpolatePulseUs(uint32_t fromUs,
                                   uint32_t toUs,
                                   uint32_t elapsedMs,
                                   uint32_t durationMs)
{
  if (durationMs == 0U || elapsedMs >= durationMs) {
    return toUs;
  }

  const int64_t delta = static_cast<int64_t>(toUs) - static_cast<int64_t>(fromUs);
  return static_cast<uint32_t>(
      static_cast<int64_t>(fromUs) +
      (delta * static_cast<int64_t>(elapsedMs)) / static_cast<int64_t>(durationMs));
}

// Returns zero when a timestamp sampled before command dispatch is
// accidentally supplied after that command. The signed-delta check remains
// correct across the normal uint32_t millis() wraparound for short intervals.
inline uint32_t elapsedMsSince(uint32_t nowMs, uint32_t startedMs)
{
  const uint32_t delta = nowMs - startedMs;
  return static_cast<int32_t>(delta) < 0 ? 0U : delta;
}

} // namespace HobbyServoMotion
