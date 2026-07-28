#include "as5048a.h"
#include "as5600.h"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); \
    ++failures; \
  } \
} while (0)

template <typename Encoder>
void verifyDirection(Encoder& encoder)
{
  encoder.setOutputDegreesPerEncoderRevolution(15.6f);

  const float positive = encoder.computeContinuousAngleDeg(4096);
  CHECK(std::fabs(positive - 3.9f) < 0.001f);
  CHECK(encoder.directionSign() == 1.0f);

  CHECK(encoder.setDirectionSign(-1.0f));
  CHECK(encoder.directionSign() == -1.0f);
  CHECK(std::fabs(encoder.lastContinuousDegrees() + 3.9f) < 0.001f);

  const float negative = encoder.computeContinuousAngleDeg(8192);
  CHECK(std::fabs(negative + 7.8f) < 0.001f);
  CHECK(std::fabs(encoder.lastContinuousEncoderDegrees() + 180.0f) < 0.001f);

  CHECK(!encoder.setDirectionSign(0.0f));
  CHECK(!encoder.setDirectionSign(2.0f));
  CHECK(encoder.directionSign() == -1.0f);

  CHECK(encoder.setDirectionSign(1.0f));
  CHECK(std::fabs(encoder.lastContinuousDegrees() - 7.8f) < 0.001f);
}

} // namespace

int main()
{
  SPIClass spi;
  AS5048A as5048a(spi, 10);
  verifyDirection(as5048a);

  TwoWire wire;
  AS5600 as5600(wire);
  verifyDirection(as5600);

  if (failures != 0) {
    std::printf("encoder direction tests failed: %d\n", failures);
    return 1;
  }

  std::printf("encoder direction tests passed\n");
  return 0;
}
