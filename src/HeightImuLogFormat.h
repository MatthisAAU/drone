#pragma once

#include <math.h>
#include <stdint.h>

namespace itc {

// One CSV row of the IMU + barometer height log.
struct HeightImuRecord {
  uint32_t timestampUs = 0;               // host time after the IMU read [us]
  uint32_t sequence = 0;                  // set by the logger
  float accelG[3] = {0.0f, 0.0f, 0.0f};   // acceleration, sensor frame [g]
  float gyroDps[3] = {0.0f, 0.0f, 0.0f};  // angular rate, sensor frame [deg/s]
  float rollDeg = NAN;                    // from AttitudeEstimator
  float pitchDeg = NAN;
  float pressurePa = NAN;                 // last BMP280 pressure [Pa]
  float temperatureC = NAN;               // last BMP280 temperature [degC]
  uint8_t baroNew = 0;                    // 1 = new barometer value in this row
};

} // namespace itc