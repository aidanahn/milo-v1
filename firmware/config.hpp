/**
 * @file config.hpp
 * @brief Hardware constants and initial values for web-editable settings.
 */

#ifndef PACER_FIRMWARE_CONFIG_HPP_
#define PACER_FIRMWARE_CONFIG_HPP_

#include <Arduino.h>

/// Hardware mapping and network configuration for the current vehicle.
namespace config {
/// Wheel circumference in meters, derived from a 121.9 mm diameter.
const double WHEEL_CIRCUMFERENCE = PI * (121.9 / 1000.0);
/// Encoder-shaft revolutions per wheel revolution across both gear stages.
const double GEAR_RATIO = (40.0 / 20.0) * (38.0 / 13.0);
const uint8_t LED_COUNT = 1;
const uint8_t LED_PIN = 38;
const uint8_t SENSOR_COUNT = 16;
/// GPIO sequence supplied to QTRSensors; order determines the position scale.
const uint8_t SENSOR_PINS[] = { 48, 10, 39, 40, 41, 42, 2, 1, 18, 17, 16, 15, 7, 6, 5, 4 };
const uint8_t ESC_PIN = 21;
const uint8_t SERVO_PIN = 47;
const char* const SSID = "PacerESP32";
const char* const PASSWORD = "04082008";

}  // namespace config

/**
 * @brief Initial run settings and PID gains, held in RAM and editable over HTTP.
 * @details Changes are not persisted across resets. Steering gains are divided
 * by 1000 before application; velocity gains are passed to PID_v1 unchanged.
 */
struct PacerSettings {
  /// Target run distance in meters.
  int distance = 400;
  /// Target duration in seconds for the full run distance.
  int pace = 60;
  double velocityKp = 1.0;
  double velocityKd = 0.0;
  double velocityKi = 2.5;
  double steeringKp = 5.0;
  double steeringKd = 1.1;
  double steeringKi = 0.0;
};

#endif
