/**
 * @file firmware.ino
 * @brief Initialize hardware and coordinate the Pacer feedback-control loop.
 *
 * Connects QTR line sensing and AS5600 odometry to steering and velocity PID
 * controllers. WebInterface dispatches run commands to the callbacks here;
 * PacerController applies actuator outputs and handles distance completion.
 */

#include <Arduino.h>
#include "as5600.hpp"
#include "pacer_controller.hpp"
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include "config.hpp"
#include "web_interface.hpp"
#include <PID_v1.h>

uint16_t sensorValues[config::SENSOR_COUNT];
PacerSettings settings;
double velocityInput, velocityOutput, velocitySetpoint;
double steeringInput, steeringOutput, steeringSetpoint;

PID velocityPID(&velocityInput, &velocityOutput, &velocitySetpoint, settings.velocityKp, settings.velocityKi, settings.velocityKd, DIRECT);
PID steeringPID(&steeringInput, &steeringOutput, &steeringSetpoint, settings.steeringKp / 1000, settings.steeringKi / 1000, settings.steeringKd / 1000, REVERSE);
AS5600 as5600("REVERSE", config::GEAR_RATIO, config::WHEEL_CIRCUMFERENCE);
Adafruit_NeoPixel strip(config::LED_COUNT, config::LED_PIN, NEO_GRB + NEO_KHZ800);
PacerController pacer(config::ESC_PIN, config::SERVO_PIN, config::GEAR_RATIO, config::WHEEL_CIRCUMFERENCE, strip);
WebInterface web(settings);
QTRSensors qtr;

void debug();
void handleStart();
void handleStop();
void handleCalibrate();
void handlePID();

/**
 * @brief Initialize serial output, control loops, web routes, and hardware.
 * @details Configures both PID controllers with a 20 ms sample interval.
 * Sensor calibration is performed separately through the web interface.
 */
void setup() {
  Serial.begin(921600);

  velocityPID.SetSampleTime(20);
  velocityPID.SetOutputLimits(0, 500);
  velocityPID.SetMode(AUTOMATIC);
  steeringSetpoint = 7500;
  steeringPID.SetSampleTime(20);
  steeringPID.SetOutputLimits(-90, 90);
  steeringPID.SetMode(AUTOMATIC);

  web.begin(handleStart, handleStop, handleCalibrate, handlePID);

  qtr.setTypeRC();
  qtr.setSensorPins(config::SENSOR_PINS, config::SENSOR_COUNT);
  qtr.setDimmingLevel(0);
  qtr.setTimeout(3000);

  as5600.begin();

  pacer.begin();
}

/**
 * @brief Service web requests and update feedback control while pacing.
 * @details Encoder and line-sensor readings feed the velocity and steering
 * controllers. PID Compute() calls respect their configured sample intervals;
 * the Arduino loop itself does not run at a fixed frequency.
 */
void loop() {
  web.update();
  if (pacer.getPacingStatus()) {
    as5600.update();

    debug();

    steeringInput = qtr.readLineWhite(sensorValues);
    steeringPID.Compute();
    velocityInput = as5600.getRotationsPerSecond();
    velocityPID.Compute();

    pacer.update(steeringOutput, velocityOutput, as5600.getDistanceTraveled());
  }
}

/**
 * @brief Write encoder speed to serial between fixed plot references.
 * @details Emits CSV values: 20, encoder-shaft rotations per second, 30.
 * The outer values are plotting references, not measured speeds.
 */
void debug() {
  Serial.print("20");
  Serial.print(",");
  Serial.print(as5600.getRotationsPerSecond());
  Serial.print(",");
  Serial.println("30");
}

/**
 * @brief Start a run using the current distance and duration settings.
 * @details Resets distance accumulation and reinitializes both PID controllers
 * with zero output. Sets the velocity target in encoder-shaft rotations per
 * second. WebInterface updates settings before invoking this callback.
 */
void handleStart() {
  as5600.resetDistanceTraveled();
  velocityPID.SetMode(MANUAL);
  velocityOutput = 0;
  velocityPID.SetMode(AUTOMATIC);
  steeringPID.SetMode(MANUAL);
  steeringOutput = 0;
  steeringPID.SetMode(AUTOMATIC);
  pacer.start(settings.distance, settings.pace);
  velocitySetpoint = pacer.getGoalRotationsPerSecond();
}

/**
 * @brief Stop pacing and restore neutral throttle and centered steering.
 */
void handleStop() {
  pacer.stop();
}

/**
 * @brief Reset and collect 400 calibration samples from the QTR array.
 * @details Shows yellow during calibration and green afterward. This callback
 * blocks web servicing and the main control loop until sampling completes;
 * the sensor array must encounter the line and background during sampling.
 */
void handleCalibrate() {
  qtr.resetCalibration();
  pacer.setStatusColor(255, 255, 0);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  pacer.setStatusColor(0, 255, 0);
}

/**
 * @brief Apply the current web-editable gains to both PID controllers.
 * @details Steering gains are divided by 1000 to match the QTR position scale;
 * velocity gains are applied directly. WebInterface updates the settings first.
 */
void handlePID() {
  steeringPID.SetTunings(settings.steeringKp / 1000, settings.steeringKi / 1000, settings.steeringKd / 1000);
  velocityPID.SetTunings(settings.velocityKp, settings.velocityKi, settings.velocityKd);
}
