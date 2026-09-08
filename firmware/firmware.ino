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

void debug() {
  Serial.print("20");
  Serial.print(",");
  Serial.print(as5600.getRotationsPerSecond());
  Serial.print(",");
  Serial.println("30");
}

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

void handleStop() {
  pacer.stop();
}

void handleCalibrate() {
  qtr.resetCalibration();
  pacer.setStatusColor(255, 255, 0);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  pacer.setStatusColor(0, 255, 0);
}

void handlePID() {
  steeringPID.SetTunings(settings.steeringKp / 1000, settings.steeringKi / 1000, settings.steeringKd / 1000);
  velocityPID.SetTunings(settings.velocityKp, settings.velocityKi, settings.velocityKd);
}
