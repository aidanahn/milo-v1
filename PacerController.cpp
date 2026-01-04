#include "PacerController.h"

PacerController::PacerController(int escPin, int servoPin, double gearRatio, double wheelCircumference, Adafruit_NeoPixel& strip) 
  : strip(strip)
{
  this->escPin = escPin;
  this->servoPin = servoPin;
  this->isPacing = false;
  this->gearRatio = gearRatio;
  this->wheelCircumference = wheelCircumference;
}

void PacerController::setESCMicroseconds(int microseconds) {
  microseconds = constrain(microseconds, 1000, 2000);
  ledcWrite(escPin, microseconds * 16383 / 20000);
}

void PacerController::setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int microseconds = map(angle, 0, 180, 1000, 2000);
  ledcWrite(servoPin, microseconds * 16383 / 20000);
}

void PacerController::resetControls() {
  setESCMicroseconds(1500);
  setServoAngle(90);
  setStatusColor(0, 255, 0);
}

void PacerController::begin() {
  ledcAttach(escPin, FREQUENCY, RESOLUTION);
  ledcAttach(servoPin, FREQUENCY, RESOLUTION);
  resetControls();
  setStatusColor(0, 255, 0);
}

void PacerController::update(double steeringOutput, double velocityOutput, double distanceTraveled) {
  setESCMicroseconds(1575 + (int)velocityOutput);
  setServoAngle(90 + (int)steeringOutput);
  if (distanceTraveled >= distance) {
    stop();
  }
}

void PacerController::setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, r, g, b);
  strip.show();
}

void PacerController::start(int distance, int pace) {
  this->distance = distance;
  this->pace = pace;
  calculateGoalRotationsPerSecond();
  setStatusColor(255, 255, 255);
  isPacing = true;
}

void PacerController::stop() {
  resetControls();
  isPacing = false;
}

bool PacerController::getPacingStatus() {
  return isPacing;
}

void PacerController::calculateGoalRotationsPerSecond() {
  goalRotationsPerSecond = (distance / (pace * wheelCircumference)) * gearRatio;
}

double PacerController::getGoalRotationsPerSecond() {
  return goalRotationsPerSecond;
}