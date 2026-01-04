#include "AS5600.h"

AS5600::AS5600(String direction, double gearRatio, double wheelCircumference) {
  if (direction == "FORWARD") {
    this->direction = "FORWARD";
  }
  else if (direction == "REVERSE") {
    this->direction = "REVERSE";
  }
  distanceTraveled = 0.0;
  rotationsPerSecond = 0.0;
  previousAngle = 0;
  previousTime = 0;
  this->gearRatio = gearRatio;
  this->wheelCircumference = wheelCircumference;
}

int AS5600::getRawAngle() {
  Wire.requestFrom(0x36, 2);
  while (Wire.available() < 2);
  int high = Wire.read();
  int low = Wire.read();
  int angle = ((high << 8) | low) & 0x0FFF;
  return angle;
}

void AS5600::begin() {
  Wire.begin();
  Wire.setClock(1000000);
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);

  previousAngle = getRawAngle();
  previousTime = micros();
}

void AS5600::update() {
  int currentAngle = getRawAngle();
  unsigned long currentTime = micros();
  double deltaTime = (currentTime - previousTime) / 1e6;

  int deltaAngle = currentAngle - previousAngle;

  if (deltaAngle > 2048) deltaAngle -= 4096;
  if (deltaAngle < -2048) deltaAngle += 4096;

  if (direction == "FORWARD") {
    rotationsPerSecond = kalman((double)deltaAngle / 4096.0 / deltaTime);
    distanceTraveled += (double)deltaAngle / 4096.0 / gearRatio * wheelCircumference;
  }

  if (direction == "REVERSE") {
    rotationsPerSecond = kalman(-((double)deltaAngle / 4096.0 / deltaTime));
    distanceTraveled -= (double)deltaAngle / 4096.0 / gearRatio * wheelCircumference;
  }

  previousAngle = currentAngle;
  previousTime = currentTime;
}

void AS5600::resetDistanceTraveled() {
  distanceTraveled = 0;
}

double AS5600::getDistanceTraveled() {
  return distanceTraveled;
}

double AS5600::getRotationsPerSecond() {
  return rotationsPerSecond;
}

double AS5600::kalman(double u) {
  static double p = 0.0;
  static double u_hat = 0.0;
  static const double r = 50.0;
  static const double h = 1.0;
  static const double q = 0.1;
  static double k = 0;

  k = p * h / (h * p * h + r);
  u_hat = u_hat + k * (u - h * u_hat);

  p = (1 - k * h) * p + q;

  return u_hat;
}