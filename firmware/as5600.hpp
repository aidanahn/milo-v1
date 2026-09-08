#ifndef PACER_FIRMWARE_AS5600_HPP_
#define PACER_FIRMWARE_AS5600_HPP_

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
  private:
    String direction;
    double distanceTraveled;
    double rotationsPerSecond;
    unsigned long previousTime;
    int previousAngle;
    double gearRatio;
    double wheelCircumference;
    int getRawAngle();
    double kalman(double u);

  public:
    AS5600(String direction, double gearRatio, double wheelCircumference);
    void begin();
    void update();
    void resetDistanceTraveled();
    double getDistanceTraveled();
    double getRotationsPerSecond();
};

#endif