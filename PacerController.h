#ifndef PacerController_h
#define PacerController_h

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PID_v1.h>

class PacerController {
  private:
    int escPin;
    int servoPin;
    int distance;
    int pace;
    bool isPacing;
    double goalRotationsPerSecond;
    double gearRatio;
    double wheelCircumference;
    const int FREQUENCY = 50;
    const int RESOLUTION = 14;
    Adafruit_NeoPixel& strip;
    void setESCMicroseconds(int microseconds);
    void setServoAngle(int angle);
    void resetControls();
    void calculateGoalRotationsPerSecond();

  public:
    PacerController(int escPin, int servoPin, double gearRatio, double wheelCircumference, Adafruit_NeoPixel& strip);
    void begin();
    void update(double steeringOutput, double velocityOutput, double distanceTraveled);
    void start(int distance, int pace);
    void stop();
    bool getPacingStatus();
    void setStatusColor(uint8_t r, uint8_t g, uint8_t b);
    double getGoalRotationsPerSecond();
};

#endif