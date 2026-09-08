/**
 * @file pacer_controller.hpp
 * @brief Actuator commands and distance-based pacing state.
 */

#ifndef PACER_FIRMWARE_PACER_CONTROLLER_HPP_
#define PACER_FIRMWARE_PACER_CONTROLLER_HPP_

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PID_v1.h>

/**
 * @brief Translate control outputs into throttle and steering commands.
 * @details Owns pacing state and target-speed conversion. Sensor acquisition
 * and PID computation are performed by the caller. The referenced LED strip
 * must outlive this controller.
 */
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
    /**
     * @brief Clamp an ESC pulse to 1000–2000 microseconds and write PWM duty.
     */
    void setESCMicroseconds(int microseconds);
    /**
     * @brief Clamp a servo angle to 0–180 degrees and map it to a 1–2 ms pulse.
     */
    void setServoAngle(int angle);
    /**
     * @brief Command a 1500 microsecond ESC pulse, 90-degree steering, and green LED.
     */
    void resetControls();
    /**
     * @brief Convert distance divided by duration into encoder-shaft speed.
     */
    void calculateGoalRotationsPerSecond();

  public:
    /**
     * @brief Store actuator connections and drivetrain geometry.
     * @param escPin ESP32 GPIO for the ESC PWM signal.
     * @param servoPin ESP32 GPIO for the steering PWM signal.
     * @param gearRatio Encoder-shaft revolutions per wheel revolution; must be positive.
     * @param wheelCircumference Wheel circumference in meters; must be positive.
     * @param strip Status LED strip, owned and initialized by the caller.
     */
    PacerController(int escPin, int servoPin, double gearRatio, double wheelCircumference, Adafruit_NeoPixel& strip);
    /**
     * @brief Attach 50 Hz, 14-bit PWM channels and command neutral controls.
     * @note PWM attachment results are not checked.
     */
    void begin();
    /**
     * @brief Apply control outputs, then stop if the distance target is reached.
     * @param steeringOutput Steering offset from center in degrees.
     * @param velocityOutput Throttle pulse offset above 1575 microseconds.
     * @param distanceTraveled Measured wheel travel in meters for the current run.
     * @pre Call only during an active run, after begin() and start().
     * @note Outputs are converted to integers and clamped at the actuator boundary;
     * this method does not independently check the pacing flag.
     */
    void update(double steeringOutput, double velocityOutput, double distanceTraveled);
    /**
     * @brief Set the run target, mark pacing active, and show a white status LED.
     * @param distance Target travel in meters; must be positive.
     * @param pace Target duration for that distance in seconds; must be positive.
     * @note Inputs are not validated. This does not reset odometry or PID state.
     */
    void start(int distance, int pace);
    /**
     * @brief Mark pacing inactive, command neutral controls, and show green.
     */
    void stop();
    /**
     * @return Whether a run has been started and has not yet been stopped.
     */
    bool getPacingStatus();
    /**
     * @brief Set and immediately display the color of status pixel zero.
     * @param r Red intensity, 0 through 255.
     * @param g Green intensity, 0 through 255.
     * @param b Blue intensity, 0 through 255.
     */
    void setStatusColor(uint8_t r, uint8_t g, uint8_t b);
    /**
     * @return Target encoder-shaft speed in revolutions per second.
     * @pre Call start() before reading the target.
     */
    double getGoalRotationsPerSecond();
};

#endif
