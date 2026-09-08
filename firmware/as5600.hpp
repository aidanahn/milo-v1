/**
 * @file as5600.hpp
 * @brief AS5600 encoder interface for signed shaft speed and wheel odometry.
 */

#ifndef PACER_FIRMWARE_AS5600_HPP_
#define PACER_FIRMWARE_AS5600_HPP_

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Estimate signed motion from a 12-bit magnetic encoder.
 * @details Speed is measured at the encoder shaft; distance accounts for the
 * transmission ratio and wheel circumference. I2C reads are synchronous and
 * have no application-level timeout. Filter history is shared across instances.
 */
class AS5600 {
  private:
    String direction;
    double distanceTraveled;
    double rotationsPerSecond;
    unsigned long previousTime;
    int previousAngle;
    double gearRatio;
    double wheelCircumference;
    /**
     * @return Raw 12-bit angle in encoder counts (0 through 4095).
     * @note Waits until two bytes are available; a missing response can block indefinitely.
     */
    int getRawAngle();
    /**
     * @brief Apply the scalar speed filter using persistent, shared history.
     * @param u Signed encoder-shaft speed in revolutions per second.
     * @return Filtered speed in the same units.
     */
    double kalman(double u);

  public:
    /**
     * @brief Configure direction and conversion from shaft rotation to travel.
     * @param direction "FORWARD" or "REVERSE"; selects the positive travel direction.
     * @param gearRatio Encoder-shaft revolutions per wheel revolution; must be positive.
     * @param wheelCircumference Wheel circumference in meters; must be positive.
     * @note Arguments are not validated. Hardware access starts in begin().
     */
    AS5600(String direction, double gearRatio, double wheelCircumference);
    /**
     * @brief Initialize Wire at 1 MHz and capture the starting angle and timestamp.
     * @pre The encoder is connected at I2C address 0x36 on the default Wire pins.
     */
    void begin();
    /**
     * @brief Sample the encoder and update filtered speed and accumulated distance.
     * @pre Call begin() first and allow a nonzero elapsed time between samples.
     * @note Shaft motion must be less than half a revolution between readings
     * for the wrap correction to recover direction unambiguously.
     */
    void update();
    /**
     * @brief Zero accumulated distance without resetting sample or filter history.
     */
    void resetDistanceTraveled();
    /**
     * @return Signed wheel travel in meters since construction or the last reset.
     */
    double getDistanceTraveled();
    /**
     * @return Latest filtered encoder-shaft speed in revolutions per second.
     */
    double getRotationsPerSecond();
};

#endif
