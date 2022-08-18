/**
 * @file      servo.h
 * @author    Creators of CHONKY 
 * @brief     Custom servo class 
 */

#pragma once

#include "pins.h"
#include "Arduino.h"
#include "constants.h"

class Servo {
  public:
    /**
     * @brief Construct a new servo object.
     * 
     * @param pwmPin pin that connects to the pwm pin of the servo.
     */
    Servo(PinName pwmPin);
    
    /**
     * @brief Method to move the servo to a specified angle, non blocking.
     * 
     * @param angle 
     */
    void write(int angle);

    /**
     * @brief Method to slowly actuate a servo to a specified angle with
     *        incremental servo.write() and delays in between
     *        (smoother than servo.write()).
     * 
     * @param angle final angle to reach by servo.
     * @param delayMilli time between increments of servo, smaller numeber
     *                   will lead to faster actuation.
     */
    void slowWrite(int finalAngle, int delayMilli);

    private:
      PinName pwmPin;
      int currentAngle;
};
