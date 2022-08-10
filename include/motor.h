/**
 * @file      motor.h
 * @author    Creators of CHONKY 
 * @brief     Header file for motor actuation
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Arduino.h"
#include "pins.h"
#include "constants.h"

class Motor {
  public:
    /**
     * @brief Construct a new Motor object
     * 
     * @param pinA PWM pin A 
     * @param pinB PWM pin B
     */
    Motor(PinName pinA, PinName pinB);

    /**
     * @brief Set the motor speed to an input speed
     * 
     * @param speed int speed from 0 to 255, inclusive (8 bits)
     */
    void setSpeed(int speed);

    /**
     * @brief Stop motors from running
     */
    void stop();

    /**
     * @brief Hard brake motors 
     * 
     * @param currDir current motor direction
     */
    void hardStop(int currDir);

  private:
    PinName pinA;
    PinName pinB;
};

#endif //__MOTOR_H__