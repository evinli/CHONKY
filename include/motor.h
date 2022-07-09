/**
 * @file      motor.h
 * @author    Creators of CHONKY 
 * @brief     Header file for the motor class 
 */

#pragma once

#include <Arduino.h>
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
    Motor(int pinA, int pinB);

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

  private:
    PinName pinA;
    PinName pinB;
    int previousSpeed;
};