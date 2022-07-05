/**
 * @file      diagnostics.h
 * @author    Creators of CHONKY 
 * @brief     Header file for testing Bluepill functionalities 
 * 
 * @copyright Copyright (c) 2022
 */

// include guard that prevents header files from being compiled multiple times
#pragma once

// include all header file dependencies here
#include "pins.h"

class Diagnostics {
  public:
    /**
     * @brief Blinks the built-in LED on the Bluepill 
     */
    static void blinkLED();

    /**
     * @brief Forces all pins to give a digital HI output (test using scope)
     */
    static void writeDigital();

    /**
     * @brief Forces all PWM-enabled pins to output a PWM signal (test using scope)
     */
    static void writePWM();

    /**
     * @brief Pull all digital pins down and print output to the serial monitor
     */
    static void readDigital();

    /**
     * @brief Pull all analog pins down and print output to the serial monior
     */
    static void readAnalog();

};