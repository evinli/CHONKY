/**
 * @file      diagnostics.h
 * @author    Creators of CHONKY 
 * @brief     Header file for testing Bluepill functionalities 
 */

#pragma once

#include <Arduino.h>
#include "pins.h"
#include "constants.h"

class Diagnostics {
  public:
    /**
     * @brief Blinks the built-in LED on the Bluepill 10 times
     */
    static void blinkLED();

    /**
     * @brief Forces all pins to give a digital HI output (test using scope)
     *        Note: PA11 and PA12 will not behave as expected due to the USB bootloader, 
     *              need to comment out contens of USBSerial.begin to enable functionality those pins
     *        Caution: Disconnect all peripherals before running this
     */
    static void writeDigital();

    /**
     * @brief Forces all PWM-enabled pins to output a PWM signal (test using scope)
     *        Note: PA11 will not output PWM without disabling USB serial
     *        Caution: Disconnect all peripherals before running this
     *
     */
    static void writePWM();

    /**
     * @brief Pull down digital pins and test digitalRead; print output to serial monitor 
     *        Note: skip PA9 and PA10 since these pins are used for the serial monitor
     */
    static void readDigital();

    /**
     * @brief Pull down analog pins and test analogRead; print output to serial monitor
     */
    static void readAnalog();

};