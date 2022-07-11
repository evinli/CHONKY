/**
 * @file      utils.h
 * @author    Creators of CHONKY 
 * @brief     Header file for common helper functions 
 */

#pragma once

#include <Arduino.h>
#include "pins.h"
#include "constants.h"

/**
 * @brief Return average analog value over a given number of samples
 * 
 * @param analogPin pin to read from
 * @param numReadings number of samples
 * @return average ADC value
 */
int getAvgAnalogValue(int analogPin, int numReadings);

/**
 * @brief Sets all the timers to 20ms period; ensures 
 *        consistency between servos and motors on the same timer
 */
void syncTimers();

