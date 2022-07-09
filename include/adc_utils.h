/**
 * @file      adc_utils.h
 * @author    Creators of CHONKY 
 * @brief     Header file for ADC calculations 
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
