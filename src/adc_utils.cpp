/**
 * @file      adc_utils.cpp
 * @author    Creators of CHONKY 
 * @brief     Function implementations for ADC calculations 
 */

#include "adc_utils.h"

int getAvgAnalogValue(int analogPin, int numReadings) {
    int avgValue = 0;
    for (int i = 0; i < numReadings; i++) {
        avgValue += analogRead(analogPin);
    }
    return (int)(avgValue / numReadings);
}
