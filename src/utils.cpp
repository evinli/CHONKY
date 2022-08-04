/**
 * @file      utils.cpp
 * @author    Creators of CHONKY 
 * @brief     Common helper function implementations
 */

#include "utils.h"

int getAvgAnalogValue(int analogPin, int numReadings) {
    int avgValue = 0;
    for (int i = 0; i < numReadings; i++) {
        avgValue += analogRead(analogPin);
    }
    return (int)(avgValue / numReadings);
}
