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

void syncTimer() {
    for (int i = 1; i <= NUM_TIMERS; i++) {
      HardwareTimer timer();
      timer.pause();
      timer.setPeriod(TIMER_PERIOD);
      timer.refresh();
      timer.resume();
  }
}
