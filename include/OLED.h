/**
 * @file      OLED.h
 * @author    Creators of CHONKY 
 * @brief     Header file for OLED print functions
 */

#pragma once

#include "Arduino.h"
#include "constants.h"
#include <Adafruit_SSD1306.h>


class OLED {
  public:
    OLED(Adafruit_SSD1306* display_handler);

    void setUp();

    /**
     * @brief Function that clears and writes to the OLED screen
     */
    void write();
  
  private:
    Adafruit_SSD1306* display;
};

