/**
 * @file      OLED.h
 * @author    Creators of CHONKY 
 * @brief     Header file for OLED print functions
 */

#pragma once

#include "Arduino.h"
#include "constants.h"
#include <Adafruit_SSD1306.h>
#include <string.h>


class OLED {
  public:
    /**
     * @brief Construct a new OLED object
     * 
     * @param display_handler Adafruit_SSD1306 object to pass in
     */
    OLED(Adafruit_SSD1306* display_handler);

    /**
     * @brief Initialize OLED display
     */
    void setUp();

    /**
     * @brief Clear OLED display
     */
    void clear();

    /**
     * @brief Function that writes to the OLED screen
     * 
     * @param y_pos y position to write to
     * @param message message to print
     */
    void write(int y_pos, std::string message);
  
  private:
    Adafruit_SSD1306* display;
};

