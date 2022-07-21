/**
 * @file      OLED.cpp
 * @author    Creators of CHONKY 
 * @brief     Function implementations for writing to the OLED 
 */

#include "OLED.h"

/////////////////// CONSTRUCTORS ///////////////////
OLED::OLED(Adafruit_SSD1306* display_handler) {
    this->display = display_handler;
}

/////////////////// METHODS ///////////////////
void OLED::setUp() {
    display->begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display->clearDisplay();
    display->setTextColor(SSD1306_WHITE);
    display->setTextSize(1);
}

void OLED::clear() {
    display->clearDisplay();
}

void OLED::write(int y_pos, std::string message) {
    display->setCursor(0, y_pos);
    display->println(message.c_str());
    display->display();
}