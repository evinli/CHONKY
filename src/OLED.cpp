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
    display->setCursor(0,0);
    display->println("TEST TEST 2");
    display->display();
}

void OLED::write() {

}