/**
 * @file      main.h
 * @author    Creators of CHONKY 
 * @brief     Main control loop for CHONKY codebase
 */

#include <Arduino.h>
#include "diagnostics.h"
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"

// Class instantiations
Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);

void setup() {
    display.setUp();
}

void loop() {
    // display_handler.setCursor(0,0);
    // display_handler.print("CHONKY 2022");
    // display_handler.display();  
    // // display.print(); 
}