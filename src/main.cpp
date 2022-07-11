/**
 * @file      main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main control loop for CHONKY codebase
 */

#include <Arduino.h>
#include <string.h>
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
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);

void setup() {
    Serial.begin(9600);
    display.setUp();
}

void loop() {
    tapeFollow.setMotorSpeed(100);
    tapeFollow.setKP(0.1);
    tapeFollow.usePID();
    delay(1000);
}