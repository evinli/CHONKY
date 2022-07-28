/**
 * @file      slave_main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main file for the slave bluepill, copy this into
 *            main.cpp to use
 */

#include "Arduino.h"
#include <string.h>
#include <NewPing.h>
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"

// Class instantiations
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
PID irFollow(IRFollower, &leftMotor, &rightMotor, &display);

void setup() {
    display.setUp();
    tapeFollow.setMotorSpeed(80);
    tapeFollow.setKP(8);
    tapeFollow.setKD(4);
    tapeFollow.setKI(0);
}

void loop() {
    tapeFollow.usePID();    
}
