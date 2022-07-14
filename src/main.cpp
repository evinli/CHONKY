/**
 * @file      main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main control loop for CHONKY codebase
 */

#include "Arduino.h"
#include <string.h>
#include <Servo.h>
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
// Servo baseServo;

void setup() {
    Serial.begin(9600);
    display.setUp();
    // baseServo.attach(BASE_PLATE_SERVO);
    // baseServo.writeMicroseconds(1000);
    // baseServo.writeMicroseconds(1700);
    // pwm_start(PA_3, 50, 200, RESOLUTION_12B_COMPARE_FORMAT);
}

void loop() {
    tapeFollow.setMotorSpeed(60);
    tapeFollow.setKP(1);
    tapeFollow.setKD(0);
    tapeFollow.setKI(0);
    tapeFollow.usePID();
}