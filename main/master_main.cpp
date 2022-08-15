// /**
//  * @file      main.cpp
//  * @author    Creators of CHONKY
//  * @brief     Main control loop for CHONKY codebase
//  */

// #include <NewPing.h>
// #include <string.h>

// #include "Arduino.h"
// #include "OLED.h"
// #include "arm.h"
// #include "constants.h"
// #include "master.h"
// #include "motor.h"
// #include "pins.h"
// #include "servo.h"

// // Class instantiations
// Servo clawServo(CLAW_SERVO);  // CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// OLED display(&display_handler);
// Servo elbowServo(ELBOW_SERVO);
// Servo baseServo(BASE_PLATE_SERVO);
// Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
// NewPing verticalSonar(CLAW_SCAN_TRIG, CLAW_SCAN_ECHO, 200);
// Arm arm(&shoulderMotor, &elbowServo, &clawServo, &baseServo, 200, &verticalSonar);
// Master master(&arm);

// void setup() {
//     #ifndef COMP_MODE
//     display.setUp();
//     #endif
//     // Keep bluepill LED on in case of noise-induced resets
//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, LOW);
// }

// void loop() {
//     master.determineState();
// }