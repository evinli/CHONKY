// /**
//  * @file      slave_main.cpp
//  * @author    Creators of CHONKY 
//  * @brief     Main file for the slave bluepill, copy this into
//  *            main.cpp to use
//  */

// #include "Arduino.h"
// #include <string.h>
// #include "PID.h"
// #include "motor.h"
// #include "pins.h"
// #include "OLED.h"
// #include "slave.h"

// // Class instantiations
// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// OLED display(&display_handler);
// Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
// Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
// PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
// PID irFollow(IRFollower, &leftMotor, &rightMotor, &display);
// Slave slave(&leftMotor, &rightMotor, &tapeFollow, &irFollow);

// void setup() {
//     display.setUp();
//     // Turn built-in LED on in case Bluepill restarts due to noise
//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, LOW);
// }

// void loop() {
//     slave.determineState();
// }
