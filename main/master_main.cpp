// /**
//  * @file      master_main.cpp
//  * @author    Creators of CHONKY 
//  * @brief     Main file for the master bluepill, copy this into
//  *            main.cpp to use
//  */

// #include "Arduino.h"
// #include <NewPing.h>
// #include "servo.h"
// #include "OLED.h"
// #include "motor.h"
// #include "constants.h"
// #include "pins.h"
// #include "master.h"

// // Class instantiations
// Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// OLED display(&display_handler);
// Servo clawServo(CLAW_SERVO); // CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
// Servo elbowServo(ELBOW_SERVO);
// Servo baseServo (BASE_PLATE_SERVO);
// Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
// NewPing horizontalSonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
// NewPing verticalSonar(RIGHT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);
// Arm mainArm(&shoulderMotor,&elbowServo,&clawServo, &baseServo, 140);

// void setup() {
//     display.setUp();
// }

// void loop() {

// }
