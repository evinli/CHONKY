/**
 * @file      main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main control loop for CHONKY codebase
 */

#include "Arduino.h"
#include <string.h>
#include <NewPing.h>
#include <Servo.h>
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"

// Class instantiations
Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
//Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
PID irFollow(IRFollower, &leftMotor, &rightMotor, &display);
// Servo baseServo;
// Servo leadScrew;
// Servo elbowServo;
// Servo clawServo;
NewPing sonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
NewPing sonar1(LEFT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);

// Variable declarations
float distance;
int leftError;
int rightError;
int centreError;
int i = 0;
int loopCount=0;

void setup() {
    display.setUp();
    irFollow.setMotorSpeed(50);
    irFollow.setKP(1);
    // rightMotor.stop();
    // leftMotor.setSpeed(100);
}

void loop() {
    // rightMotor.stop();
    // leftMotor.setSpeed(100);

    irFollow.usePID();
}
    
    

