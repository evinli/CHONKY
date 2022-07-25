/**
 * @file      main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main control loop for CHONKY codebase
 */

#include "Arduino.h"
#include <string.h>
#include <NewPing.h>
//#include "ServoP.h"
#include <Servo.h>
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"
#include "arm.h"
#include "constants.h"

// Class instantiations
//Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
// Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
//ServoP clawServo(CLAW_SERVO); //CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
//Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
//PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
ServoP elbowServo(ELBOW_SERVO);
Motor shoulderMotor(PB_8, SHOULDER_MOTOR_B);
Servo baseServo;
// NewPing sonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
// NewPing sonar1(LEFT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);
//Arm mainArm(&shoulderMotor,&elbowServo,&clawServo,140);

int potValue;

int loopCount=0;

void setup() {
    pinMode(PB0, INPUT);
    display.setUp();
    baseServo.attach(PA0);
}

void moveBase(int angle){
    int potValue = analogRead(PB0);
    int targetValue = (angle * (1023) /(348.7-22.3)) -69.89; // This is the input angle in terms of the potentiometer values

    while (abs(potValue - targetValue) > 5)
    { 
        if (potValue < targetValue)
        {
            baseServo.write(90);
            while (potValue < targetValue)
            {
                potValue = analogRead(PB0);
            }
        }
        else if (potValue > targetValue)
        {
            baseServo.write(75);
            while (potValue > targetValue)
            {
                potValue = analogRead(PB0);
            }
        }
    }
    baseServo.write(83);
}


void loop() {
    for (int i=3;i<30;i++){
        moveBase(i*10);
        display.clear();
        display.write(0,std::to_string(i));
        delay(500);
    }
}