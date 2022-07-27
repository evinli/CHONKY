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
ServoP clawServo(CLAW_SERVO); //CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
//PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
ServoP elbowServo(ELBOW_SERVO);
ServoP baseServo (BASE_PLATE_SERVO);

Motor shoulderMotor(PB_8, PB_9);
// NewPing sonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
// NewPing sonar1(LEFT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);
Arm mainArm(&shoulderMotor,&elbowServo,&clawServo, &baseServo, 140);

int potValue;
int loopCount=0;

void setup() {
    display.setUp();
    display.clear();
    display.write(0,"Starting");

}

void loop(){
    //pwm_start(PB_8,100,1000,TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    //pwm_start(PB_9,100,1000,TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    //mainArm.moveShoulderJoint(90);    
    //shoulderMotor.setSpeed(-100);
    //mainArm.rotateBase(180);
    mainArm.elbow->write(20);
    mainArm.moveShoulderJoint(90);
    mainArm.rotateBase(180);
    mainArm.claw->write(30);
    delay(2000);
    display.clear();
    display.write(0,"moving in plane");
    mainArm.moveInPlane(10,40);
    display.clear();
    display.write(0,"done moving in plane");
    delay(2000);
    mainArm.claw->write(30);
    mainArm.elbow->write(20);
    mainArm.moveShoulderJoint(90);
    mainArm.rotateBase(90);
    delay(2000);
    mainArm.moveInPlane(10,40);
    delay(1000);
    mainArm.claw->write(270);
    delay(2000);
}