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
// Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
// Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
Servo clawServo(CLAW_SERVO); //CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
//PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
Servo elbowServo(ELBOW_SERVO);
Servo baseServo (BASE_PLATE_SERVO);
Motor shoulderMotor(PB_8, PB_9);
NewPing horizontalSonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
NewPing verticalSonar(RIGHT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);
Arm mainArm(&shoulderMotor,&elbowServo,&clawServo, &baseServo, 150);

int potValue;
int loopCount=0;                      
int treasureDistance;
const int movingAvgRange=50;
int sonarDist[movingAvgRange];
int sum;
int threshold=35;

void setup() {
    display.setUp();
    display.clear();
    display.write(0,"started");
    delay(1000);
}

void loop(){
    //pwm_start(PB_8,100,1000,TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    //pwm_start(PB_9,100,1000,TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    //mainArm.moveShoulderJoint(90);    
    //shoulderMotor.setSpeed(-100);
    //mainArm.rotateBase(180);

    // for(int i=0;i<6;i++){
    //     elbowServo.slowWrite(90,i*10);
    //     delay(3000);
    //     elbowServo.slowWrite(45,i*10);
    //     delay(3000);
    // }
    
    // display.clear();
    // display.write(0,"done with alpha calc");
    // double shoulderJointAngle=alpha+theta;
    // display.clear();
    // display.write(0,"done with shoulder joint calc");
    // display.write(0,std::to_string(phi));
    // display.clear();
    // display.write(0,"wrote phi");
    // display.write(0,std::to_string(shoulderJointAngle));
    // delay(500);
    // mainArm.moveInPlane(3,26);
    // mainArm.moveInPlane(20,26);
     mainArm.sweep(3,16,26);
    // delay(2000);

    // loopCount++;
    // if (loopCount>30){
    //     for (int i=0;i<(movingAvgRange-1);i++){
    //         sonarDist[i]=sonarDist[i+1];
    //     }
    //     sonarDist[movingAvgRange-1]=horizontalSonar.ping_cm();

    //     sum=0;
    //     for (int i=0;i<movingAvgRange;i++){
    //         sum+=sonarDist[i];
    //     }

    //     if ((sum/movingAvgRange)<threshold){
    //         //display.clear();
    //         //display.write(0,"treasure detected");
    //         Serial.println("treasure detected");
    //         slave.stop();

    //     }
    //     else{
    //         //display.clear();
    //     }

    //     Serial.println(sum/movingAvgRange);

    // } else{
    //     sonarDist[loopCount-1]=horizontalSonar.ping_cm();
    // }

}