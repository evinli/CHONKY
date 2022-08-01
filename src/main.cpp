<<<<<<< HEAD
=======
/**
 * @file      main.cpp
 * @author    Creators of CHONKY
 * @brief     Main control loop for CHONKY codebase
 */

#include <NewPing.h>
#include <string.h>

#include "Arduino.h"
#include "OLED.h"
#include "PID.h"
#include "arm.h"
#include "constants.h"
#include "motor.h"
#include "pins.h"
#include "servo.h"

// Class instantiations
// Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
// Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
Servo clawServo(CLAW_SERVO);  // CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
// PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
Servo elbowServo(ELBOW_SERVO);
Servo baseServo(PA_8);
Motor shoulderMotor(PB_8, PB_9);
NewPing horizontalSonar(RIGHT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);  // NewPing setup of pins and maximum distance.
NewPing verticalSonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200);
Arm mainArm(&shoulderMotor, &elbowServo, &clawServo, &baseServo, 150, &verticalSonar, &horizontalSonar, &display);

int potValue;
int loopCount = 0;
int treasureDistance;
const int movingAvgRange = 50;
int sonarDist[movingAvgRange];
int sum;
int threshold = 59;
int loopFlag = 0;
double loopAvg = 0;

void setup() {
    display.setUp();
    display.clear();
    display.write(0, "yup");
    pinMode(PA0, INPUT);
    delay(1000);
}

double ultrasonicAvg(int numReadings) {
    int sum = 0;
    int reading;

    for (int i = 0; i < numReadings; i++) {
        reading = horizontalSonar.ping_cm();
        if (reading == 0) {
            sum += 400;
        } else {
            sum += reading;
        }
    }
    return ((double)sum / (double)numReadings);

    pinMode(PA4, INPUT);
}

void loop() {
    // pwm_start(PB_8,100,1000,TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(PB_9,100,1000,TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    // mainArm.moveShoulderJoint(90);
    // shoulderMotor.setSpeed(-100);
    // mainArm.rotateBase(180);

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
    // mainArm.moveInPlaneShoulderFirst(3,26);
    // mainArm.moveInPlaneShoulderFirst(20,26);
    // display.clear();
    // display.write(0,std::to_string(analogRead(PA0)));
    // mainArm.elbow->write(0);

    // display.clear();
    // display.write(0,std::to_string(analogRead(PA0)));
    // mainArm.sweepAndDetect(5,15,26);
    // display.clear();
    // display.write(0,"treasure detected");
    // //mainArm.testShoulder();

    // display.clear();
    // display.write(0,std::to_string((int)mainArm.avgSampleSonar(40)));
    // mainArm.rotateBase(90);
    // mainArm.moveShoulderJoint(90);
    // elbowServo.write(90);
    // delay(2000);
    // display.clear();
    // display.write(0,std::to_string(analogRead(PA0)));
    // display.write(20,std::to_string(analogRead(PA4)));
    // mainArm.sweepAndDetect(3,16,26);
    // mainArm.moveShoulderJoint(90);
    // display.clear();
    // display.write(0,"moved to 90");
    // mainArm.elbow->write(90);
    // delay(1000);
    // mainArm.sweepAndDetect(3,16,26);

    // display.clear();
    // display.write(0,std::to_string(analogRead(PA0)));

    // mainArm.testShoulder();

    // for(double i=3;i<16;i+=SWEEP_STEP_SIZE){
    //         loopAvg=0;
    //         mainArm.moveInPlaneShoulderFirst(i,26);
    //         int avgUltrasonic=ultrasonicAvg(40);
    //         loopAvg+=avgUltrasonic;
    //         if (avgUltrasonic<10){
    //             display.clear();
    //             display.write(0,"treasure detected");
    //             delay(3000);
    //             mainArm.sweep(i,i+8,26);
    //             mainArm.moveInPlaneShoulderFirst(i+8,21);
    //             delay(1000);
    //             mainArm.claw->write(160);
    //             loopFlag=1;
    //             display.clear();
    //             display.write(0,"treasure detected");
    //             break;
    //         }
    //    }

    // WORKING FOR TIME TRIALS
    //  mainArm.moveShoulderJoint(90);
    //  mainArm.elbow->write(90);
    //  delay(3000);
    //  mainArm.sweepAndDetect(3,20,35);
    //  delay(3000);

    // WORKING TREASURE GRASPING CODE
    mainArm.rotateBase(270);
    mainArm.moveShoulderJoint(90);
    mainArm.moveInPlaneElbowFirst(6, 25);
    delay(2000);
    for (int i = 220; i < 300; i++) {
        mainArm.rotateBase(i);
        int ultraSonicReading = ultrasonicAvg(50);
        if (ultraSonicReading < 25) {
            display.clear();
            display.write(0, "treasure detected");
            int secondaryVerification = ultrasonicAvg(50);
            if (secondaryVerification < 25) {
                display.write(20, "secondary verification passed");
                mainArm.rotateBase(i + 4);
                mainArm.sweepAndDetect(5, 20, 35);
                break;
            }
        }
    }

    // display.clear();
    /// ROTATION DETECTION
    // if (loopFlag==0) {

    // display.clear();
    // display.write(20,std::to_string((int)loopAvg));
    // delay(2000);

    // loopCount++;
    // display.clear();
    // display.write(0,std::to_string(verticalSonar.ping_cm()));
    // display.write(20,std::to_string(loopCount));
    // mainArm.elbow->write(90);

    // int before = millis();

    // ultrasonicAvg(100);

    // int after=millis();

    // display.clear();
    // display.write(0, std::to_string((int(after-before))));
    // delay(1000);

    // loopCount++;
    // if (loopCount>movingAvgRange){
    //      for (int i=0;i<(movingAvgRange-1);i++){
    //         sonarDist[i]=sonarDist[i+1];
    //     }
    //     sonarDist[movingAvgRange-1]=verticalSonar.ping_cm();

    //     sum=0;

    //     for (int i=0;i<movingAvgRange;i++){
    //         sum+=sonarDist[i];
    //     }

    //     display.clear();

    //     if ((sum/movingAvgRange)<threshold){
    //         display.clear();
    //         display.write(0,"treasure detected");
    //         display.write(15,std::to_string(sum/movingAvgRange));
    //         //Serial.println("treasure detected");
    //     }
    //     else{
    //         display.clear();
    //         display.write(0,"");
    //     }

    // } else{
    //     sonarDist[loopCount-1]=horizontalSonar.ping_cm();
    //     display.clear();
    //     display.write(0,std::to_string(loopCount));
    // }
}
>>>>>>> Arm
