/**
 * @file      main.cpp
 * @author    Creators of CHONKY
 * @brief     Main control loop for CHONKY codebase
 */

#include <NewPing.h>
#include <string.h>

#include "Arduino.h"
#include "OLED.h"
#include "arm.h"
#include "constants.h"
#include "motor.h"
#include "pins.h"
#include "servo.h"
#include "master.h"

// Class instantiations
Servo clawServo(CLAW_SERVO); // CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
Servo elbowServo(ELBOW_SERVO);
Servo baseServo(BASE_PLATE_SERVO);
Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
NewPing horizontalSonar(TREASURE_TRIG, TREASURE_ECHO, 200);  // NewPing setup of pins and maximum distance
NewPing verticalSonar(CLAW_SCAN_TRIG, CLAW_SCAN_ECHO, 200);
Arm arm(&shoulderMotor, &elbowServo, &clawServo, &baseServo, 150, &verticalSonar, &horizontalSonar, &display);

// Variable declarations
MasterState state;
int idolCount;

// Function prototypes
double idolDetect(int numReadings);
bool advanceState();
void signalSlaveAdvance();
void endSlaveAdvanceSignal();
void stopSlave();
void goSlave();

void setup() {
    display.setUp();
    state = MasterState::Inactive;
    idolCount = 0;
    pinMode(SLAVE_ADVANCE_STATE, OUTPUT);
    pinMode(SLAVE_STOP_DRIVE, OUTPUT);
}

void loop() {
    display.clear();
    endSlaveAdvanceSignal();

    switch(state) {
        case(MasterState::Inactive): {
            arm.rotateBase(RIGHT_SIDE_ANGLE);
            arm.moveInPlaneElbowFirst(15, 25); // trial and error, where to move arm so that it can detect idol 1
            clawServo.write(CLAW_OPEN_ANGLE);
            advanceState();
            signalSlaveAdvance();
        }

        case(MasterState::FirstIdol): {
            if (idolDetect(IDOL_DETECT_SAMPLES) < 30) { // trial and error, how close is idol 1 to arm?
                display.clear();
                display.write(0, "Idol 1 Detected");
                stopSlave();
                if (idolDetect(IDOL_DETECT_SAMPLES) < 30) { // trial and error, how close is idol 1 to arm?
                    display.clear();
                    display.write(0, "Idol 1 Detected & Verified");
                    // Treasure pickup sequence
                    arm.sweepAndDetect(15, 30, 45, LEFT_DROPOFF_ANGLE); // trial and error, where should arm go to pick up idol 1

                    // Reset arm 
                    arm.rotateBase(RIGHT_SIDE_ANGLE);
                    arm.moveInPlaneElbowFirst(15, 25); // trial and error, where to move arm to detect idol 2
                    advanceState();
                    goSlave();
                }
            }
        }

        case(MasterState::SecondIdol): {
            if (idolDetect(IDOL_DETECT_SAMPLES) < 20) { // trial and error
                display.clear();
                display.write(0, "Idol 2 Detected");
                stopSlave();
                if (idolDetect(IDOL_DETECT_SAMPLES) < 20) { // trial and error
                    display.clear();
                    display.write(0, "Idol 2 Detected & Verified");
                    // Treasure pickup sequence
                    arm.sweepAndDetect(15, 20, 45, LEFT_DROPOFF_ANGLE); // trial and error, where should arm go to pick up idol 2

                    // Move arm into resting position 
                    arm.goToRestingPos();
                    advanceState();
                    goSlave();
                }
            }
        }
    }
    // // WORKING TREASURE GRASPING CODE
    // mainArm.rotateBase(270);
    // mainArm.moveShoulderJoint(90);
    // mainArm.moveInPlaneElbowFirst(6, 25);
    // delay(2000);
    // for (int i = 220; i < 300; i++) {
    //     mainArm.rotateBase(i);
    //     int ultraSonicReading = ultrasonicAvg(50);
    //     if (ultraSonicReading < 25) {
    //         display.clear();
    //         display.write(0, "treasure detected");
    //         int secondaryVerification = ultrasonicAvg(50);
    //         if (secondaryVerification < 25) {
    //             display.write(20, "secondary verification passed");
    //             mainArm.rotateBase(i + 4);
    //             mainArm.sweepAndDetect(5, 20, 35);
    //             break;
    //         }
    //     }
    // }

    // display.clear();

}

double idolDetect(int numReadings) {
    int sum = 0;
    int reading;

    for (int i = 0; i < numReadings; i++) {
        reading = horizontalSonar.ping_cm();
        if (reading == 0) {
            sum += 400;
        } 
        else {
            sum += reading;
        }
    }
    return (double)(sum)/numReadings;
}

bool advanceState() {
    if (state == MasterState::Done) {
         return false; 
    }

    state = static_cast<MasterState>(static_cast<int>(state) + 1);
    return true;
}

void signalSlaveAdvance() {
    digitalWrite(SLAVE_ADVANCE_STATE, HIGH);
    // Ensure slave has recieved signal to advance
    delay(100);
}

void endSlaveAdvanceSignal() { 
    digitalWrite(SLAVE_ADVANCE_STATE, LOW); 
}

void stopSlave() {
    digitalWrite(SLAVE_STOP_DRIVE, HIGH);
    // Ensure slave has fully stopped before performing any other tasks
    delay(500);
}

void goSlave() { 
    digitalWrite(SLAVE_STOP_DRIVE, LOW); 
}