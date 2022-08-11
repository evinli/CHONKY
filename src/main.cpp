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
#include "master.h"
#include "motor.h"
#include "pins.h"
#include "servo.h"

// Class instantiations
Servo clawServo(CLAW_SERVO);  // CLAW SERVO MUST BE INITIALIZED BEFORE SHOULDER DUE TO PIN DEPENDENCY
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
Servo elbowServo(ELBOW_SERVO);
Servo baseServo(BASE_PLATE_SERVO);
Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
NewPing horizontalSonar(TREASURE_TRIG, TREASURE_ECHO, 200);  // NewPing setup of pins and maximum distance
NewPing verticalSonar(CLAW_SCAN_TRIG, CLAW_SCAN_ECHO, 200);
Arm arm(&shoulderMotor, &elbowServo, &clawServo, &baseServo, 200, &verticalSonar, &horizontalSonar, &display);

// Variable declarations
MasterState state;
int idolCount;
long beforeTime;

// Function prototypes
double idolDetect(int numReadings);
double idolDetectHorizontal(int numReadings);
bool advanceState();
void signalSlaveAdvance();
void endSlaveAdvanceSignal();
void stopSlave();
void goSlave();

void setup() {
    display.setUp();
    display.clear();
    display.write(0, "started");
    state = MasterState::Inactive;
    idolCount = 0;
    pinMode(SLAVE_ADVANCE_STATE, OUTPUT);
    pinMode(SLAVE_STOP_DRIVE, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void tuneJoint(int pin, int displayPosition) {
    display.clear();
    display.write(displayPosition, std::to_string(analogRead(pin)));
}

void loop() {
    endSlaveAdvanceSignal();

    switch (state) {
        case (MasterState::Inactive): {
            display.clear();
            display.write(0, "Inactive State");
            // Set arm to first idol detect position
            clawServo.write(CLAW_OPEN_ANGLE);
            arm.rotateBase(270);
            arm.moveInPlaneElbowFirst(8, 32);
            delay(1000);
            advanceState();
            signalSlaveAdvance();
            break;
        }

        case (MasterState::FirstIdol): {
            display.clear();
            display.write(0, "First Idol State");
            if (idolDetect(IDOL_DETECT_SAMPLES) < 15) {  // trial and error, how close is idol 1 to arm?
                display.clear();
                display.write(0, "Idol 1 Detected");
                if (idolDetect(IDOL_DETECT_SAMPLES) < 15) {  // trial and error, how close is idol 1 to arm?
                    display.clear();
                    display.write(0, "Idol 1 Detected & Verified");
                    stopSlave();
                    delay(1000);
                    arm.rotateBase(280);                // HC: move backward since base doesn't stop instantly
                    arm.moveInPlaneElbowFirst(13, 30);  // HC: move down and out to correct position

                    delay(1000);  // wait to hover for bomb detection

                    if (!arm.magneticBomb()) {
                        arm.grasp();
                        delay(500);
                        arm.dropInBasket(RIGHT_DROPOFF_ANGLE);
                    } else {
                        display.clear();
                        display.write(0, "bomb detected");
                        delay(1000);
                        arm.moveInPlaneShoulderFirst(13, 37);
                    }

                    //  Reset arm
                    arm.rotateBase(270);
                    arm.moveInPlaneShoulderFirst(13, 37);
                    clawServo.write(60);  // trial and error, where to move arm to detect idol 2
                    advanceState();
                    signalSlaveAdvance();
                    goSlave();
                }
            }
            break;
        }

        case (MasterState::PostFirstIdol): {
            delay(5000);
            arm.moveInPlaneElbowFirst(12, 33);
            advanceState();
            break;
        }

        case (MasterState::SecondIdol): {
            display.clear();
            display.write(0, "Second Idol State");
            if (idolDetect(20) < 10) {  // trial and error
                display.clear();
                display.write(0, "Idol 2 Detected");
                display.clear();
                display.write(0, "Idol 2 Detected & Verified");
                stopSlave();
                arm.rotateBase(278);
                // Treasure pickup sequence
                arm.moveInPlaneElbowFirst(14, 30);  //
                delay(1000);

                if (!arm.magneticBomb()) {
                    arm.grasp();
                    delay(500);
                    arm.dropInBasket(RIGHT_DROPOFF_ANGLE);
                } else {
                    display.clear();
                    display.write(0, "bomb detected");
                    delay(1000);
                    arm.moveInPlaneShoulderFirst(13, 37);
                }

                // // Move arm into resting position
                arm.goToRestingPos();
                baseServo.write(BASE_CW_SPEED);
                delay(250);
                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

                advanceState();
                signalSlaveAdvance();
                goSlave();
                beforeTime = millis();
            }
            break;
        }

        case (MasterState::ThirdIdol): {
            display.clear();
            display.write(0, "Third idol state");

            if (millis() - beforeTime > 7500) {
                arm.moveInPlaneShoulderFirst(16, 35);
                clawServo.write(CLAW_OPEN_ANGLE);
                display.clear();
                display.write(0, "Third idol state");
                delay(1000);
                arm.moveInPlaneShoulderFirst(16, 35);
                clawServo.write(CLAW_OPEN_ANGLE);
                baseServo.write(BASE_CCW_SPEED);
                delay(250);
                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
                arm.rotateBase(270);
                arm.rotateBase(270);
                arm.moveInPlaneShoulderFirst(9, 29); //MOVE THIS FURTHER OUT ???
                int loopFlag = 1;

                double slope = ((double)(BASE_ONE_EIGHTY - BASE_NINETY)) / (double)(180 - 90);
                int targetValue = ((double)135 * slope) - (slope * 90 - BASE_NINETY);

                while (analogRead(BASE_POT) > targetValue && loopFlag) {
                    baseServo.write(75);
                    if (idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                        loopFlag = 0;
                    }
                }

                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

                if (!loopFlag) {
                    display.clear();
                    display.write(0, "treasure detected");
                    arm.moveInPlaneShoulderFirst(9, 33);
                    delay(1000);
                }

                if (!arm.magneticBomb() && !loopFlag) {
                    arm.grasp();
                    delay(500);
                    arm.dropInBasket(RIGHT_DROPOFF_ANGLE);
                } else if (!loopFlag) {
                    display.clear();
                    display.write(0, "bomb detected");
                    delay(1000);
                } else {
                    arm.moveInPlaneShoulderFirst(17, 31);
                    targetValue = ((double)270 * slope) - (slope * 90 - BASE_NINETY);
                    loopFlag = 1;

                    while (analogRead(BASE_POT) < targetValue && loopFlag) {
                        baseServo.write(86); 
                        if (idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                            loopFlag = 0;
                        }
                    }
                    pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

                    if (!loopFlag) {
                        display.clear();
                        display.write(0, "treasure detected");
                        arm.moveInPlaneShoulderFirst(19, 28);
                        delay(1000);
                    }

                    if (!arm.magneticBomb() && !loopFlag) {
                        arm.grasp();
                        delay(500);
                        arm.dropInBasket(RIGHT_DROPOFF_ANGLE);
                    } else if (!loopFlag) {
                        display.clear();
                        display.write(0, "bomb detected");
                        delay(1000);
                    }
                }
                // reset for next idol
                arm.rotateBase(270);
                arm.moveInPlaneElbowFirst(20, 35);
                arm.moveInPlaneElbowFirst(20, 35);
                advanceState();
                signalSlaveAdvance();

                beforeTime = millis();
            }

            break;
        }

        case (MasterState::FourthIdol): {
            if (millis() - beforeTime > 7500) {
                clawServo.write(CLAW_OPEN_ANGLE);
                display.clear();
                display.write(0, "Fourth idol state");
                arm.moveInPlaneShoulderFirst(18, 35);
                arm.rotateBase(315);
                arm.rotateBase(315);
                arm.moveInPlaneShoulderFirst(14, 34);
                int loopFlag = 1;

                double slope = ((double)(BASE_ONE_EIGHTY - BASE_NINETY)) / (double)(180 - 90);
                int targetValue = ((double)180 * slope) - (slope * 90 - BASE_NINETY);

                while (analogRead(BASE_POT) > targetValue && loopFlag) {
                    baseServo.write(75);
                    if (idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                        loopFlag = 0;
                    }
                }

                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

                if (!loopFlag) {
                    display.clear();
                    display.write(0, "treasure detected");
                    arm.moveInPlaneShoulderFirst(16, 30);
                    delay(1000);
                }

                if (!arm.magneticBomb() && !loopFlag) {
                    arm.grasp();
                    delay(500);
                    arm.dropInBasket(RIGHT_DROPOFF_ANGLE);
                } else if (!loopFlag) {
                    display.clear();
                    display.write(0, "bomb detected");
                    delay(1000);
                } else {
                    arm.moveInPlaneShoulderFirst(10, 32);
                    targetValue = ((double)315 * slope) - (slope * 90 - BASE_NINETY);
                    loopFlag = 1;

                    while (analogRead(BASE_POT) < targetValue && loopFlag) {
                        baseServo.write(86);
                        if (idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                            loopFlag = 0;
                        }
                    }
                    pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

                    if (!loopFlag) {
                        display.clear();
                        display.write(0, "treasure detected");
                        arm.moveInPlaneShoulderFirst(14, 30);
                        delay(1000);
                    }

                    if (!arm.magneticBomb() && !loopFlag) {
                        arm.grasp();
                        delay(500);
                        arm.dropInBasket(RIGHT_DROPOFF_ANGLE);
                    } else if (!loopFlag) {
                        display.clear();
                        display.write(0, "bomb detected");
                        delay(1000);
                    }
                }

                advanceState();
            }

            break;
        }
    }
}

double idolDetect(int numReadings) {
    int sum = 0;
    int reading;

    for (int i = 0; i < numReadings; i++) {
        reading = verticalSonar.ping_cm();
        if (reading == 0) {
            sum += 400;
        } else {
            sum += reading;
        }
    }
    return (double)(sum) / numReadings;
}

double idolDetectHorizontal(int numReadings) {
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
    return (double)(sum) / numReadings;
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