/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine implementation for the slave Bluepill
 */

#include "master.h"

/////////////////// CONSTRUCTORS ///////////////////
Master::Master(Arm* arm) {
    this->arm = arm;
    currentState = MasterState::Inactive;
    pinMode(SLAVE_ADVANCE_STATE, INPUT_PULLDOWN);
    pinMode(SLAVE_STOP_DRIVE, INPUT_PULLDOWN);
}

/////////////////// METHODS ///////////////////
void Master::determineState() {
    endSlaveAdvanceSignal();

    switch (currentState) {
        case (MasterState::Inactive): {
            // Set arm to first idol detect position
            arm->claw->write(CLAW_OPEN_ANGLE);
            arm->rotateBase(270);
            arm->moveInPlaneElbowFirst(8, 32);
            delay(1000);
            advanceState();
            signalSlaveAdvance();
            break;
        }

        case (MasterState::FirstIdol): {
            if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 15) {  
                if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 15) { // second verification
                    disableSlave();
                    
                    // Move into grasp position
                    arm->rotateBase(280);                
                    arm->moveInPlaneElbowFirst(13, 30);  
                    delay(1000); 

                    // Check if idol contains bomb
                    if (!arm->magneticBomb()) {
                        arm->grasp();
                        delay(500);
                        arm->dropInBasket(RIGHT_DROPOFF_ANGLE);
                    } else {
                        delay(1000);
                        arm->moveInPlaneShoulderFirst(13, 37);
                    }

                    // Reset arm for next idol
                    arm->rotateBase(270);
                    arm->moveInPlaneShoulderFirst(13, 37);
                    arm->claw->write(60);  
                    advanceState();
                    signalSlaveAdvance();
                    enableSlave();
                }
            }
            break;
        }

        case (MasterState::PostFirstIdol): {
            delay(5000);
            arm->moveInPlaneElbowFirst(12, 33);
            advanceState();
            break;
        }

        case (MasterState::SecondIdol): {
            if (arm->idolDetect(20) < 10) {  
                disableSlave();
                arm->rotateBase(278);
                arm->moveInPlaneElbowFirst(14, 30); 
                delay(1000);

                if (!arm->magneticBomb()) {
                    arm->grasp();
                    delay(500);
                    arm->dropInBasket(RIGHT_DROPOFF_ANGLE);
                } else {
                    delay(1000);
                    arm->moveInPlaneShoulderFirst(13, 37);
                }

                // Move arm into resting position
                arm->goToRestingPos();
                arm->base->write(BASE_CW_SPEED);
                delay(250);
                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

                advanceState();
                signalSlaveAdvance();
                enableSlave();
                lastEventTime = millis();
            }
            break;
        }

        case (MasterState::ThirdIdol): {
            if (millis() - lastEventTime > 7500) {
                arm->moveInPlaneShoulderFirst(16, 35);
                arm->claw->write(CLAW_OPEN_ANGLE);
                delay(1000);
                arm->moveInPlaneShoulderFirst(19, 35);
                arm->claw->write(CLAW_OPEN_ANGLE);
                arm->base->write(BASE_CCW_SPEED);
                delay(250);
                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
                arm->rotateBase(270);
                arm->rotateBase(270);
                arm->moveInPlaneShoulderFirst(12, 29);

                bool treasureDetected = false;
                double slope = ((double)(BASE_ONE_EIGHTY - BASE_NINETY)) / (double)(180 - 90);
                int targetValue = ((double)135 * slope) - (slope * 90 - BASE_NINETY);

                // Scan until idol detected
                while (analogRead(BASE_POT) > targetValue) {
                    arm->base->write(75);
                    if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                        treasureDetected = true;
                        break;
                    }
                }
                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
                arm->moveInPlaneShoulderFirst(12, 30);
                delay(1000);
                
                // If treasure detected, check if bomb; else, start second scan
                if (treasureDetected) {
                    if (!arm->magneticBomb()) {
                        arm->grasp();
                        delay(500);
                        arm->dropInBasket(RIGHT_DROPOFF_ANGLE);
                    } 
                    else {
                        delay(1000);
                    }
                }
                else {
                    arm->moveInPlaneShoulderFirst(17, 31);
                    targetValue = ((double)270 * slope) - (slope * 90 - BASE_NINETY);

                    while (analogRead(BASE_POT) < targetValue) {
                        arm->base->write(86); 
                        if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                            treasureDetected = true;
                            break;
                        }
                    }
                    pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
                    arm->moveInPlaneShoulderFirst(19, 30);
                    delay(1000);

                    // If treasure detected, check if bomb
                    if (treasureDetected) {
                        if (!arm->magneticBomb()) {
                            arm->grasp();
                            delay(500);
                            arm->dropInBasket(RIGHT_DROPOFF_ANGLE);
                        }
                        else {
                            delay(1000);
                        }
                    }      
                }

                // Move arm into next idol detect position
                arm->rotateBase(270);
                arm->moveInPlaneElbowFirst(20, 35);
                advanceState();
                signalSlaveAdvance();
                lastEventTime = millis();
            }
            break;
        }

        case (MasterState::FourthIdol): {
            if (millis() - lastEventTime > 7500) {
                arm->claw->write(CLAW_OPEN_ANGLE);
                arm->moveInPlaneShoulderFirst(18, 35);
                arm->rotateBase(315);
                arm->moveInPlaneShoulderFirst(14, 34);
                
                bool treasureDetected = false;
                double slope = ((double)(BASE_ONE_EIGHTY - BASE_NINETY)) / (double)(180 - 90);
                int targetValue = ((double)180 * slope) - (slope * 90 - BASE_NINETY);

                while (analogRead(BASE_POT) > targetValue) {
                    arm->base->write(75);
                    if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                        treasureDetected = true;
                        break;
                    }
                }
                pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
                arm->moveInPlaneShoulderFirst(16, 30);
                delay(1000);

                // If treasure detected, check if bomb; else, start second scan
                if (treasureDetected) {
                    if (!arm->magneticBomb()) {
                        arm->grasp();
                        delay(500);
                        arm->dropInBasket(RIGHT_DROPOFF_ANGLE);
                    } 
                    else {
                        delay(1000);
                    }
                }
                else {
                    arm->moveInPlaneShoulderFirst(10, 32);
                    targetValue = ((double)315 * slope) - (slope * 90 - BASE_NINETY);

                    while (analogRead(BASE_POT) < targetValue) {
                        arm->base->write(86);
                        if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 15) {
                            treasureDetected = true;
                            break;
                        }
                    }
                    pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
                    arm->moveInPlaneShoulderFirst(14, 30);
                    delay(1000);
                
                    // If treasure detected, check if bomb
                    if (treasureDetected) {
                        if (!arm->magneticBomb()) {
                            arm->grasp();
                            delay(500);
                            arm->dropInBasket(RIGHT_DROPOFF_ANGLE);
                        }
                        else {
                            delay(1000);
                        }
                    }  
                }
                advanceState();
            }
            break;
        }

        case(MasterState::Done): {
            break;
        }
    }
}

void Master::advanceState() {
    if (currentState != MasterState::Done) {
        currentState = static_cast<MasterState>(static_cast<int>(currentState) + 1);
    }
}

void Master::signalSlaveAdvance() {
    digitalWrite(SLAVE_ADVANCE_STATE, HIGH);
    // Ensure slave has recieved signal to advance
    delay(100);
}

void Master::endSlaveAdvanceSignal() {
    digitalWrite(SLAVE_ADVANCE_STATE, LOW);
}

void Master::enableSlave() {
    digitalWrite(SLAVE_STOP_DRIVE, HIGH);
    // Ensure slave has fully stopped before performing any other tasks
    delay(500);
}

void Master::disableSlave() {
    digitalWrite(SLAVE_STOP_DRIVE, LOW);
}
