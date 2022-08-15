/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine implementation for the slave Bluepill
 */

#include "slave.h"

/////////////////// CONSTRUCTORS ///////////////////
Slave::Slave(Motor* leftMotor, Motor* rightMotor, PID* tapeFollow, PID* irFollow) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->tapeFollow = tapeFollow;
    this->irFollow = irFollow;
    currentState = SlaveState::Inactive;
    lastAdvanceTime = millis();
    slaveEnabled = true;
    alreadyStopped = false;
    pinMode(SLAVE_ADVANCE_STATE, INPUT_PULLDOWN);
    pinMode(SLAVE_STOP_DRIVE, INPUT_PULLDOWN);
}

/////////////////// METHODS ///////////////////
void Slave::determineState() {
    // Reset slave enabled flag
    slaveEnabled = true;

    // Check comms pins 
    if (digitalRead(SLAVE_ADVANCE_STATE) == HIGH) {
        advanceState();
    }
    if (digitalRead(SLAVE_STOP_DRIVE) == HIGH) {
        if (alreadyStopped == true) {
            leftMotor->stop();
            rightMotor->stop();
        }
        else {
            leftMotor->hardStop(FORWARDS_DIR);
            rightMotor->hardStop(FORWARDS_DIR);
            alreadyStopped = true;
        }
        slaveEnabled = false;
    }
    
    // Run state machine
    if (slaveEnabled) {
        alreadyStopped = false;
        switch(currentState) {
            case(SlaveState::Inactive): {
                break;
            }

            case(SlaveState::TapeFollow): {
                tapeFollow->setMotorSpeed(150);
                tapeFollow->setKP(16);
                tapeFollow->setKD(10);
                tapeFollow->setKI(0);
                tapeFollow->usePID();
                break;
            }

            case(SlaveState::RefindTapePostFirstIdol): {             
              if (tapeFollow->refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow->resetPID();
                    advanceState();
                }
                break;
            }
            
            case(SlaveState::ChickenWire): {
                tapeFollow->setMotorSpeed(100);
                tapeFollow->setKP(12);
                tapeFollow->setKD(6);
                tapeFollow->setKI(0);

                // Chicken wire routine
                if (tapeFollow->usePID() == ALL_HIGH) {
                    moveForTime(160, 160, 700);
                    tapeFollow->resetPID(); 
                    advanceState();
                }
                break;
            }

            case(SlaveState::RefindTapePostChickenWire): {
                if (tapeFollow->refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow->resetPID();
                    advanceState();
                } 
                else {
                    if (tapeFollow->refindTape(RIGHT_SIDE, MAX_SWEEP_TIME * 2)) {
                        tapeFollow->resetPID();
                        advanceState();
                    }
                }
                break;
            }

            case(SlaveState::TapeFollowPostRamp): {
                tapeFollow->setMotorSpeed(120);
                tapeFollow->setKP(12);
                tapeFollow->setKD(6);
                tapeFollow->setKI(0);
                tapeFollow->usePID();
                break;
            }

            case(SlaveState::RefindTapePostSecondIdol): {
                if (tapeFollow->refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow->resetPID();
                    advanceState();
                }
                break;
            }

            case(SlaveState::NavigateArchway): {
                moveForTime(-150, -150, 300);
                leftMotor->hardStop(BACKWARDS_DIR);
                rightMotor->hardStop(BACKWARDS_DIR);
                if (tapeFollow->refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow->resetPID();
                    lastEventTime = millis();
                    advanceState();
                }
                break;
            }

            case(SlaveState::Archway): {
                tapeFollow->setMotorSpeed(130);
                tapeFollow->setKP(16);
                tapeFollow->setKD(6);
                tapeFollow->setKI(0);

                // Pause briefly at T-stop
                if (tapeFollow->usePID() == ALL_HIGH) {
                    leftMotor->stop();
                    rightMotor->stop();
                    delay(1000);
                    tapeFollow->resetPID();

                    // Make adjustment to fit through archway
                    moveForTime(-150, -150, 180);
                    moveForTime(-70, 120, 100);
                    advanceState();
                    lastEventTime = millis();
                } 
                break;
            }

            case(SlaveState::IRFollow): {
                if (millis() - lastEventTime < IR_FOLLOW_ARCHWAY) { 
                    irFollow->setMotorSpeed(120);
                    irFollow->setKP(18);
                    irFollow->setKD(12);
                    irFollow->setKI(0);
                    irFollow->usePID();
                } else {
                    leftMotor->hardStop(FORWARDS_DIR);
                    rightMotor->hardStop(FORWARDS_DIR);
                    advanceState();
                }
                break;
            }

            case(SlaveState::DriveToThirdIdol): {
                // Turn to face third idol
                moveForTime(-120, 150, 1000);
                leftMotor->hardStop(BACKWARDS_DIR);
                rightMotor->hardStop(FORWARDS_DIR);

                // Drive closer
                moveForTime(120, 120, 500);
                leftMotor->hardStop(FORWARDS_DIR);
                rightMotor->hardStop(FORWARDS_DIR);
                advanceState();
                break;
            }

            case(SlaveState::WaitForThirdIdolPickup): {
                break;           
            }
            
            case(SlaveState::RefindIRPostThirdIdol): {
                // Reverse movements from the last state
                moveForTime(-120, -120, 500);
                leftMotor->hardStop(BACKWARDS_DIR);
                rightMotor->hardStop(BACKWARDS_DIR);
                moveForTime(120, -150, 1550);
                leftMotor->hardStop(FORWARDS_DIR);
                rightMotor->hardStop(BACKWARDS_DIR);

                advanceState();
                lastEventTime = millis();
                break;
            }

            case(SlaveState::IRFollowToFourthIdol): {
                if (millis() - lastEventTime < IR_FOLLOW_IDOL) {
                    irFollow->setMotorSpeed(120);
                    irFollow->setKP(15);
                    irFollow->setKD(9);
                    irFollow->setKI(0);
                    irFollow->usePID();
                }
                else {
                    leftMotor->hardStop(FORWARDS_DIR);
                    rightMotor->hardStop(FORWARDS_DIR);
                    advanceState();
                }
                break;
            }

            case(SlaveState::Done): {
                leftMotor->stop();
                rightMotor->stop();
                break;
            }
        }
    }  
}

void Slave::moveForTime(int leftMotorSpeed, int rightMotorSpeed, int timeMillis) {
    leftMotor->setSpeed(leftMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);
    delay(timeMillis);
}

void Slave::advanceState() {
    // Advance only if going into valid state and we aren't accidentally advancing multiple times
    if (millis() - lastAdvanceTime >= MIN_ADVANCE_TIME && currentState != SlaveState::Done) { 
        lastAdvanceTime = millis();
        currentState = static_cast<SlaveState>(static_cast<int>(currentState)+ 1);
    }
}
