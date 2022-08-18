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
            arm->rotateBase(BASE_IDOL_DETECT_ANGLE);
            arm->moveInPlaneElbowFirst(IDOL_DETECTION_DIST, IDOL_DETECTION_HEIGHT);
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
                    arm->rotateBase(BASE_IDOL_DETECT_ANGLE+DETECT_ANGLE_GRASP_OFFSET);    
                    arm->graspSequence(IDOL_DETECTION_DIST,IDOL_GRASP_HEIGHT, RIGHT_DROPOFF_ANGLE);

                    // Reset arm for next idol
                    arm->rotateBase(BASE_IDOL_DETECT_ANGLE); 
                    advanceState();
                    signalSlaveAdvance();
                    enableSlave();
                }
            }
            break;
        }

        case (MasterState::PostFirstIdol): {
            // wait to avoid detecting the first idol again in case it is a bomb
            delay(5000);
            arm->moveInPlaneElbowFirst(IDOL_DETECTION_DIST, IDOL_DETECTION_HEIGHT);
            advanceState();
            break;
        }

        case (MasterState::SecondIdol): {
            if (arm->idolDetect(IDOL_DETECT_SAMPLES) < 10) {  
                disableSlave();
                arm->rotateBase(BASE_IDOL_DETECT_ANGLE+DETECT_ANGLE_GRASP_OFFSET);
                arm->graspSequence(IDOL_DETECTION_DIST,IDOL_GRASP_HEIGHT,RIGHT_DROPOFF_ANGLE);

                // Move arm into resting position
                arm->goToRestingPos();

                advanceState();
                signalSlaveAdvance();
                enableSlave();
                lastEventTime = millis(); 
            }
            break;
        }

        case (MasterState::ThirdIdol): {

            // wait to get past the archway
            if (millis() - lastEventTime > 7500) {
                arm->rotationalSweep(270,135,10,20,IDOL_DETECTION_HEIGHT,RIGHT_DROPOFF_ANGLE,2); //suitable angular and distance values found through testing

                // Move arm into next idol detect position
                arm->rotateBase(BASE_IDOL_DETECT_ANGLE);
                arm->moveInPlaneElbowFirst(IDOL_DETECTION_DIST, IDOL_DETECTION_HEIGHT);

                advanceState();
                signalSlaveAdvance();
                lastEventTime = millis();
            }
            break;
        }

        case (MasterState::FourthIdol): {
            if (millis() - lastEventTime > 6000) {
                arm->rotationalSweep(315,180,10,20,IDOL_DETECTION_HEIGHT,RIGHT_DROPOFF_ANGLE,2); //suitable angular and distance values found through testing
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
