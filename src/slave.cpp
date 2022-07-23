/**
 * @file      slave.cpp
 * @author    Creators of CHONKY 
 * @brief     State machine implementation for the slave Bluepill
 */

#include "slave.h"

/////////////////// CONSTRUCTORS ///////////////////
Slave::Slave(Motor* leftMotor, Motor* rightMotor, PID* tapeFollow, OLED* display) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->tapeFollow = tapeFollow; 
    this->display = display;
    state = SlaveState::Inactive;
}

///////////////// METHODS ///////////////////
void Slave::determineState() {
    if (digitalRead(SLAVE_ADVANCE_STATE) == HIGH) {
        advanceState();
    }

    if (digitalRead(SLAVE_STOP_DRIVE) == HIGH) {
        leftMotor->stop();
        rightMotor->stop();
    }

    switch(state) {
        case SlaveState::Inactive: 
            leftMotor->stop();
            rightMotor->stop();
            break;
        
        case SlaveState::TapeFollowing:
            tapeFollow->setMotorSpeed(70);
            tapeFollow->setKP(8);
            tapeFollow->setKD(3);
            tapeFollow->setKI(0);
            if (tapeFollow->usePID() == T_STOP) {
                leftMotor->stop();
                rightMotor->stop();
            }
            advanceState();
            break;
        
        // case SlaveState::Archway:


        // case SlaveState::IRFollowing:
        

        // case SlaveState::EdgeFollowing:

    }
}

bool Slave::advanceState() {
    // TODO: need some checks here so that we don't advance states multiple times accidentally
    state = static_cast<SlaveState>(state + 1);
}
