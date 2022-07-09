/**
 * @file      PID.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the PID feedback-control
 */

#include "PID.h"

/////////////////// CONSTRUCTORS ///////////////////
PID::PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, int baseSpeed) {
    this->baseSpeed = baseSpeed;
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;

    switch (pidType) {
        case PIDType::TapeFollower: {
            leftSensor = LEFT_TAPE_SENSOR;
            rightSensor = RIGHT_TAPE_SENSOR;
            break;
        }
        case PIDType::EdgeFollower: {
            leftSensor = LEFT_EDGE_SENSOR;
            rightSensor = RIGHT_EDGE_SENSOR;
            break;
        }
        default: {
            // should never get here
            break;
        }
    }
}

/////////////////// METHODS ///////////////////
void PID::setKP(int KP) {
    this->KP = KP;
}

void PID::setKD(int KD) {
     this->KP = KD;
}
    
void PID::setKI(int KI) {
     this->KP = KI;
}

void PID::usePID() {
    int error = getAvgAnalogValue(leftSensor, 1) - getAvgAnalogValue(rightSensor, 1);
    P = error;
    I += error;
    D = error - previousError;
    previousError = error;

    int modMotorSpeed = P*KP + I*KI + D*KD;
    int leftMotorSpeed = baseSpeed - modMotorSpeed; // decrease left motor speed if tilting to the right, vice versa
    int rightMotorSpeed = baseSpeed + modMotorSpeed; // increase right motor speed if tilting to the right, vice versa

    leftMotor->setSpeed(leftMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);
}


