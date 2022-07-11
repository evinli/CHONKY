/**
 * @file      PID.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the PID feedback-control
 */

#include "PID.h"

/////////////////// CONSTRUCTORS ///////////////////
PID::PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor) {
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
void PID::setMotorSpeed(int motorSpeed) {
    this->motorSpeed = motorSpeed;
}

void PID::setKP(float KP) {
    this->KP = KP;
}

void PID::setKD(float KD) {
     this->KD = KD;
}
    
void PID::setKI(float KI) {
     this->KI = KI;
}

void PID::usePID() {
    // int error = getAvgAnalogValue(leftSensor, 1) - getAvgAnalogValue(rightSensor, 1);
    int error = 2890;
    P = error;
    I += error;
    D = error - previousError;
    previousError = error;

    int modMotorSpeed = P*KP + I*KI + D*KD;
    int leftMotorSpeed = motorSpeed - modMotorSpeed; // decrease left motor speed if tilting to the right, vice versa
    int rightMotorSpeed = motorSpeed + modMotorSpeed; // increase right motor speed if tilting to the right, vice versa

    Serial.printf("PID left motor speed (no limit): %d\n", leftMotorSpeed);
    leftMotor->setSpeed(leftMotorSpeed);

    Serial.printf("PID left motor speed (no limit): %d\n", rightMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);
}


