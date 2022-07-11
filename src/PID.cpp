/**
 * @file      PID.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the PID feedback-control
 */

#include "PID.h"

/////////////////// CONSTRUCTORS ///////////////////
PID::PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, OLED* display) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->display = display;

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
    int error = getAvgAnalogValue(leftSensor, 1) - getAvgAnalogValue(rightSensor, 1);
    Serial.printf("PID error: %d\n", error);
    P = error;
    I += error;
    D = error - previousError;
    previousError = error;

    // (+) modMotorSpeed = tilting right = correct to the left
    // (-) modMotorSpeed = tilting left = correct to the right
    int modMotorSpeed = P*KP + I*KI + D*KD;
    int leftMotorSpeed = motorSpeed - modMotorSpeed; 
    int rightMotorSpeed = motorSpeed + modMotorSpeed; 

    // Set new motor speeds
    Serial.printf("PID left motor speed: %d\n", leftMotorSpeed);
    Serial.printf("PID right motor speed): %d\n", rightMotorSpeed);
    leftMotor->setSpeed(leftMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);

    // Testing purposes only
    display->clear();
    if (leftMotorSpeed < rightMotorSpeed) {
        display->write(0, "Turning left");
    }
    else if (leftMotorSpeed > rightMotorSpeed) {
        display->write(0, "Turning right");
    } else {
        display->write(0, "Going straight");
    }
}


