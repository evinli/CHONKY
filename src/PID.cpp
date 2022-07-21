/**
 * @file      PID.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the PID feedback-control
 */

#include "PID.h"

/////////////////// CONSTRUCTORS ///////////////////
PID::PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, OLED* display) {
    motorSpeed, lastError, P, I, D = 0;
    this->pidType = pidType;
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->display = display;

    switch (pidType) {
        case PIDType::TapeFollower: 
            pinMode(LEFT_TAPE_SENSOR, INPUT);
            pinMode(CENTER_TAPE_SENSOR, INPUT);
            pinMode(RIGHT_TAPE_SENSOR, INPUT);
            KP, KD, KI = 0;
            break;
        
        // case PIDType::EdgeFollower: 
        //     // leftSensor = LEFT_EDGE_SENSOR;
        //     // rightSensor = RIGHT_EDGE_SENSOR;
        //     break;
        
        default: 
            // should never get here
            break;
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

int PID::usePID() {
    // get reflectance sensor readings
    int leftReading = getAvgAnalogValue(LEFT_TAPE_SENSOR, TAPE_NUM_READINGS);
    int centreReading = getAvgAnalogValue(CENTER_TAPE_SENSOR, TAPE_NUM_READINGS);
    int rightReading = getAvgAnalogValue(RIGHT_TAPE_SENSOR, TAPE_NUM_READINGS);
    int error;

    // display readings
    display->clear();
    display->write(0, "Left Reading:" + std::to_string(leftReading));
    display->write(15, "Centre Reading:" + std::to_string(centreReading));
    display->write(30, "Right Reading:" + std::to_string(rightReading));

    switch (pidType) {
        case PIDType::TapeFollower: 
            bool leftOnWhite = sensorOnWhite(leftReading, TAPE_WHITE_THRESHOLD);
            bool centreOnWhite = sensorOnWhite(centreReading, TAPE_WHITE_THRESHOLD);
            bool rightOnWhite = sensorOnWhite(rightReading, TAPE_WHITE_THRESHOLD);
            error = getTapeError(leftOnWhite, centreOnWhite, rightOnWhite);
            display->write(45, "Error:" + std::to_string(error));
            if (error == T_STOP) {
                return error;
            }
            break;

        // case PIDType::EdgeFollower:
        //     break;
        
        // default:
        //     hello
    }

    P = error;
    I += error;
    D = error - lastError; 
    lastError = error;

    // (+) modMotorSpeed = tilting right = correct to the left
    // (-) modMotorSpeed = tilting left = correct to the right
    int modMotorSpeed = P*KP + I*KI + D*KD;
    int leftMotorSpeed = motorSpeed - modMotorSpeed; 
    int rightMotorSpeed = motorSpeed + modMotorSpeed; 

    // set new motor speeds
    leftMotor->setSpeed(leftMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);
    return error;
}

bool PID::sensorOnWhite(int reading, int threshold) {
    if (reading < threshold) {
        return true;
    }
    return false;
}

int PID::getTapeError(bool leftOnWhite, bool centreOnWhite, bool rightOnWhite) {
    // TRUTH TABLE
    // WHITE, WHITE, WHITE, lastError > 0: error = three off 
    // BLACK, WHITE, WHITE:                error = two off
    // BLACK, BLACK, WHITE:                error = one off
    // WHITE, BLACK, WHITE:                error = none off
    // WHITE, BLACK, BLACK:                error = -one off
    // WHITE, WHITE, BLACK:                error = -two off
    // WHITE, WHITE, WHITE, lastError < 0: error = -three off

    int error;
    if (leftOnWhite && centreOnWhite && rightOnWhite) {
        if (lastError > 0) {
            error = TAPE_THREE_OFF; // lost tape completely
        }
        else if (lastError < 0) {
            error = -TAPE_THREE_OFF; // lost tape completely
        }
    } 
    else if (!leftOnWhite && centreOnWhite && rightOnWhite) error = TAPE_TWO_OFF;
    else if (leftOnWhite && centreOnWhite && !rightOnWhite) error = -TAPE_TWO_OFF;
    else if (!leftOnWhite && !centreOnWhite && rightOnWhite) error = TAPE_ONE_OFF;
    else if (leftOnWhite && !centreOnWhite && !rightOnWhite) error = -TAPE_ONE_OFF;
    else if (leftOnWhite && !centreOnWhite && rightOnWhite) error = TAPE_ON;
    else if (!leftOnWhite && !centreOnWhite && !rightOnWhite) error = T_STOP;
    else error = TAPE_ON;

    return error;
}


