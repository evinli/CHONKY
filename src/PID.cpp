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
            // leftSensorPin = LEFT_TAPE_SENSOR;
            // centreSensorPin = CENTER_TAPE_SENSOR;
            // rightSensorPin = RIGHT_TAPE_SENSOR;
            // threshold = TAPE_WHITE_THRESHOLD;
            // numReadings = TAPE_NUM_READINGS;
            pinMode(LEFT_TAPE_SENSOR, INPUT);
            pinMode(CENTER_TAPE_SENSOR, INPUT);
            pinMode(RIGHT_TAPE_SENSOR, INPUT);
            KP = TAPE_KP;
            KD = TAPE_KD;
            KI = TAPE_KI;
            break;
        
        // case PIDType::EdgeFollower: 
        //     // leftSensor = LEFT_EDGE_SENSOR;
        //     // rightSensor = RIGHT_EDGE_SENSOR;
        //     break;
        
        // default: 
        //     // should never get here
        //     break;
    }
}

/////////////////// METHODS ///////////////////
// void PID::setThreshold(int threshold) {
//     this->threshold = threshold;
// }

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
            bool leftOnWhite = digitizeReading(leftReading, TAPE_WHITE_THRESHOLD);
            bool centreOnWhite = digitizeReading(centreReading, TAPE_WHITE_THRESHOLD);
            bool rightOnWhite = digitizeReading(rightReading, TAPE_WHITE_THRESHOLD);
            error = getTapeError(leftOnWhite, centreOnWhite, rightOnWhite);
            display->write(45, "Error:" + std::to_string(error));
            break;

        // case PIDType::EdgeFollower:
        //     break;
        
        // default:
        //     // hello
        //     break;
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

    // Set new motor speeds
    Serial.printf("PID left motor speed: %d\n", leftMotorSpeed);
    Serial.printf("PID right motor speed): %d\n", rightMotorSpeed);
    leftMotor->setSpeed(leftMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);

    // // Testing purposes only
    // display->clear();
    // if (leftMotorSpeed < rightMotorSpeed) {
    //     display->write(0, "Turning left");
    // }
    // else if (leftMotorSpeed > rightMotorSpeed) {
    //     display->write(0, "Turning right");
    // } else {
    //     display->write(0, "Going straight"); 
    // }
}

bool PID::digitizeReading(int reading, int threshold) {
    if (reading < threshold) {
        return true;
    }
    return false;
}

int PID::getTapeError(bool leftOnWhite, bool centreOnWhite, bool rightOnWhite) {
    // TRUTH TABLE
    // leftOnWhite, !centreOnWhite, rightOnWhite: error = 0
    // !leftOnWhite, !centreOnWhite, !rightOnWhite: error = 0
    // leftOnWhite, centreOnWhite, !rightOnWhite: error = -1
    // leftOnWhitee, centreOnWhite, rightOnWhite, lastError = -1: error = -5
    // !leftOnWhite, centreOnWhite, rightOnWHite: error = 1
    // leftOnWhitee, centreOnWhite, rightOnWhite, lastError = 1: error = 5

    // bad code, need to optimize
    int error = 0;
    if (leftOnWhite && !centreOnWhite && rightOnWhite) error = TAPE_ON;
    else if (leftOnWhite && centreOnWhite && !rightOnWhite) error = -TAPE_ONE_OFF;
    else if (!leftOnWhite && centreOnWhite && rightOnWhite) error = TAPE_ONE_OFF;
    else if (leftOnWhite && centreOnWhite && rightOnWhite) {
        if (lastError > 0) {
            error = TAPE_BOTH_OFF; // lost tape completely
        }
        else if (lastError < 0) {
            error = -TAPE_BOTH_OFF; // lost tape completely
        }
    }
    else error = TAPE_ON;

    return error; // chicken wire
}


