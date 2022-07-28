/**
 * @file      arm.cpp
 * @author    Creators of CHONKY
 * @brief     Class implementation for the arm class
 */

#include "arm.h"

/////////////////// CONSTRUCTORS ///////////////////
Arm::Arm(Motor* shoulder, Servo* elbow, Servo* claw, Servo* base, int shoulderSpeed) {
    this->shoulder = shoulder;
    this->elbow = elbow;
    this->claw = claw;
    this->base = base;
    this->shoulderSpeed = shoulderSpeed;
}

/////////////////// PUBLIC METHODS ///////////////////
void Arm::moveInPlane(double distanceFromChassis, double heightAboveGround) {
    double hypotenuse = getHypotenuse(heightAboveGround, distanceFromChassis);
    double phi = getPhi(hypotenuse);
    double theta = getTheta(hypotenuse, phi);
    double alpha = getAlpha(heightAboveGround, distanceFromChassis);
    double shoulderJointAngle = alpha + theta;
    moveShoulderJoint(shoulderJointAngle);
    elbow->write(180 - phi);
}

bool Arm::grabTreasure() {
    return true;
}

void Arm::moveShoulderJoint(int angle) {
    int potValue = analogRead(SHOULDER_POT);
    int inputValue = angle*(SHOULDER_NINETY - SHOULDER_ZERO)/(SHOULDER_RANGE) + SHOULDER_ZERO; // input angle in terms of the potentiometer values

    while (abs(potValue - inputValue) > POT_MOTOR_ERROR) { 
        if (potValue < inputValue) {
            shoulder->setSpeed(shoulderSpeed - SHOULDER_SPEED_OFFSET);
            while (potValue < inputValue) {
                potValue = analogRead(SHOULDER_POT);
            }
        }
        else if (potValue > inputValue) {
            shoulder->setSpeed(-shoulderSpeed);
            while (potValue > inputValue) {
                potValue = analogRead(SHOULDER_POT);
            }
        }
    }
    shoulder->stop();
}

void Arm::rotateBase(int angle){
    int potValue = analogRead(BASE_POT);
    int targetValue = angle*(-1023)/(350 - 16) + 1072; // TO BE MADE TO CONSTANTS

    while (abs(potValue - targetValue) > POT_MOTOR_ERROR) { 
        if (potValue < targetValue) {
            base->write(75); // change into constants
            while (potValue < targetValue) {
                potValue = analogRead(BASE_POT);
            }
        }
        else if (potValue > targetValue) {
            base->write(90); // change into constants
            while (potValue > targetValue) {
                potValue = analogRead(BASE_POT);
            }
        }
    }

    base->write(BASE_SERVO_STOP_ANGLE);
}

// void Arm::sweep(double startingDist, double endingDist, double height){
//     for(double i=startingDist;i<endingDist;i+SWEEP_STEP_SIZE){
//         moveInPlane(i,height);
//     }
// }

/////////////////// PRIVATE METHODS ///////////////////
double Arm::getHypotenuse(double heightAboveGround, double distanceFromChassis) {
    double y = heightAboveGround - SHOULDER_HEIGHT;
    double x = distanceFromChassis + SHOULDER_CHASSIS_EDGE_DIST; 
    return sqrt(pow(x, 2) + pow(y, 2));
}


double Arm::getPhi(double hypotenuse) {
        double l1Squared = pow(BICEP_LENGTH, 2);
        double l2Squared = pow(FOREARM_LENGTH, 2);
        double l3Squared = pow(hypotenuse, 2);
        double acosOut = acos((l1Squared + l2Squared - l3Squared) / (2 * FOREARM_LENGTH * BICEP_LENGTH));
        return (RAD_TO_DEG * acosOut);
}

double Arm::getTheta(double hypotenuse, double phi) {
    double aSinVal = asin((FOREARM_LENGTH * sin(phi * RAD_TO_DEG)) / hypotenuse);
    return (RAD_TO_DEG * aSinVal);
}

double Arm::getAlpha(double heightAboveGround, double distanceFromChassis) {
    double x = distanceFromChassis + SHOULDER_CHASSIS_EDGE_DIST; 
    double y = heightAboveGround - SHOULDER_HEIGHT;
    return (RAD_TO_DEG * atan(y/x));
}
