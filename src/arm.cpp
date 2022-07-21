/**
 * @file      arm.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the arm class
 */

#include "arm.h"

/////////////////// CONSTRUCTORS ///////////////////
Arm::Arm(Motor* shoulder, Servo* elbow, Servo* claw) {
    this->shoulder = shoulder;
    this->elbow = elbow;
    this->claw = claw;
    elbow->attach(ELBOW_SERVO);
    claw->attach(CLAW_SERVO);
}   

/////////////////// METHODS ///////////////////
bool Arm::moveElbow(int angle) {

}

bool Arm::moveShoulder(int angle) {
}

bool Arm::grabTreasure() {

}