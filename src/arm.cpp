/**
 * @file      arm.cpp
 * @author    Creators of CHONKY
 * @brief     Class implementation for the arm class
 */

#include "arm.h"

/////////////////// CONSTRUCTORS ///////////////////
Arm::Arm(Motor *shoulder, ServoP *elbow, ServoP *claw, ServoP *base, int shoulderSpeed)
{
    this->shoulder = shoulder;
    this->elbow = elbow;
    this->claw = claw;
    this->shoulderSpeed = shoulderSpeed;
    this->base= base;
}

/////////////////// METHODS ///////////////////
void Arm::moveInPlane(double distanceFromChassis, double heightAboveGround) {
    double l3=getL3(heightAboveGround, distanceFromChassis);
    double phi=getPhi(l3);
    double theta=getTheta(l3,phi);
    double alpha=getAlpha(heightAboveGround, distanceFromChassis);
    double shoulderJointAngle=alpha+theta;
    moveShoulderJoint(shoulderJointAngle);
    elbow->write(180-phi);
    delay(500);
}

bool Arm::grabTreasure(){
    return true;
}

void Arm::moveShoulderJoint(int angle)
{
    int potValue = analogRead(SHOULDER_POT);
    int inputValue = (angle * (-380) / 90) + 620; // This is the input angle in terms of the potentiometer values

    while (abs(potValue - inputValue) > 5) { 
        if (potValue < inputValue) {
            shoulder->setSpeed(shoulderSpeed);
            while (potValue < inputValue)
            {
                potValue = analogRead(SHOULDER_POT);
            }
        }
        else if (potValue > inputValue) {
            shoulder->setSpeed(-shoulderSpeed);
            while (potValue > inputValue)
            {
                potValue = analogRead(SHOULDER_POT);
            }
        }
    }
    shoulder->stop();
}

double Arm:: getL3(double heightAboveGround, double distanceFromChassis){
    double y = heightAboveGround-SHOULDER_HEIGHT;
    double x = distanceFromChassis+SHOULDER_CHASSIS_EDGE_DIST; 
    return sqrt(pow(x,2)+pow(y,2));
}


double Arm::getPhi(double l3)
    {
        double radianConversion = double(180) / PI;
        double l1Squared = pow(BICEP_LENGTH, 2);
        double l2Squared = pow(FOREARM_LENGTH, 2);
        double l3Squared = pow(l3, 2);
        double acosOut = acos((l1Squared + l2Squared - l3Squared) / (2 * FOREARM_LENGTH * BICEP_LENGTH));
        return ((double(180) / PI) * (acosOut));
    }

double Arm::getTheta(double l3, double phi){
    double aSinVal=asin((FOREARM_LENGTH*sin((phi)*(double(PI)/180)))/l3);
    return((double(180)/PI)*aSinVal);
}

double Arm::getAlpha(double heightAboveGround, double distanceFromChassis){
    double y = heightAboveGround-SHOULDER_HEIGHT;
    double x = distanceFromChassis+SHOULDER_CHASSIS_EDGE_DIST; 
    return ((double(180)/PI)*atan(y/x));
}

void Arm::rotateBase(int angle){
    int potValue = analogRead(BASE_POT);
    int targetValue = (angle * (1023) /(348.7-22.3)) -69.89;

    while (abs(potValue - targetValue) > 5) { 
        if (potValue < targetValue) {
            base->write(90);
            while (potValue < targetValue)
            {
                potValue = analogRead(BASE_POT);
            }
        }
        else if (potValue > targetValue) {
            base->write(75);
            while (potValue > targetValue)
            {
                potValue = analogRead(BASE_POT);
            }
        }
    }
    base->write(83);
}