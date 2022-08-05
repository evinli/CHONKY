/**
 * @file      arm.cpp
 * @author    Creators of CHONKY
 * @brief     Class implementation for the arm class
 */

#include "arm.h"

/////////////////// CONSTRUCTORS ///////////////////
Arm::Arm(Motor* shoulder, Servo* elbow, Servo* claw, Servo* base, int shoulderSpeed, NewPing* verticalSonar, NewPing* horizontalSonar, OLED* display) {
    this->shoulder = shoulder;
    this->elbow = elbow;
    this->claw = claw;
    this->base = base;
    this->shoulderSpeed = shoulderSpeed;
    this->verticalSonar = verticalSonar;
    this->horizontalSonar = horizontalSonar;
    this->display = display;
    pinMode(SHOULDER_POT, INPUT);
    pinMode(BASE_POT, INPUT);
}

/////////////////// METHODS ///////////////////
void Arm::moveInPlaneShoulderFirst(double distanceFromChassis, double heightAboveGround) {
    double hypotenuse = getHypotenuse(heightAboveGround, distanceFromChassis);
    double phi = getPhi(hypotenuse);
    double theta = getTheta(hypotenuse, phi);
    double alpha = getAlpha(heightAboveGround, distanceFromChassis);
    double shoulderJointAngle = alpha + theta;
    moveShoulderJoint(shoulderJointAngle);
    elbow->slowWrite(180 - phi, 8);
}

void Arm::moveInPlaneElbowFirst(double distanceFromChassis, double heightAboveGround) {
    double hypotenuse = getHypotenuse(heightAboveGround, distanceFromChassis);
    double phi = getPhi(hypotenuse);
    double theta = getTheta(hypotenuse, phi);
    double alpha = getAlpha(heightAboveGround, distanceFromChassis);
    double shoulderJointAngle = alpha + theta;
    elbow->slowWrite(180 - phi, 8);
    moveShoulderJoint(shoulderJointAngle);
}

void Arm::grasp() {
    claw->write(CLAW_GRASP_ANGLE);
}

void Arm::moveShoulderJoint(int angle) {
    if (angle > 90 || angle < 0){
        return;
    }

    int potValue = analogRead(SHOULDER_POT);
    int inputValue = angle * (SHOULDER_NINETY - SHOULDER_ZERO) / (SHOULDER_RANGE) + SHOULDER_ZERO;  // input angle in terms of the potentiometer values

    while (abs(potValue - inputValue) > POT_MOTOR_ERROR) {
        if (potValue < inputValue) {
            shoulder->setSpeed(shoulderSpeed);
            while (potValue < inputValue) {
                potValue = analogRead(SHOULDER_POT);
            }
        } else if (potValue > inputValue) {
            shoulder->setSpeed(-shoulderSpeed);
            while (potValue > inputValue) {
                potValue = analogRead(SHOULDER_POT);
            }
        }
    }
    shoulder->stop();
}

void Arm::rotateBase(int angle) {
    int potValue = analogRead(BASE_POT);
    int targetValue = angle * (526 - 250) / (180 - 90) - 26;  // TO BE MADE TO CONSTANTS

    display->clear();
    display->write(0, std::to_string(targetValue));
    display->write(20, std::to_string(angle));

    while (abs(potValue - targetValue) > POT_MOTOR_ERROR) {
        if (potValue < targetValue) {
            base->write(90);  // change into constants
            while (potValue < targetValue) {
                potValue = analogRead(BASE_POT);
            }
        } else if (potValue > targetValue) {
            base->write(75);  // change into constants
            while (potValue > targetValue) {
                potValue = analogRead(BASE_POT);
            }
        }
    }

    // base->write(BASE_SERVO_STOP_ANGLE);
    pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
}

void Arm::dropInBasket(int dropOffSide) {
    moveShoulderJoint(90); // trial and error
    elbow->write(30); // trial and error
    rotateBase(dropOffSide);
    delay(500);
    claw->write(CLAW_OPEN_ANGLE);
    delay(1500);
}

void Arm::sweep(double startingDist, double endingDist, double height) {
    for (double i = startingDist; i < endingDist; i += SWEEP_STEP_SIZE) {
        moveInPlaneShoulderFirst(i, height);
    }
}

void Arm::sweepAndDetect(double startingDist, double endingDist, double height, int dropOffSide) {
    for (double i = startingDist; i < endingDist; i += SWEEP_STEP_SIZE) {
        moveInPlaneShoulderFirst(i, height);
        if (avgSampleSonar(CLAW_SCAN_SAMPLES, this->verticalSonar) < 10) {
            this->display->clear();
            this->display->write(0, "Claw Scan Triggered");
            graspSequence(i, height);
            dropInBasket(dropOffSide);
            break;
        }
    }
}

void Arm::graspSequence(double startingDist, double currentHeight) {
    // map distance away to additional distance required to travel (i.e. closer needs less distance)
    // TO CHANGE, THIS IS CURRENLY CONSTANT:
    double finalDist = startingDist + 6; // trial and error
    sweep(startingDist, finalDist, currentHeight + 1); // +1 because our height seems to drop as a sweep
    moveInPlaneShoulderFirst(finalDist, currentHeight - 3); // trial and error
    delay(1000); // make sure we're fully downright before we grasp
    grasp(); 
    delay(1000); // make sure we have idol in our grasp
}

void Arm::goToRestingPos() {
    // need to verify these values
    // rotateBase(MIDDLE_ANGLE);
    // elbow->slowWrite(0, 8);
    // moveShoulderJoint(0);
}

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
    double aSinVal = asin((FOREARM_LENGTH * sin(phi * DEG_TO_RAD)) / hypotenuse);
    return (RAD_TO_DEG * aSinVal);
}

double Arm::getAlpha(double heightAboveGround, double distanceFromChassis) {
    double x = distanceFromChassis + SHOULDER_CHASSIS_EDGE_DIST;
    double y = heightAboveGround - SHOULDER_HEIGHT;
    return (RAD_TO_DEG * atan(y / x));
}

double Arm::avgSampleSonar(int numReadings, NewPing* sonarSensor) {
    int sum = 0;

    for (int i = 0; i < numReadings; i++) {
        sum += sonarSensor->ping_cm();
    }

    return (double)(sum)/numReadings;
}

// TESTING STUFF
void Arm::testShoulder() {
    for (int i = 0; i < 7; i++) {
        moveShoulderJoint(i * 15);
        delay(2000);
    }
}

void Arm::testElbow() {
    for (int i = 0; i < 10; i++) {
        this->elbow->write(i * 15);
        delay(2000);
    }
}

void Arm::testBase() {
    for (int i = 2; i < 23; i++) {
        rotateBase(i * 15);
        delay(2000);
    }
}

void Arm::testClaw() {
    for (int i = 3; i < 10; i++) {
        this->claw->write(i * 15);
        delay(1000);
    }
}

void Arm::testArm() {
    testShoulder();
    testElbow();
    this->elbow->slowWrite(90, 8);
    testBase();
    testClaw();
    rotateBase(180);
    sweep(5, 15, 26);
    this->claw->write(45);
    delay(500);
    this->claw->write(100);
    delay(500);
    this->claw->write(45);
    delay(500);
    this->claw->write(100);
    delay(500);
}