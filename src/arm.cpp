/**
 * @file      arm.cpp
 * @author    Creators of CHONKY
 * @brief     Class implementation for the arm class
 */

#include "arm.h"

/////////////////// CONSTRUCTORS ///////////////////
Arm::Arm(Motor* shoulder, Servo* elbow, Servo* claw, Servo* base, int shoulderSpeed, NewPing* verticalSonar) {
    this->shoulder = shoulder;
    this->elbow = elbow;
    this->claw = claw;
    this->base = base;
    this->shoulderSpeed = shoulderSpeed;
    this->verticalSonar = verticalSonar;
    pinMode(SHOULDER_POT, INPUT);
    pinMode(BASE_POT, INPUT);
    pinMode(HALL_SENSOR, INPUT_PULLUP);
}

/////////////////// METHODS ///////////////////
void Arm::moveInPlaneShoulderFirst(double distanceFromChassis, double heightAboveGround) {
    double hypotenuse = getHypotenuse(heightAboveGround, distanceFromChassis);;
    double phi = getPhi(hypotenuse);
    double theta = getTheta(hypotenuse, phi);
    double alpha = getAlpha(heightAboveGround, distanceFromChassis);
    double shoulderJointAngle = alpha + theta;
    moveShoulderJoint(shoulderJointAngle);
    delay(300);
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
    delay(300);
    moveShoulderJoint(shoulderJointAngle);
}

void Arm::grasp() {
    claw->write(CLAW_GRASP_ANGLE);
}

void Arm::moveShoulderJoint(int angle) {
    if (angle < 0) {
        return;
    }

    int potValue = analogRead(SHOULDER_POT);
    int inputValue = angle * (SHOULDER_NINETY - SHOULDER_ZERO) / (SHOULDER_RANGE) + SHOULDER_ZERO;  // input angle in terms of the potentiometer values

    while (abs(potValue - inputValue) > POT_MOTOR_ERROR) {
        if ((potValue) < inputValue) {
            shoulder->setSpeed(-(shoulderSpeed + 20)); // harder to raise shoulder than lower 
            while (potValue < inputValue) {
                potValue = analogRead(SHOULDER_POT);  
            }
        } else if ((potValue) > inputValue) {
            shoulder->setSpeed(shoulderSpeed);
            while (potValue> inputValue) {
                potValue = analogRead(SHOULDER_POT);
            }
        }
    }

    shoulder->stop();
}

void Arm::rotateBase(int angle) {
    int potValue = analogRead(BASE_POT);
    double slope = ((double)(BASE_ONE_EIGHTY - BASE_NINETY))/(double)(180 - 90);
    int targetValue =((double)(angle * slope)) - ((slope * 90) - BASE_NINETY);  

    while (abs(potValue - targetValue) > POT_MOTOR_ERROR) {
        if (potValue < targetValue) {
            base->write(BASE_CW_SPEED);  
            while (potValue < targetValue) {
                potValue = analogRead(BASE_POT);
            }
        } else if (potValue > targetValue) {
            base->write(BASE_CCW_SPEED); 
            while (potValue > targetValue) {
                potValue = analogRead(BASE_POT);
            }
        }
    }

    pwm_start(BASE_PLATE_SERVO, SERVO_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
}

double Arm::idolDetect(int numReadings) {
    int sum = 0;
    int reading;

    for (int i = 0; i < numReadings; i++) {
        reading = verticalSonar->ping_cm();
        sum += reading;
    }
    
    return (double)(sum)/numReadings;
}

void Arm::dropInBasket(int dropOffSide) {
    moveShoulderJoint(110);
    delay(500);
    moveShoulderJoint(110);
    delay(500);
    moveShoulderJoint(110);
    delay(250);
    moveShoulderJoint(110);
    this->elbow->slowWrite(30, 8); 
    rotateBase(dropOffSide);
    this->elbow->slowWrite(92, 8);
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
        if (idolDetect(IDOL_DETECT_SAMPLES) < 10) {
            graspSequence(i, height);
            dropInBasket(dropOffSide);
            break;
        }
    }
}

boolean Arm::magnetSweepAndDetect(double startingDist, double endingDist, double height, int dropOffSide) {
    for (double i = startingDist; i < endingDist; i += SWEEP_STEP_SIZE) {
        moveInPlaneShoulderFirst(i, height);
        if (magneticBomb()) {
            return false;
        }
    }
    return true;
}

void Arm::graspSequence(double startingDist, double currentHeight) {
    // map distance away to additional distance required to travel (i.e. closer needs less distance)
    // TO CHANGE, THIS IS CURRENLY CONSTANT:
    double finalDist = startingDist + 4; // trial and error
    sweep(startingDist, finalDist, currentHeight + 1); // +1 because our height seems to drop as a sweep
    moveInPlaneShoulderFirst(finalDist, currentHeight); // trial and error
    delay(1000); // make sure we're fully downright before we grasp
    grasp(); 
    delay(1000); // make sure we have idol in our grasp
}

void Arm::goToRestingPos() {
    // Move up
    moveShoulderJoint(110);
    this->elbow->slowWrite(70,8);
    claw->write(CLAW_OPEN_ANGLE);

    // Resting Position
    rotateBase(350);
    delay(300);
    rotateBase(350);
    elbow->slowWrite(35,8);
    this->claw->slowWrite(CLAW_GRASP_ANGLE,8);
    moveShoulderJoint(18);
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

bool Arm::magneticBomb() {
    int magnet_detected = 0;

    for (int i = 0; i < 20; i++){
        magnet_detected += digitalRead(HALL_SENSOR);
    }

    return magnet_detected < 20;
}

// USED FOR TESTING
void Arm::testShoulder() {
    this->elbow->slowWrite(0,10);
    for (int i = 0; i < 7; i++) {
        moveShoulderJoint(i * 15);
        delay(2000);
    }
}

void Arm::testElbow() {
    for (int i = 0; i < 10; i++) {
        this->elbow->slowWrite(i * 15,10);
        delay(2000);
    }
}

void Arm::testBase() {
    for (int i = 3; i < 23; i++) {
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
    sweep(5, 15, 32);
    this->claw->write(45);
    delay(500);
    this->claw->write(130);
    delay(500);
    this->claw->write(45);
    delay(500);
    this->claw->write(130);
    delay(500);
}
