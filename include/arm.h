/**
 * @file      arm.h
 * @author    Creators of CHONKY 
 * @brief     Header file for arm actuation and control
 */

#pragma once

#include "Arduino.h"
#include "motor.h"
#include "servo.h"
#include "OLED.h"
#include <NewPing.h>

class Arm {
  public:
    /**
     * @brief Construct a new Arm object
     * 
     * @param shoulder motor for the shoulder joint
     * @param elbow object for elbow servo (non continuous), assumes 90 is and 0 is 
     * @param claw object for claw servo
     * @param base object for continuous base servo
     * @param shoulderSpeed speed of shoulder joint motor
     * @param verticalSonar object for vertical sonar
     */
    Arm(Motor* shoulder, Servo* elbow, Servo* claw, Servo* base, int shoulderSpeed, NewPing* verticalSonar);

    /**
     * @brief Move the arm to a specified distance away from the chassis at a 
     *        specific height while keeping base rotation angle constant
     * 
     * @param distanceFromChassis 
     * @param heightAboveGround 
     */
    void moveInPlaneShoulderFirst(double distanceFromChassis, double heightAboveGround);
    void moveInPlaneElbowFirst(double distanceFromChassis, double heightAboveGround);

    /**
     * @brief Close claw and grasp object
     */
    void grasp();

    /**
     * @brief Move the shoulder join to a given angle
     * 
     * @param angle 
     */
    void moveShoulderJoint(int angle);

    /**
     * @brief Rotate the base of the arm to a given angle
     * 
     * @param angle
     */
    void rotateBase(int angle);

    /**
     * @brief Get average sonar reading over given number of samples
     * 
     * @param numReadings number of samples
     * @return double average reading
     */
    double idolDetect(int numReadings);

    /**
     * @brief Move arm to hover over basket
     * 
     * @param dropOffSide side to rotate to
     */
    void dropInBasket(int dropOffSide);  

    /**
     * @brief Move arm into resting position
     */
    void goToRestingPos();

    /**
     * @brief Scan for magnetic fields
     * 
     * @return true if magnetic field present, false otherwise 
     */
    bool magneticBomb();
    
    // UNUSED METHODS, FOR TESTING PURPOSES ONLY
    void sweep(double startingDist, double endingDist, double height); 

    void sweepAndDetect(double startingDist, double endingDist, double height, int dropOffSide);

    void graspSequence(double startingDistFromChassis, double finalHeight);

    void testShoulder();

    void testElbow();

    void testClaw();

    void testBase();

    void testArm();

    void flatten();    

    bool magnetSweepAndDetect(double startingDist, double endingDist, double height, int dropOffSide);
    
    Motor* shoulder;
    Servo* claw;
    Servo* base;
    Servo* elbow;
    NewPing* verticalSonar;

  private:
    int shoulderSpeed;    
        
    double getHypotenuse(double heightAboveGround, double distanceFromChassis);

    double getPhi(double hypotenuse);

    double getTheta(double hypotenuse, double phi);

    double getAlpha(double heightAboveGround, double distanceFromChassis);
};
