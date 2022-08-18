/**
 * @file      arm.h
 * @author    Creators of CHONKY
 * @brief     Header file for arm actuation and control
 */

#pragma once

#include <NewPing.h>

#include "Arduino.h"
#include "OLED.h"
#include "motor.h"
#include "servo.h"

class Arm {
   public:
    Servo* claw;

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
     *        specific height while keeping base rotation angle constant. Moves the shoulder
     *        joint before the elbow.
     *
     * @param distanceFromChassis
     * @param heightAboveGround
     */
    void moveInPlaneShoulderFirst(double distanceFromChassis, double heightAboveGround);

    /**
     * @brief Move the arm to a specified distance away from the chassis at a
     *        specific height while keeping base rotation angle constant. Moves the elbow
     *        joint before the shoulder.
     *
     * @param distanceFromChassis
     * @param heightAboveGround
     */
    void moveInPlaneElbowFirst(double distanceFromChassis, double heightAboveGround);

    /**
     * @brief Close claw and grasp object
     */
    void grasp();

    /**
     * @brief Move the shoulder join to a given angle
     *
     * @param angle angle to move shoulder joint to. When shoulder linkage is
     *              perpendicular to the base, the angle is 90. When shoulder is 
     *              parallel, the angle is 0. Range of shoulderis 0 to 110 degrees, 
     *              any angles outside this range will not allow the shoulder to move.
     */
    void moveShoulderJoint(int angle);

    /**
     * @brief Rotate the base of the arm to a given angle
     *
     * @param angle 0 is when shoulder worm gear shaft faces the back of the chassis
     *              and angle increases as you rotate clockwise (when observing from above).
     *              Range of arm is 15 to 350 degrees due to potentiometer limitations.
     */
    void rotateBase(int angle);

    /**
     * @brief Get average sonar reading over given number of samples
     *
     * @param numReadings number of samples
     * @return double average reading, distances above ___ are assigned distance of 400 (arbitrary large number)
     */
    double idolDetect(int numReadings);

    /**
     * @brief Move arm to drop off a grasped object
     *
     * @param dropOffSide side to rotate to
     */
    void dropInBasket(int dropOffSide);

    /**
     * @brief Move arm into resting position to pass through the archway and minimize overall footprint
     */
    void goToRestingPos();

    /**
     * @brief Sweep the base over an angular range with the arm at a set height
     *        to detect treasures with the downward facing sonar sensor.
     *        increments the distance away from the chassis with each sweep. 
     *
     * @param startingAngle must be greater than ending angle. This marks the beginning of the sweep, must be less than 350 degrees.
     * @param endingAngle must be less than starting angle, this marks the limit of the sweep, must be greater than 15 degrees.
     * @param startingDist initial distance away from chassis for the initial sweep.
     * @param finalDist maximum distance away from chassis for sweeps. Difference in finalDist and startingDist
     *                  must be a multiple of distIncrementSize to achieve full range of distances.
     * @param sweepHeight height at which all sweeps are executed.
     * @param dropOffAngle base angle to drop off the treasure if found and grasped.
     * @param distIncrementSize size of increments for the distance away from chassis. 
     */
    void rotationalSweep(int startingAngle, int endingAngle, int startingDist, int finalDist, int sweepHeight, int dropOffAngle, int distIncrementSize);

    /**
     * @brief Scan for magnetic fields
     *
     * @return true if magnetic field present, false otherwise
     */

    bool magneticBomb();

    /**
     * @brief grasps an object at given distance and height, and drops it off at a give base angle. 
     *
     * @param graspDist distance away from the chassis to grasp at.
     * @param graspHeight heigh above the ground to grasp at.
     * @param dropOffAngle base angle to drop off the grasped object, must be less than 350 degrees but greater than 15 degrees.
     */
    void graspSequence(double graspDist, double graspHeight, int dropOffAngle);

    // UNUSED METHODS, FOR TESTING PURPOSES ONLY
    /**
     * @brief move the arm across a distance while maintaining a height
     *
     * @param startingDist
     * @param endingDist
     * @param height
     */
    void sweep(double startingDist, double endingDist, double height);

    /**
     * @brief same as sweep but if an object is detected during the sweep by the downward sonar, pick it up and drop it in the basket
     *
     * @param startingDist
     * @param endingDist
     * @param height
     * @param dropOffSide
     */
    void sweepAndDetect(double startingDist, double endingDist, double height, int dropOffSide);

    /**
     * @brief move the shoulder in 15 degree increments from 0 to 90 degrees to confirm functionality.
     * 
     */
    void testShoulder();
    
    /**
     * @brief move the elbow in 15 degree increments from ___ to __ to confirm functionality
     * 
     */
    void testElbow();

    /**
     * @brief move claw rippers in 15 degree increments from ___ to ___ to confirm functionality
     * 
     */
    void testClaw();

    /**
     * @brief move the base in 15 degree increments from ___ to ___ to confirm functionality
     * 
     */
    void testBase();

    /**
     * @brief actuate various joints of arm to confirm overall functionality. 
     * 
     */
    void testArm();

   private:
    int shoulderSpeed;
    Motor* shoulder;
    Servo* base;
    Servo* elbow;
    NewPing* verticalSonar;

    double getHypotenuse(double heightAboveGround, double distanceFromChassis);

    double getPhi(double hypotenuse);

    double getTheta(double hypotenuse, double phi);

    double getAlpha(double heightAboveGround, double distanceFromChassis);
};
