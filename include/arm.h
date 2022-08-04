/**
 * @file      arm.h
 * @author    Creators of CHONKY 
 * @brief     Header file for arm actuation and control
 */

#ifndef __ARM_H__
#define __ARM_H__

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
     */
    Arm(Motor* shoulder, Servo* elbow, Servo* claw, Servo* base, int shoulderSpeed, NewPing* verticalSonar, NewPing* horizontalSonar, OLED* display);

    /**
     * @brief Move the arm to a specified distance away from the chassis at a 
     *        specific height while keeping base rotation angle constant
     * 
     * @param distanceFromChassis 
     * @param heightAboveGround 
     */
    void moveInPlaneShoulderFirst(double distanceFromChassis, double heightAboveGround);
    void moveInPlaneElbowFirst(double distanceFromChassis, double heightAboveGround);

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

    void sweep(double startingDist, double endingDist, double height); 
    
    void sweepAndDetect(double startingDist, double endingDist, double height);

    void graspSequence(double startingDistFromChassis, double finalHeight);

    void testShoulder();

    void testElbow();

    void testClaw();

    void testBase();

    void testArm();
    
    double avgSampleSonar(int numReadings, NewPing* sonarSensor);

    void dropInBasket();


    Motor* shoulder;
    Servo* elbow;
    Servo* claw;
    Servo* base;
    NewPing* verticalSonar;
    NewPing* horizontalSonar;
    OLED* display;
    int shoulderSpeed;     

  private:
    
        
    double getHypotenuse(double heightAboveGround, double distanceFromChassis);

    double getPhi(double hypotenuse);

    double getTheta(double hypotenuse, double phi);

    double getAlpha(double heightAboveGround, double distanceFromChassis);

};

#endif // __ARM_H__
