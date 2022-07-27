/**
 * @file      arm.h
 * @author    Creators of CHONKY 
 * @brief     Header file for arm actuation and control
 */

#pragma once

#include "Arduino.h"
#include "motor.h"
#include "ServoP.h"

class Arm {
    public:
      /**
       * @brief Construct a new Arm object
       * 
       * @param shoulder motor for the shoulder joint
       * @param elbow object for elbow servo (non continuous), assumes 90 is and 0 is 
       * @param claw object for claw servo
       * @param base object for continuous base servo
       * @param shoulderSpeed integer speed, from 0 to 255 for speed of the shoulder motor
       */
      Arm(Motor* shoulder, ServoP* elbow, ServoP* claw, ServoP* base, int shoulderSpeed);

      /**
       * @brief move the arm to a specified distance away from the chassis at a specific height while keeping base rotation angle constant
       * 
       * @param distanceFromChassis 
       * @param heightAboveGround 
       */
      void moveInPlane(double distanceFromChassis, double heightAboveGround);

      bool grabTreasure();

      /**
       * @brief move the shoulder join to a given angle
       * 
       * @param angle 
       */
      void moveShoulderJoint(int angle);

      /**
       * @brief 
       * 
       * @param angle rotate the base of the arm to a given angel
       */
      void rotateBase(int angle);
      
      double getL3(double heightAboveGround, double distanceFromChassis);

      double getPhi(double l3);

      double getTheta(double l3, double phi);

      double getAlpha(double heightAboveGround, double distanceFromChassis);

      ServoP* elbow;
      ServoP* claw;
      Motor* shoulder;
      ServoP* base;
      int shoulderSpeed;
      
    private:
      

};