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
      Arm(Motor* shoulder, ServoP* elbow, ServoP* claw, int shoulderSpeed);

      void positionOverTreasure(double distanceFromChassis, double heightAboveGround);

      bool grabTreasure();

      void moveShoulderJoint(int angle);
      
      double getL3(double heightAboveGround, double distanceFromChassis);

      double getPhi(double l3);

      double getTheta(double l3, double phi);

      double getAlpha(double heightAboveGround, double distanceFromChassis);

    private:
      ServoP* elbow;
      ServoP* claw;
      Motor* shoulder;
      int shoulderSpeed;

};