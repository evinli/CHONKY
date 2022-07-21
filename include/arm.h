/**
 * @file      arm.h
 * @author    Creators of CHONKY 
 * @brief     Header file for arm actuation and control
 */

#pragma once

#include "Arduino.h"
#include <Servo.h>
#include "motor.h"

class Arm {
    public:
      Arm(Motor* shoulder, Servo* elbow, Servo* claw);

      bool moveElbow(int angle);

      bool moveShoulder(int angle);

      bool grabTreasure();

    private:
      Servo* elbow;
      Servo* claw;
      Motor* shoulder;
};