/**
 * @file      PID.h
 * @author    Creators of CHONKY 
 * @brief     Header file for the PID feedback-control class
 */

#pragma once

#include "pins.h"
#include "motor.h"
#include "utils.h"
#include "OLED.h"
#include <string.h>

typedef enum {
    TapeFollower,
    EdgeFollower,
} PIDType; // create new alias PIDType as an enum data type

class PID {
  public:
    PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, OLED* display);

    void setMotorSpeed(int motorSpeed);

    void setKP(float KP);

    void setKD(float KD);
    
    void setKI(float KI);

    void usePID();

  private:
    float KP, KD, KI;
    int motorSpeed;
    int P, I, D;
    int previousError;
    int leftSensor, rightSensor;
    Motor* leftMotor;
    Motor* rightMotor;
    OLED* display;
};