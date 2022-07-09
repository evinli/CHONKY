/**
 * @file      PID.h
 * @author    Creators of CHONKY 
 * @brief     Header file for the PID feedback-control class
 */

#pragma once

#include "pins.h"
#include "motor.h"
#include "adc_utils.h"

typedef enum {
    TapeFollower,
    EdgeFollower,
} PIDType; // create new alias PIDType as an enum data type

class PID {
  public:
    PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, int baseSpeed);

    void setKP(int KP);

    void setKD(int KD);
    
    void setKI(int KI);

    void usePID();

  private:
    float KP, KD, KI;
    int P, I, D;
    int previousError;
    int leftSensor, rightSensor;
    Motor* leftMotor;
    Motor* rightMotor;
    int baseSpeed;
};