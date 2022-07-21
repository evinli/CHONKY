/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the slave Bluepill
 */

#pragma once

#include "Arduino.h"
#include "motor.h"
#include "PID.h"
#include "OLED.h"

typedef enum {
    Inactive,
    TapeFollowing,
    Archway,
    IRFollowing,
    EdgeFollowing,
} SlaveState;

class Slave {
  public:
    Slave(Motor* leftMotor, Motor* rightMotor, PID* tapeFollow, OLED* display);

    void determineState();

    // Only used for debugging
    void setState(SlaveState state);

    void moveForTime(int leftMotorSpeed, int rightMotorSpeed, int duration);

  private:
    SlaveState state;
    Motor* leftMotor;
    Motor* rightMotor;
    PID* tapeFollow;
    OLED* display;

    bool advanceState();   
};