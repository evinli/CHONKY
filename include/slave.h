/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the slave Bluepill
 */

#pragma once

#include <Arduino.h>
#include "PID.h"

typedef enum {
    Inactive,
    TapeFollowing,
    ChickenWire,
    Archway,
    IRFollowing,
    EdgeFollowing,
} SlaveState;

class Slave {
  public:
    Slave(PID* tapeFollow);

    SlaveState determineState();

    // Only used for debugging
    void setState(SlaveState state);

    void moveForTime(int leftMotorSpeed, int rightMotorSpeed, int duration);

  private:
    SlaveState state;
    PID* tapeFollow;
    
    bool advanceState();   
};