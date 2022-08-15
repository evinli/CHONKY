/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the slave Bluepill
 */

#pragma once

#include "Arduino.h"
#include "motor.h"
#include "PID.h"

typedef enum {
    Inactive,
    TapeFollow,
    RefindTapePostFirstIdol,
    ChickenWire,
    RefindTapePostChickenWire,
    TapeFollowPostRamp,
    RefindTapePostSecondIdol,
    NavigateArchway,
    Archway,
    IRFollow,
    DriveToThirdIdol,
    WaitForThirdIdolPickup,
    RefindIRPostThirdIdol,
    IRFollowToFourthIdol,
    Done,
} SlaveState;

class Slave {
  public:
    Slave(Motor* leftMotor, Motor* rightMotor, PID* tapeFollow, PID* irFollow);

    void determineState();

  private:
    SlaveState currentState;
    Motor* leftMotor;
    Motor* rightMotor;
    PID* tapeFollow;
    PID* irFollow;
    long lastAdvanceTime;
    long lastEventTime;
    bool slaveEnabled;
    bool alreadyStopped;

    void moveForTime(int leftMotorSpeed, int rightMotorSpeed, int timeMillis); 

    void advanceState();

};