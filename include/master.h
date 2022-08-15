/**
 * @file      master.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the master Bluepill
 */

#pragma once

#include "Arduino.h"
#include "NewPing.h"
#include "arm.h"

typedef enum {
    Inactive,
    FirstIdol,
    PostFirstIdol,
    SecondIdol,
    ThirdIdol,
    FourthIdol,
    Done,
} MasterState;

class Master {
  public: 
    Master(Arm* arm);

    void determineState();
    
  private:
    MasterState currentState;
    Arm* arm;
    long lastEventTime;

    void advanceState();

    void signalSlaveAdvance();

    void endSlaveAdvanceSignal();

    void enableSlave();

    void disableSlave();
};