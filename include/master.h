/**
 * @file      master.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the master Bluepill
 */

#pragma once

#include <Arduino.h>

typedef enum {
    Inactive,
    TapeFollowing,
    ChickenWire,
    Archway,
    IRFollowing,
    EdgeFollowing,
    TreasurePickup,
    ZiplineAlign,
    ZiplineDeploy,
    ZiplineDismount,
} MasterState;

class Master {
  public: 
    Master();

    MasterState determineState();

    // Used only for debugging
    void setState(MasterState state);
    
  private:
    MasterState state;

    bool advanceState();

    void signalSlaveAdvance();

    void endSlaveSignal();

    void enableSlave();

    void disableSlave();
};