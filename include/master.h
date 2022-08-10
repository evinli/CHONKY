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

// class Master {
//   public: 
//     Master(NewPing* leftUltrasonic, NewPing* rightUltrasonic, Arm* arm);

//     void determineState();

//     // Used only for debugging
//     void setState(MasterState state);
    
//   private:
//     MasterState state;
//     NewPing* leftUltrasonic;
//     NewPing* rightUltrasonic;
//     Arm* arm;

//     bool advanceState();

//     void signalSlaveAdvance();

//     void endSlaveSignal();

//     void enableSlave();

//     void disableSlave();
// };