/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the slave Bluepill
 */

#pragma once

#include "Arduino.h"

typedef enum {
    Inactive,
    TapeFollowToFirstIdol,
    RefindTapePostFirstIdol,
    ChickenWire,
    RefindTapePostChickenWire,
    TapeFollowToSecondIdol,
    RefindTapePostSecondIdol,
    BackUp,
    RefindTapeAgain,
    Archway,
    IRStraightFollow,
    DriveToThirdIdol,
    WaitForThirdIdolPickup,
    RefindIRPostThirdIdol,
    IRFollowToFourthIdol,
    Done,
} SlaveState;