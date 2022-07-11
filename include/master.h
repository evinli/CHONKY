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
} masterStates;