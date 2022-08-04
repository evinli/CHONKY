/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the slave Bluepill
 */

#pragma once

#include "Arduino.h"

typedef enum {
    Inactive,
    TapeFollowing,
    Archway,
    IRFollowing,
    EdgeFollowing,
    Done,
} SlaveState;