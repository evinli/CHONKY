/**
 * @file      slave.h
 * @author    Creators of CHONKY 
 * @brief     State machine header file for the slave Bluepill
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
} slaveStates;

class Slave {
  public:
    

};