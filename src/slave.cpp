/**
 * @file      slave.cpp
 * @author    Creators of CHONKY 
 * @brief     State machine implementation for the slave Bluepill
 */

#include "slave.h"

/////////////////// CONSTRUCTORS ///////////////////
Slave::Slave(PID* tapeFollow) {
    this->tapeFollow = tapeFollow;

}

/////////////////// METHODS ///////////////////
// SlaveState Slave::determineState() {
//     // insert communication checks

//     switch(state) {
//         case SlaveState::Inactive: 
        
//         case SlaveState::TapeFollowing:
//             tapeFollow->setMotorSpeed(100);
//             tapeFollow->setKP(0.1);
//             tapeFollow->usePID();
//             break;
        
//         case SlaveState::ChickenWire:

//         case SlaveState::Archway:

//         case SlaveState::IRFollowing:

//         case SlaveState::EdgeFollowing:

//     }
// }
