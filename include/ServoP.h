/**
 * @file      servo.h
 * @author    Creators of CHONKY 
 * @brief     Custom servo class 
 */

#ifndef __SERVOP_H__
#define __SERVOP_H__

#include "pins.h"
#include "Arduino.h"
#include "constants.h"


class ServoP{
    public:
    ServoP(PinName pwmPin);
    
    void write(double angle);

    double angle;

    void resetServo();

    private:
    PinName pwmPin;
};

#endif // __SERVOP_H__