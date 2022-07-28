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
    /**
     * @brief Construct a new Servo P object
     * 
     * @param pwmPin: pin that connects to the pwm pin of the servo
     */
    ServoP(PinName pwmPin);
    
    /**
     * @brief method to move the servo to a specified angle
     * 
     * @param angle 
     */
    void write(double angle);

    void slowWrite(int finalAngle, int delayLengthMilli);

    void resetServo();

    double angle;

    private:
    PinName pwmPin;
};

#endif // __SERVOP_H__