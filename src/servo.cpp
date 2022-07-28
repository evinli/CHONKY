/**
 * @file      servo.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the motor class 
 */

#include "servo.h"

/////////////////// CONSTRUCTORS ///////////////////
Servo::Servo(PinName pwmPin) {
    this->pwmPin = pwmPin;
    pinMode(pwmPin, OUTPUT);
    pwm_start(pwmPin, SERVO_FREQ, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

void Servo::write(int angle) {
     int pwmValue = static_cast<int>(angle*(SERVO_ONE_EIGHTY_VALUE - SERVO_ZERO_VALUE)/(SERVO_RANGE) + SERVO_ZERO_VALUE);
     pwm_start(pwmPin, SERVO_FREQ, pwmValue, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
     currentAngle = angle;
}

void Servo::slowWrite(int angle, int delayMilli) {
    if (angle > currentAngle) {
        for (int i = currentAngle; i < angle; i++) {
            this->write(i);
            delay(delayMilli);
        }
    }
    else {
        for (int i = currentAngle; i > angle; i--) {
            this->write(i);
            delay(delayMilli);
        }
    }
}
