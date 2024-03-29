/**
 * @file      motors.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the motor class 
 */

#include "motor.h"

/////////////////// CONSTRUCTORS ///////////////////
Motor::Motor(PinName pinA, PinName pinB) {
    this->pinA = pinA;
    this->pinB = pinB;
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pwm_start(pinA, MOTOR_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(pinB, MOTOR_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
}

/////////////////// METHODS ///////////////////
void Motor::setSpeed(int speed) {
    // Set foward/reverse speed limits
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        pwm_start(pinA, MOTOR_FREQ, abs(speed) * MAP_8_BIT_TO_12_BIT, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pinB, MOTOR_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    } 
    else {
        pwm_start(pinA, MOTOR_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pinB, MOTOR_FREQ, abs(speed) * MAP_8_BIT_TO_12_BIT, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    }
}

void Motor::stop() {
    pwm_start(pinA, MOTOR_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(pinB, MOTOR_FREQ, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
}

void Motor::hardStop(int currDir) {
    if (currDir == FORWARDS_DIR) {
        this->setSpeed(-MAX_SPEED);
    }
    else if (currDir == BACKWARDS_DIR) {
        this->setSpeed(MAX_SPEED);
    }
    delay(40);
    this->stop();
}

