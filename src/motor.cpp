/**
 * @file      motors.cpp
 * @author    Creators of CHONKY 
 * @brief     Class for driving motors 
 */

#include "motor.h"

/////////////////// CONSTRUCTORS ///////////////////
Motor::Motor(int pinA, int pinB) {
    this->pinA = (PinName)pinA;
    this->pinB = (PinName)pinB;
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pwm_start(this->pinA, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(this->pinB, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

/////////////////// METHODS ///////////////////
void Motor::setSpeed(int speed) {
    // Set foward/reverse speed limits
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // TODO: Need to add a delay if motor speed suddenly changes to smooth oscillations

    if (speed > 0) {
        pwm_start(pinA, MOTOR_FREQ, speed * MAP_8_BIT_TO_12_BIT, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pinB, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    } 
    else {
        pwm_start(pinA, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(pinB, MOTOR_FREQ, speed * MAP_8_BIT_TO_12_BIT, RESOLUTION_12B_COMPARE_FORMAT);
    }

    // Save new speed 
    previousSpeed = speed;
}

void Motor::stop() {
    pwm_start(pinA, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(pinB, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

