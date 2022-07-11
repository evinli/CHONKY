/**
 * @file      pins.h
 * @author    Creators of CHONKY 
 * @brief     Pin definitions
 */

#pragma once

#define LED_BUILTIN PB_2

// MOTORS
// pwm_start function requires pins to be of PinName type
#define RIGHT_MOTOR_A PB_8 
#define RIGHT_MOTOR_B PB_9 
#define LEFT_MOTOR_A PA_2
#define LEFT_MOTOR_B PA_3

// REFLECTANCE SENSORS
#define LEFT_TAPE_SENSOR PB0
#define RIGHT_TAPE_SENSOR PB1
#define LEFT_EDGE_SENSOR PA6
#define RIGHT_EDGE_SENSOR PA7

// OLED SCREEN
#define OLED_CLOCK PB6
#define OLED_DATA PB7

#define LED_BUILTIN PB2