/**
 * @file      pins.h
 * @author    Creators of CHONKY 
 * @brief     Pin definitions
 */

#pragma once

/////////////////// SHARED ///////////////////
// MISC
#define LED_BUILTIN PB2


/////////////////// MASTER BLUEPILL ///////////////////
// // SERVOS 
// #define BASE_PLATE_SERVO PA3

// OLED SCREEN
#define OLED_CLOCK PB6
#define OLED_DATA PB7

/////////////////// SLAVE BLUEPILL ///////////////////
// MOTORS
// pwm_start function requires pins to be of PinName type
#define LEFT_MOTOR_A PB_1
#define LEFT_MOTOR_B PB_0
#define RIGHT_MOTOR_A PB_8 
#define RIGHT_MOTOR_B PB_9

// REFLECTANCE SENSORS
#define LEFT_TAPE_SENSOR PA3
#define CENTER_TAPE_SENSOR PA2
#define RIGHT_TAPE_SENSOR PA1
#define LEFT_EDGE_SENSOR PB0
#define RIGHT_EDGE_SENSOR PB1

// IR SENSORS
#define IR_LEFT_DETECT PA7
#define IR_RIGHT_DETECT PA6

// // OLED SCREEN
// #define OLED_CLOCK PB10
// #define OLED_DATA PB11