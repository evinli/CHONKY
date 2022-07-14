/**
 * @file      constants.h
 * @author    Creators of CHONKY 
 * @brief     List of all constants used for CHONKY firmware
 */

#pragma once

// PINS
#define ANALOG_READ_RESOLUTION 12
#define BLINK_COUNT 10
#define NUM_PWM_PINS 16
#define NUM_ANALOG_PINS 10

// TIMER
#define NUM_TIMERS 4
#define TIMER_PERIOD 20

// MOTORS
#define MOTOR_FREQ 100
#define MAP_8_BIT_TO_12_BIT 16

// OLED DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // since display does not have a reset pin accessible

// TAPE FOLLOWING
#define TAPE_WHITE_THRESHOLD 300
#define TAPE_NUM_READINGS 1
#define TAPE_KP 0 // need to replace with actual values
#define TAPE_KD 0 // need to replace with actual values
#define TAPE_KI 0 // need to replace with actual values
#define TAPE_ON 0
#define TAPE_ONE_OFF 1
#define TAPE_BOTH_OFF 5

