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
#define TAPE_WHITE_THRESHOLD 400
#define TAPE_NUM_READINGS 1
#define TAPE_ON 0
#define TAPE_ONE_OFF 1
#define TAPE_TWO_OFF 3
#define TAPE_THREE_OFF 6
#define T_STOP -1

// TREASURE DETECTION
#define IDOL_DISTANCE 30

//SERVOS
#define SERVO_FREQ 50
#define SERVO_ZERO_VALUE 540
#define SERVO_ONE_EIGHTY_VALUE 2400
#define SERVO_RANGE 180

//ARM DIMENSIONS
const double FOREARM_LENGTH=17;
const double SHOULDER_HEIGHT=25;
const double BICEP_LENGTH=17;
const double SHOULDER_CHASSIS_EDGE_DIST=7;


