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

// PWM FREQ
#define MOTOR_FREQ 100
#define SERVO_FREQ 50
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

// SERVO TUNING
#define SERVO_ZERO_VALUE 544
#define SERVO_ONE_EIGHTY_VALUE 2400
#define SERVO_RANGE 180
#define BASE_SERVO_STOP_ANGLE 84

//ARM
const double FOREARM_LENGTH=17;
const double SHOULDER_HEIGHT=25;
const double BICEP_LENGTH=17;
const double SHOULDER_CHASSIS_EDGE_DIST=11;
const double SWEEP_STEP_SIZE=0.3;
#define SHOULDER_MAX_ANGLE 90
#define SHOULDER_MIN_ANLGE 0

// POT TUNING
#define SHOULDER_NINETY 895
#define SHOULDER_ZERO 516
#define SHOULDER_RANGE SHOULDER_MAX_ANGLE-SHOULDER_MIN_ANLGE
#define BASE_NINETY 255
#define BASE_ONE_EIGHTY 528
 // put base pot values here too

// MOTOR 
#define POT_MOTOR_ERROR 5
#define SHOULDER_SPEED_OFFSET 20 
 

//TREASURE DETECTION
#define SWEEP_SAMPLE_COUNT 40

//CLAW
#define CLAW_GRASP_ANGLE 130
#define CLAW_OPEN_ANGLE 45

