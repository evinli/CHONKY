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
#define ALL_HIGH -1

// IR FOLLOWING
#define IR_THRESHOLD 190
#define IR_NUM_READINGS 1
#define ON_TEN_K 0
#define IR_ONE_OFF 1
#define IR_TWO_OFF 3
#define IR_THREE_OFF 6

// SERVO TUNING
#define SERVO_ZERO_VALUE 540
#define SERVO_ONE_EIGHTY_VALUE 2400
#define SERVO_RANGE 180
#define BASE_SERVO_STOP_ANGLE 83

// ARM
const double FOREARM_LENGTH = 17;
const double SHOULDER_HEIGHT = 6;
const double BICEP_LENGTH = 17;
const double SHOULDER_CHASSIS_EDGE_DIST = 11;
const double SWEEP_STEP_SIZE = 0.3;

// POT TUNING
#define SHOULDER_NINETY 895
#define SHOULDER_ZERO 530
#define SHOULDER_RANGE 90
 // put base pot values here too

// MOTOR 
#define POT_MOTOR_ERROR 5
#define SHOULDER_SPEED_OFFSET 20 

// STATE MACHINE
#define MIN_ADVANCE_TIME 500
 





