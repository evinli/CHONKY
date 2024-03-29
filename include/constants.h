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
#define MAX_SPEED 255
#define FORWARDS_DIR 0
#define BACKWARDS_DIR 1

// OLED DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // since display does not have a reset pin accessible

// TAPE FOLLOWING
#define TAPE_WHITE_THRESHOLD 400
#define TAPE_ON 0
#define TAPE_ONE_OFF 1
#define TAPE_TWO_OFF 3
#define TAPE_THREE_OFF 6
#define ALL_HIGH 50

// SWEEPING
#define LEFT_SIDE 0
#define RIGHT_SIDE 1
#define MAX_SWEEP_TIME 2000
#define SWEEP_SPEED 95

// IR FOLLOWING
#define IR_THRESHOLD 100
#define IR_NUM_READINGS 1
#define ON_IR 0
#define IR_ONE_OFF 3
#define IR_FOLLOW_ARCHWAY 3000
#define IR_FOLLOW_IDOL 4000

// SERVO TUNING
#define SERVO_ZERO_VALUE 544
#define SERVO_ONE_EIGHTY_VALUE 2400
#define SERVO_RANGE 180
#define BASE_SERVO_STOP_ANGLE 80
#define SMOOTH_SLOW_WRITE_TIME 8 //value found experimentally to give smooth and reasonably slow servo actuation

// ARM
#define FOREARM_LENGTH 17
#define SHOULDER_HEIGHT 24
#define BICEP_LENGTH 22
#define SHOULDER_CHASSIS_EDGE_DIST 11
#define SWEEP_STEP_SIZE 0.3
#define SHOULDER_MAX_ANGLE 90
#define SHOULDER_MIN_ANGLE 0
#define BASE_IDOL_DETECT_ANGLE 270
#define DETECT_ANGLE_GRASP_OFFSET 10
#define IDOL_DETECTION_HEIGHT 32
#define IDOL_DETECTION_DIST 10
#define DETECTION_BASE_ANGLE 270
#define IDOL_GRASP_HEIGHT 28
#define BOMB_AVOID_DIST 13
#define BOMB_AVOID_HEIGHT 37
#define SHOULDER_DROPOFF_ANGLE 110
#define ELBOW_ROTATE_ANGLE 30
#define ELBOW_DROPOFF_ANGLE 90

// POT TUNING
#define SHOULDER_NINETY 678
#define SHOULDER_ZERO 415
#define SHOULDER_RANGE SHOULDER_MAX_ANGLE-SHOULDER_MIN_ANGLE
#define BASE_NINETY 155
#define BASE_ONE_EIGHTY 424

// MOTOR 
#define POT_MOTOR_ERROR 3
#define SHOULDER_SPEED_OFFSET 20 

// STATE MACHINE
#define MIN_ADVANCE_TIME 500
 
// TREASURE DETECTION
#define CLAW_SCAN_SAMPLES 40
#define IDOL_DETECT_SAMPLES 50

// CLAW
#define CLAW_GRASP_ANGLE 150
#define CLAW_OPEN_ANGLE 30
#define HALL_SAMPLE_COUNT 20

// BASE 
#define RIGHT_SIDE_ANGLE 270
#define LEFT_SIDE_ANGLE 90
#define MIDDLE_ANGLE 180
#define RIGHT_DROPOFF_ANGLE 350
#define LEFT_DROPOFF_ANGLE 10
#define BASE_CW_SPEED 90
#define BASE_CCW_SPEED 60

