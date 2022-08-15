/**
 * @file      pins.h
 * @author    Creators of CHONKY 
 * @brief     Pin definitions
 */

#pragma once

/////////////////// SHARED ///////////////////
// OLED SCREEN
#define OLED_CLOCK PB6
#define OLED_DATA PB7

// SERIAL MONITOR
#define SERIAL_TX PA9
#define SERIAL_RX PA10

// MISC
#define LED_BUILTIN PC13

/////////////////// MASTER BLUEPILL ///////////////////
// ULTRSONIC
#define TREASURE_TRIG PB15
#define TREASURE_ECHO PB14
#define CLAW_SCAN_TRIG PB10
#define CLAW_SCAN_ECHO PB11

// MOTORS
#define SHOULDER_MOTOR_A PB_8
#define SHOULDER_MOTOR_B PB_9

// SERVOS 
#define BASE_PLATE_SERVO PA_1
#define ELBOW_SERVO PA_2
#define CLAW_SERVO PA_0

// POTENTIOMETERS
#define SHOULDER_POT PA5
#define BASE_POT PA4

// MISC 
#define HALL_SENSOR PB3

// COMMUNICATIONS
#define MASTER_STOP_DRIVE PB12
#define MASTER_ADVANCE_STATE PB13

/////////////////// SLAVE BLUEPILL ///////////////////
// MOTORS
#define LEFT_MOTOR_A PA_9
#define LEFT_MOTOR_B PA_10
#define RIGHT_MOTOR_A PB_8 
#define RIGHT_MOTOR_B PB_9

// REFLECTANCE SENSORS
#define LEFT_TAPE_SENSOR PA2
#define CENTER_TAPE_SENSOR PA1
#define RIGHT_TAPE_SENSOR PA0 
#define LEFT_EDGE_SENSOR PA6 
#define RIGHT_EDGE_SENSOR PA5 

// IR SENSORS
#define IR_LEFT_DETECT PB1
#define IR_CENTRE_DETECT PB0
#define IR_RIGHT_DETECT PA7
#define IR_MOSFET PB10

// POTENTIOMETERS
#define KP_POT PA4
#define KD_POT PA3

// COMMUNICATIONS
#define SLAVE_STOP_DRIVE PB12
#define SLAVE_ADVANCE_STATE PB13
