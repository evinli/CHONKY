/**
 * @file      PID.h
 * @author    Creators of CHONKY 
 * @brief     Header file for the PID feedback-control class
 */

#pragma once

#include "pins.h"
#include "motor.h"
#include "utils.h"
#include "OLED.h"
#include <string.h>

typedef enum {
    TapeFollower,
    EdgeFollower,
    IRFollower,
} PIDType; // create new alias PIDType as an enum data type

class PID {
  public:
    /**
     * @brief Construct a new PID object
     * 
     * @param pidType Type of PID
     * @param leftMotor pointer to left rear motor
     * @param rightMotor pointer to right rear motor
     * @param display pointer to OLED object
     */
    PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, OLED* display);

    /**
     * @brief Set base speed for rear motors
     * 
     * @param motorSpeed motor speed 
     */
    void setMotorSpeed(int motorSpeed);

    /**
     * @brief Set KP value for PID controller
     * 
     * @param KP proportional gain
     */
    void setKP(float KP);

    /**
     * @brief Set KD value for PID controller
     * 
     * @param KD derivative gain 
     */
    void setKD(float KD);

    /**
     * @brief Set KP value for PID controller
     * 
     * @param KI integral gain
     */
    void setKI(float KI);

    /**
     * @brief Run the PID controller
     * 
     * @return current PID error
     */
    int usePID();

  private:
    float KP, KD, KI;
    int motorSpeed;
    int P, I, D;
    int lastError;
    PIDType pidType;
    Motor* leftMotor;
    Motor* rightMotor;
    OLED* display;

    /**
     * @brief Check if reflectance sensor is on white paint
     * 
     * @param reading reflectance sensor reading
     * @param threshold cutoff threshold for white paint
     * @return true if sensor is on white paint (reading < threshold)
     * @return false if sensor is on black tape (reading > threshold)
     */
    bool sensorOnWhite(int reading, int threshold);

    /**
     * @brief Check if IR sensor is not detecting 10kHz
     * 
     * @param reading IR sensor reading
     * @param threshold cutoff threshold for off IR
     * @return true if sensor is not reading 10kHz (reading < threshold)
     * @return false if sensor is reading 10kHz (reading > threshold)
     */
    bool sensorOnIR(int reading, int threshold);

    /**
     * @brief Determine tape following PID error based on sensor states
     * 
     * @param leftOnWhite left sensor state
     * @param centreOnWhite centre sensor state
     * @param rightOnWhite right sensor state
     * @return PID error
     */
    int getTapeError(bool leftOnWhite, bool centreOnWhite, bool rightOnWhite);

    /**
     * @brief Determine IR following PID error based on sensor states
     * 
     * @param leftOffIR left sensor state
     * @param centreOffIR centre sensor state
     * @param rightOffIR right sensor state
     * @return PID error
     */
    int getIRError(bool leftOffIR, bool centreOffIR, bool rightOffIR);
};