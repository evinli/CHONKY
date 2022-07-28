/**
 * @file      servo.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the motor class 
 */

#include "ServoP.h"

/////////////////// CONSTRUCTORS ///////////////////
ServoP::ServoP(PinName pwmPin){
    this->pwmPin = pwmPin;
    pinMode(PA_1, OUTPUT);
    pwm_start(this->pwmPin,SERVO_FREQ,0,TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

void ServoP::resetServo(){
    pwm_start(this->pwmPin,SERVO_FREQ,0,TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

void ServoP::write(double angle){
     this->angle=angle;
     double value=(angle)*((double(SERVO_ONE_EIGHTY_VALUE-SERVO_ZERO_VALUE))/double(SERVO_RANGE))+SERVO_ZERO_VALUE;
     pwm_start(this->pwmPin,SERVO_FREQ,(int)(value),TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

void ServoP::slowWrite(int finalAngle, int delayLengthMilli){
    if(finalAngle>this->angle){
        for (int i=this->angle;i<finalAngle;i++){
        this->write(i);
        delay(delayLengthMilli);
        }
    }
    else{
        for (int i=this->angle;i>finalAngle;i--){
        this->write(i);
        delay(delayLengthMilli);
        }
    }
}
