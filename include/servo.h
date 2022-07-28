/**
 * @file      servo.h
 * @author    Creators of CHONKY 
 * @brief     Custom servo class 
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "pins.h"
#include "Arduino.h"
#include "constants.h"

class Servo {
  public:
    /**
     * @brief Construct a new servo object
     * 
     * @param pwmPin pin that connects to the pwm pin of the servo
     */
    Servo(PinName pwmPin);
    
    /**
     * @brief Method to move the servo to a specified angle
     * 
     * @param angle 
     */
    void write(int angle);

    /**
     * @brief Method to gradually servo to a specified angle 
     *        (smoother than servo.write())
     * 
     * @param angle
     * @param delayMilli  
     */
    void slowWrite(int finalAngle, int delayMilli);

    private:
      PinName pwmPin;
      int currentAngle;
};

#endif // __SERVOP_H__