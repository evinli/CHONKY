/**
 * @file      slave_main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main file for the slave bluepill, copy this into
 *            main.cpp to use
 */

#include "Arduino.h"
#include <string.h>
#include <NewPing.h>
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"

// Class instantiations
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
PID irFollow(IRFollower, &leftMotor, &rightMotor, &display);

int state;

void setup() {
    display.setUp();
    state = 1;
}

void loop() {
    switch(state) {
        case(0): {
            tapeFollow.setMotorSpeed(85);
            tapeFollow.setKP(11);
            tapeFollow.setKD(5);
            tapeFollow.setKI(0);
            tapeFollow.usePID();

            // get IR sensor readings
            // digitalWrite(IR_MOSFET, HIGH);
            // delayMicroseconds(300);
            // digitalWrite(IR_MOSFET, LOW);
            // delayMicroseconds(25);
            // int leftReading = analogRead(IR_LEFT_DETECT);
            
            // digitalWrite(IR_MOSFET, HIGH);
            // delayMicroseconds(300);
            // digitalWrite(IR_MOSFET, LOW);
            // delayMicroseconds(25);
            // int centreReading = analogRead(IR_CENTRE_DETECT);

            // digitalWrite(IR_MOSFET, HIGH);
            // delayMicroseconds(300);
            // digitalWrite(IR_MOSFET, LOW);
            // delayMicroseconds(25);
            // int rightReading = analogRead(IR_RIGHT_DETECT);

            // display.write(40, std::to_string(leftReading));
            // display.write(50, std::to_string(centreReading));
            // display.write(60, std::to_string(rightReading));

            // if (centreReading > IR_THRESHOLD) {
            //     state = 1;
            //     display.clear();
            //     display.write(0, "at archway");
            // }
        
            // if (tapeFollow.usePID() == T_STOP) {
            //     if (tapeFollow.usePID() == T_STOP) {
            //         leftMotor.stop();
            //         rightMotor.stop();
            //         state = 1;
            //     }
            // }
            break;
        }
        case(1): {
            irFollow.setMotorSpeed(100);
            irFollow.setKP(11);
            irFollow.setKD(0);
            irFollow.setKI(0);
            irFollow.usePID();
            break;
        }
    }
}