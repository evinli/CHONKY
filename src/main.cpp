/**
 * @file      main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main control loop for CHONKY codebase
 */

#include "Arduino.h"
#include <string.h>
#include <NewPing.h>
#include <Servo.h>
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"

// Class instantiations
Motor rightMotor(PB_0, PB_1);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
//Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
PID irFollow(IRFollower, &leftMotor, &rightMotor, &display);
Servo baseServo;
Servo leadScrew;
Servo elbowServo;
Servo clawServo;
NewPing sonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
NewPing sonar1(LEFT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);

// Variable declarations
float distance;
int leftError;
int rightError;
int centreError;
int i = 0;
int loopCount=0;

void setup() {
    display.setUp();
    irFollow.setMotorSpeed(50);
    irFollow.setKD(1);
}

void loop() {

            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            int leftReading = analogRead(IR_LEFT_DETECT);
            
            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            int centreReading = analogRead(IR_CENTRE_DETECT);

            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            int rightReading = analogRead(IR_RIGHT_DETECT);

            display.write(0,std::to_string(rightReading));
            display.write(10,std::to_string(centreReading));
            display.write(20,std::to_string(leftReading));
            display.clear();

    //irFollow.usePID();
}
    
    

