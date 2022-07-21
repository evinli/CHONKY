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
Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
Motor shoulderMotor(SHOULDER_MOTOR_A, SHOULDER_MOTOR_B);
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
Servo baseServo;
Servo leadScrew;
Servo elbowServo;
Servo clawServo;
NewPing sonar(LEFT_TREASURE_TRIG, LEFT_TREASURE_ECHO, 200); // NewPing setup of pins and maximum distance.
NewPing sonar1(LEFT_TREASURE_TRIG, RIGHT_TREASURE_ECHO, 200);

// Variable declarations
float distance;

void setup() {
    pinMode(SLAVE_ADVANCE_STATE, INPUT_PULLDOWN);
    pinMode(SLAVE_STOP_DRIVE, INPUT_PULLDOWN);
    display.setUp();
    // baseServo.attach(BASE_PLATE_SERVO);
    // leadScrew.attach(LEAD_SCREW);
    // elbowServo.attach(ELBOW_SERVO);
    // pinMode(LEFT_TREASURE_TRIG, OUTPUT);
    // pinMode(LEFT_TREASURE_ECHO, INPUT);

}

void loop() {
    // leftMotor.setSpeed(180);
    // rightMotor.setSpeed(180);
    tapeFollow.setMotorSpeed(70);
    tapeFollow.setKP(8);
    tapeFollow.setKD(3);
    tapeFollow.setKI(0);
    tapeFollow.usePID();
    // leadScrew.write(0); // cw
    // delay(2000);
    // leadScrew.write(86); // still
    // delay(2000);
    // leadScrew.write(180); // ccw
    // delay(2000);
    // leadScrew.write(86); // still
    // delay(2000);

    // elbowServo.write(0);
    // delay(1000);
    // elbowServo.write(70);
    // delay(500);
    // elbowServo.write(140);
    // delay(3000);

    // float sum = 0;
    // for (int i = 0; i < 5; i++) {
    //     sum += sonar.ping_cm();
    // }
    // distance = sum / 5;
    // if (distance < IDOL_DISTANCE) {
    //     leftMotor.stop();
    //     rightMotor.stop();
    //     display.clear();
    //     display.write(0, "Idol detected");
    //     delay(10000);
    // } else {
    //     tapeFollow.usePID();
    // }

//     // Clears the trigPin condition
//   digitalWrite(LEFT_TREASURE_TRIG, LOW);
//   delayMicroseconds(2);
//   // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
//   digitalWrite(LEFT_TREASURE_TRIG, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(LEFT_TREASURE_TRIG, LOW);
//   // Reads the echoPin, returns the sound wave travel time in microseconds
//   duration = pulseIn(LEFT_TREASURE_ECHO, HIGH);
//   // Calculating the distance
//   distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
//   // Displays the distance on the Serial Monitor
}