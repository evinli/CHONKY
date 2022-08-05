/**
 * @file      slave_main.cpp
 * @author    Creators of CHONKY 
 * @brief     Main file for the slave bluepill, copy this into
 *            main.cpp to use
 */

#include "Arduino.h"
#include <string.h>
#include "PID.h"
#include "motor.h"
#include "pins.h"
#include "OLED.h"
#include "slave.h"

// Class instantiations
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OLED display(&display_handler);
Motor rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B);
Motor leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B);
PID tapeFollow(TapeFollower, &leftMotor, &rightMotor, &display);
PID irFollow(IRFollower, &leftMotor, &rightMotor, &display);

// Variable declarations
int state, idolCount;
bool slaveEnabled;
long lastAdvanceTime;

// Function prototypes
bool advanceState();
void stopSlaveISR();

void setup() {
    display.setUp();
    state = SlaveState::TapeFollowing;
    idolCount = 0;
    slaveEnabled = true;
    lastAdvanceTime = millis();
    pinMode(SLAVE_ADVANCE_STATE, INPUT_PULLDOWN);
    pinMode(SLAVE_STOP_DRIVE, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(SLAVE_STOP_DRIVE), stopSlaveISR, RISING);
}

void loop() {
    // Reset slave enabled flag
    slaveEnabled = true;

    // Check comms pins 
    if (digitalRead(SLAVE_ADVANCE_STATE) == HIGH) {
        advanceState();
    }
    if (digitalRead(SLAVE_STOP_DRIVE) == HIGH) {
        leftMotor.stop();
        rightMotor.stop();
        slaveEnabled = false;
    }
    
    // Run state machine
    if (slaveEnabled) {
        switch(state) {
            case(SlaveState::Inactive): {
                display.clear();
                display.write(0, "Inactive State");
                break;
            }

            case(SlaveState::TapeFollowing): {
                tapeFollow.setMotorSpeed(90);
                tapeFollow.setKP(15);
                tapeFollow.setKD(0);
                tapeFollow.setKI(0);
                int tapeState = tapeFollow.usePID(idolCount);
                if (tapeState == ALL_HIGH) {
                    advanceState();
                }
                // leftMotor.setSpeed(82);
                // rightMotor.setSpeed(98);
                break;
            }
            
            case(SlaveState::Archway): {
                leftMotor.stop();
                rightMotor.stop();
                delay(5000);
                advanceState();
                break;
            }

            case(SlaveState::IRFollowing): {
                irFollow.setMotorSpeed(100);
                irFollow.setKP(11);
                irFollow.setKD(0);
                irFollow.setKI(0);
                irFollow.usePID(idolCount);
                break;
            }
        }
    }  
}

void stopSlaveISR() {
    idolCount++;
}

bool advanceState() {
    // Ensure doesn't advance state twice
    if (millis() - lastAdvanceTime < MIN_ADVANCE_TIME) { 
        return false; 
    }
    lastAdvanceTime = millis();
    if (state == SlaveState::Done) {
         return false; 
    }

    state = static_cast<SlaveState>(static_cast<int>(state) + 1);
    return true;
}