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
SlaveState state;
bool slaveEnabled;
long lastAdvanceTime;
long irFollowTime;

// Function prototypes
bool advanceState();

void setup() {
    display.setUp();
    state = SlaveState::Inactive;
    slaveEnabled = true;
    lastAdvanceTime = millis();
    pinMode(SLAVE_ADVANCE_STATE, INPUT_PULLDOWN);
    pinMode(SLAVE_STOP_DRIVE, INPUT_PULLDOWN);
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
                display.write(0, "State 0");
                break;
            }

            case(SlaveState::TapeFollowToFirstIdol): {
                display.clear();
                display.write(0, "State 1");
                tapeFollow.setMotorSpeed(110);
                tapeFollow.setKP(12);
                tapeFollow.setKD(6);//loser
                tapeFollow.setKI(0);
                tapeFollow.usePID();
                break;
            }

            case(SlaveState::RefindTapePostFirstIdol): {
                display.clear();
                display.write(0, "State 2");
                if (tapeFollow.refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow.resetPID();
                    advanceState();
                }
                break;
            }
            
            case(SlaveState::ChickenWire): {
                display.clear();
                display.write(0, "State 3");
                tapeFollow.setMotorSpeed(100);
                tapeFollow.setKP(14);
                tapeFollow.setKD(8);
                tapeFollow.setKI(0);
                if (tapeFollow.usePID() == ALL_HIGH) {
                    leftMotor.setSpeed(120);
                    rightMotor.setSpeed(130);
                    delay(1000);
                    tapeFollow.resetPID(); 
                    advanceState();
                }
                break;
            }

            case(SlaveState::RefindTapePostChickenWire): {
                display.clear();
                display.write(0, "State 4");
                if (tapeFollow.refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow.resetPID();
                    advanceState();
                } 
                else {
                    if (tapeFollow.refindTape(RIGHT_SIDE, MAX_SWEEP_TIME * 2)) {
                        tapeFollow.resetPID();
                        advanceState();
                    }
                }
                break;
            }

            case(SlaveState::TapeFollowToSecondIdol): {
                display.clear();
                display.write(0, "State 5");
                tapeFollow.setMotorSpeed(100);
                tapeFollow.setKP(12);
                tapeFollow.setKD(6);
                tapeFollow.setKI(0);
                tapeFollow.usePID();
                break;
            }

            case(SlaveState::RefindTapePostSecondIdol): {
                display.clear();
                display.write(0, "State 6");
                leftMotor.setSpeed(-100);
                rightMotor.setSpeed(-110);
                delay(500);
                if (tapeFollow.refindTape(LEFT_SIDE, MAX_SWEEP_TIME)) {
                    tapeFollow.resetPID();
                    advanceState();
                }
                break;
            }
            
            case(SlaveState::Archway): {
                display.clear();
                display.write(0, "State 7");
                tapeFollow.setMotorSpeed(100);
                tapeFollow.setKP(15);
                tapeFollow.setKD(9);
                tapeFollow.setKI(0);
                if (tapeFollow.usePID() == ALL_HIGH) {
                    tapeFollow.resetPID();
                    delay(2000);
                    advanceState();
                    irFollowTime = millis();
                }
                break;
            }

            case(SlaveState::IRStraightFollow): {
                display.clear();
                display.write(0, "State 8");
                if (millis() - irFollowTime < IR_FOLLOW_ARCHWAY) { // trial and error IR_FOLLOW_ARCHWAY time until we get far enough through the archway
                    irFollow.setMotorSpeed(120);
                    irFollow.setKP(12);
                    irFollow.setKD(6);
                    irFollow.setKI(0);
                    irFollow.usePID();
                } else {
                    advanceState();
                }
                break;
            }

            case(SlaveState::DriveToThirdIdol): {
                display.clear();
                display.write(0, "State 9");
                leftMotor.stop();
                rightMotor.setSpeed(100);
                delay(1000); // trial and error this until we get this to 90deg
                leftMotor.stop();
                rightMotor.hardStop(FORWARDS_DIR);
                advanceState();
                break;
            }

            case(SlaveState::WaitForThirdIdolPickup): {
                display.clear();
                display.write(0, "State 10");
                break;           
            }
            
            case(SlaveState::RefindIRPostThirdIdol): {
                display.clear();
                display.write(0, "State 11");
                leftMotor.stop();
                rightMotor.setSpeed(-100);
                delay(1000); // trial and error this until we get this to 90deg
                leftMotor.stop();
                rightMotor.hardStop(BACKWARDS_DIR);
                advanceState();
                break;
            }

            case(SlaveState::IRFollowToFourthIdol): {
                display.clear();
                display.write(0, "State 12");
                irFollow.setMotorSpeed(120);
                irFollow.setKP(12);
                irFollow.setKD(6);
                irFollow.setKI(0);
                irFollow.usePID();
                break;
            }

            case(SlaveState::Done): {
                display.clear();
                display.write(0, "State 13");
                leftMotor.stop();
                rightMotor.stop();
                break;
            }
        }
    }  
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