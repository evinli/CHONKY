/**
 * @file      PID.cpp
 * @author    Creators of CHONKY 
 * @brief     Class implementation for the PID feedback-control
 */

#include "PID.h"

/////////////////// CONSTRUCTORS ///////////////////
PID::PID(PIDType pidType, Motor* leftMotor, Motor* rightMotor, OLED* display) {
    motorSpeed, lastError, P, I, D = 0;
    this->pidType = pidType;
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->display = display;

    switch (pidType) {
        case PIDType::TapeFollower: 
            pinMode(LEFT_TAPE_SENSOR, INPUT);
            pinMode(CENTER_TAPE_SENSOR, INPUT);
            pinMode(RIGHT_TAPE_SENSOR, INPUT);
            KP, KD, KI = 0;
            break;
        
        case PIDType::IRFollower:
            pinMode(IR_LEFT_DETECT, INPUT);
            pinMode(IR_CENTRE_DETECT, INPUT);
            pinMode(IR_RIGHT_DETECT, INPUT);
            pinMode(IR_MOSFET, OUTPUT);
            KP, KD, KI = 0;
            break;

        // case PIDType::EdgeFollower: 
        //     // leftSensor = LEFT_EDGE_SENSOR;
        //     // rightSensor = RIGHT_EDGE_SENSOR;
        //     break;
        
        default: 
            // should never get here
            break;
    }
}

/////////////////// METHODS ///////////////////
void PID::setMotorSpeed(int motorSpeed) {
    this->motorSpeed = motorSpeed;
}

void PID::setKP(float KP) {
    this->KP = KP;
}

void PID::setKD(float KD) {
     this->KD = KD;
}
    
void PID::setKI(float KI) {
     this->KI = KI;
}

int PID::usePID() {
    int error;
    int leftReading;
    int centreReading;
    int rightReading;

    switch (pidType) {         
        case PIDType::TapeFollower: {
            // get reflectance sensor readings
            leftReading = getAvgAnalogValue(LEFT_TAPE_SENSOR, TAPE_NUM_READINGS);
            centreReading = getAvgAnalogValue(CENTER_TAPE_SENSOR, TAPE_NUM_READINGS);
            rightReading = getAvgAnalogValue(RIGHT_TAPE_SENSOR, TAPE_NUM_READINGS);

            // display readings
            display->clear();
            display->write(0, "Left Reading:" + std::to_string(leftReading));
            display->write(15, "Centre Reading:" + std::to_string(centreReading));
            display->write(30, "Right Reading:" + std::to_string(rightReading));

            bool leftOnWhite = sensorOnWhite(leftReading, TAPE_WHITE_THRESHOLD);
            bool centreOnWhite = sensorOnWhite(centreReading, TAPE_WHITE_THRESHOLD);
            bool rightOnWhite = sensorOnWhite(rightReading, TAPE_WHITE_THRESHOLD);
            error = getTapeError(leftOnWhite, centreOnWhite, rightOnWhite);
            display->write(45, "Error:" + std::to_string(error));
            if (error == T_STOP) {
                return error;
            }
            break;
        }
        
        case PIDType::IRFollower:{
            // get IR sensor readings
            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            leftReading = analogRead(IR_LEFT_DETECT);
            
            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            centreReading = analogRead(IR_CENTRE_DETECT);

            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            rightReading = analogRead(IR_RIGHT_DETECT);

            // display readings
            display->clear();
            display->write(0, "Left Reading:" + std::to_string(leftReading));
            display->write(10, "Centre Reading:" + std::to_string(centreReading));
            display->write(20, "Right Reading:" + std::to_string(rightReading));
            
            // determine sensor state
            bool leftOnIR = sensorOnIR(leftReading, IR_THRESHOLD);
            bool centreOnIR = sensorOnIR(centreReading, IR_THRESHOLD);
            bool rightOnIR = sensorOnIR(rightReading, IR_THRESHOLD);

            // get error
            error = getIRError(leftOnIR, centreOnIR, rightOnIR);

            // display error
            display->write(30, "Error:" + std::to_string(error));
            break;
        }
        // case PIDType::EdgeFollower:
        //     break;
        
        // default:
        //     hello
    }

    P = error;
    I += error;
    D = error - lastError; 
    lastError = error;

    // (+) modMotorSpeed = tilting right = correct to the left
    // (-) modMotorSpeed = tilting left = correct to the right
    int modMotorSpeed = P*KP + I*KI + D*KD;
    int leftMotorSpeed = motorSpeed - modMotorSpeed; 
    int rightMotorSpeed = motorSpeed + modMotorSpeed; 

    display->write(40, "Left Motor Speed: " + std::to_string(leftMotorSpeed));
    display->write(50, "Rigth Motor Speed: " + std::to_string(rightMotorSpeed));
    
    // set new motor speeds
    leftMotor->setSpeed(leftMotorSpeed);
    rightMotor->setSpeed(rightMotorSpeed);
    return error;
}

bool PID::sensorOnWhite(int reading, int threshold) {
    if (reading < threshold) {
        return true;
    }
    return false;
}

bool PID::sensorOnIR(int reading, int threshold) {
    if (reading > threshold) {
        return true;
    }
    return false;
}

int PID::getTapeError(bool leftOnWhite, bool centreOnWhite, bool rightOnWhite) {
    // TRUTH TABLE
    // WHITE, WHITE, WHITE, lastError > 0: error = three off 
    // BLACK, WHITE, WHITE:                error = two off
    // BLACK, BLACK, WHITE:                error = one off
    // WHITE, BLACK, WHITE:                error = none off
    // WHITE, BLACK, BLACK:                error = -one off
    // WHITE, WHITE, BLACK:                error = -two off
    // WHITE, WHITE, WHITE, lastError < 0: error = -three off

    int error = TAPE_ON;
    if (leftOnWhite && centreOnWhite && rightOnWhite) {
        if (lastError > 0) {
            error = TAPE_THREE_OFF; // lost tape completely
        }
        else if (lastError < 0) {
            error = -TAPE_THREE_OFF; // lost tape completely
        }
    } 
    else if (!leftOnWhite && centreOnWhite && rightOnWhite) error = TAPE_TWO_OFF;
    else if (leftOnWhite && centreOnWhite && !rightOnWhite) error = -TAPE_TWO_OFF;
    else if (!leftOnWhite && !centreOnWhite && rightOnWhite) error = TAPE_ONE_OFF;
    else if (leftOnWhite && !centreOnWhite && !rightOnWhite) error = -TAPE_ONE_OFF;
    else if (leftOnWhite && !centreOnWhite && rightOnWhite) error = TAPE_ON;
    else if (!leftOnWhite && !centreOnWhite && !rightOnWhite) error = T_STOP;
    else error = TAPE_ON;

    return error;
}

int PID::getIRError(bool leftOnIR, bool centreOnIR, bool rightOnIR) {
    // TRUTH TABLE
    // OFF, OFF, OFF, lastError > 0: error = three off 
    // ON, OFF, OFF :                error = two off
    // ON, ON, OFF  :                error = one off
    // OFF, ON, OFF :                error = none off
    // OFF, ON, ON  :                error = -one off
    // OFF, OFF, ON :                error = -two off
    // OFF, OFF, OFF, lastError < 0: error = -three off

    int error = ON_TEN_K;
    if (!leftOnIR && !centreOnIR && !rightOnIR) {
        if (this->lastError > 0) {
            error = IR_THREE_OFF;
        }
        else if (this->lastError < 0) {
            error = -IR_THREE_OFF;
        }
    }
    else if (leftOnIR && !centreOnIR && !rightOnIR) {
        error = IR_TWO_OFF;
    }
    else if (leftOnIR && centreOnIR && !rightOnIR) {
        error = IR_ONE_OFF;
    }
    else if (!leftOnIR && centreOnIR && !rightOnIR) {
        error = ON_TEN_K;
    }
    else if (!leftOnIR && centreOnIR && rightOnIR) {
        error = -IR_ONE_OFF;
    }
    else if (!leftOnIR && !centreOnIR && rightOnIR) {
        error = -IR_TWO_OFF;
    }

    return error;
}


