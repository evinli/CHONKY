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
            // Get reflectance sensor readings
            leftReading = analogRead(LEFT_TAPE_SENSOR);
            centreReading = analogRead(CENTER_TAPE_SENSOR);
            rightReading = analogRead(RIGHT_TAPE_SENSOR);

            // Display readings
            // display->write(10, "Left Reading:" + std::to_string(leftReading));
            // display->write(20, "Centre Reading:" + std::to_string(centreReading));
            // display->write(30, "Right Reading:" + std::to_string(rightReading));

            // Get tape error
            bool leftOnWhite = sensorOnWhite(leftReading, TAPE_WHITE_THRESHOLD);
            bool centreOnWhite = sensorOnWhite(centreReading, TAPE_WHITE_THRESHOLD);
            bool rightOnWhite = sensorOnWhite(rightReading, TAPE_WHITE_THRESHOLD);
            error = getTapeError(leftOnWhite, centreOnWhite, rightOnWhite);

            // Display tape error 
            // display->write(40, "Error:" + std::to_string(error));

            // Return error immediately if at chicken wire or t-stop
            if (error == ALL_HIGH) {
                return error;
            }
            break;
        }
        
        case PIDType::IRFollower: {
            // Get IR sensor readings
            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            leftReading = analogRead(IR_LEFT_DETECT);
            
            // digitalWrite(IR_MOSFET, HIGH);
            // delayMicroseconds(300);
            // digitalWrite(IR_MOSFET, LOW);
            // delayMicroseconds(25);
            // centreReading = analogRead(IR_CENTRE_DETECT);

            digitalWrite(IR_MOSFET, HIGH);
            delayMicroseconds(300);
            digitalWrite(IR_MOSFET, LOW);
            delayMicroseconds(25);
            rightReading = analogRead(IR_RIGHT_DETECT);

            // Display readings
            // display->write(10, "Left Reading IR:" + std::to_string(leftReading));
            // display->write(20, "Centre Reading IR:" + std::to_string(centreReading));
            // display->write(30, "Right Reading IR:" + std::to_string(rightReading));
            
            // Determine sensor state
            bool leftOnIR = sensorOnIR(leftReading, IR_THRESHOLD);
            // bool centreOnIR = sensorOnIR(centreReading, IR_THRESHOLD);
            bool rightOnIR = sensorOnIR(rightReading, IR_THRESHOLD);

            // Get IR error
            error = getIRError(leftOnIR, rightOnIR);

            // Display error
            // display->write(40, "Error:" + std::to_string(error));
            break;
        }

        case PIDType::EdgeFollower: {
            break;
        }
        
        default: {
            break;
        }
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

    // Set new motor speeds
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
    // BLACK, BLACK, BLACK:                error = all high

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
    else if (!leftOnWhite && !centreOnWhite && !rightOnWhite) error = ALL_HIGH;
    else error = TAPE_ON;

    return error;
}

bool PID::allOnWhite() {
    int leftReading = analogRead(LEFT_TAPE_SENSOR);
    int centreReading = analogRead(CENTER_TAPE_SENSOR);
    int rightReading = analogRead(RIGHT_TAPE_SENSOR);

    bool leftOnWhite = sensorOnWhite(leftReading, TAPE_WHITE_THRESHOLD);
    bool centreOnWhite = sensorOnWhite(centreReading, TAPE_WHITE_THRESHOLD);
    bool rightOnWhite = sensorOnWhite(rightReading, TAPE_WHITE_THRESHOLD);

    return leftOnWhite && centreOnWhite && rightOnWhite;
}

bool PID::allOnBlack() {
    int leftReading = analogRead(LEFT_TAPE_SENSOR);
    int centreReading = analogRead(CENTER_TAPE_SENSOR);
    int rightReading = analogRead(RIGHT_TAPE_SENSOR);

    bool leftOnWhite = sensorOnWhite(leftReading, TAPE_WHITE_THRESHOLD);
    bool centreOnWhite = sensorOnWhite(centreReading, TAPE_WHITE_THRESHOLD);
    bool rightOnWhite = sensorOnWhite(rightReading, TAPE_WHITE_THRESHOLD);

    return !leftOnWhite && !centreOnWhite && !rightOnWhite;
}

bool PID::centreOnWhite() {
     int centreReading = analogRead(CENTER_TAPE_SENSOR);
     bool centreOnWhite = sensorOnWhite(centreReading, TAPE_WHITE_THRESHOLD);

     return centreOnWhite;
}


int PID::getIRError(bool leftOnIR, bool rightOnIR) {
    // TRUTH TABLE
    // ON, OFF                       error = one off
    // OFF, OFF:                     error = none off
    // OFF, ON                       error = -one off

    // OFF, OFF, OFF, lastError > 0: error = three off 
    // ON, OFF, OFF :                error = two off
    // ON, ON, OFF  :                error = one off
    // OFF, ON, OFF :                error = none off
    // OFF, ON, ON  :                error = -one off
    // OFF, OFF, ON :                error = -two off
    // OFF, OFF, OFF, lastError < 0: error = -three off

    int error = ON_TEN_K;
    if (!leftOnIR && !rightOnIR) error = ON_TEN_K;
    else if (leftOnIR && !rightOnIR) error = IR_ONE_OFF;
    else if (!leftOnIR && rightOnIR) error = -IR_ONE_OFF;
    else error = ON_TEN_K;

    return error;

    // if (!leftOnIR && !centreOnIR && !rightOnIR) {
    //     if (this->lastError > 0) {
    //         error = IR_THREE_OFF;
    //     }
    //     else if (this->lastError < 0) {
    //         error = -IR_THREE_OFF;
    //     }
    // }
    // else if (leftOnIR && !centreOnIR && !rightOnIR) error = IR_TWO_OFF;
    // else if (leftOnIR && centreOnIR && !rightOnIR) error = IR_ONE_OFF;
    // else if (!leftOnIR && centreOnIR && !rightOnIR) error = ON_TEN_K;
    // else if (!leftOnIR && centreOnIR && rightOnIR) error = -IR_ONE_OFF;
    // else if (!leftOnIR && !centreOnIR && rightOnIR) error = -IR_TWO_OFF;
    // else error = ON_TEN_K;
}

bool PID::refindTape(int sideToSweep, long maxSweepTime) {
    long startOfSweep = millis();
    bool foundTape = true;

    while (allOnWhite()) {
        if (millis() - startOfSweep < MAX_SWEEP_TIME) {
            if (sideToSweep == LEFT_SIDE) {
                leftMotor->setSpeed(-SWEEP_SPEED);
                rightMotor->setSpeed(SWEEP_SPEED);
            }
            if (sideToSweep == RIGHT_SIDE) {
                leftMotor->setSpeed(SWEEP_SPEED);
                rightMotor->setSpeed(-SWEEP_SPEED);
            }
        }
        else {
            foundTape = false;
            break;
        }
    }

    leftMotor->stop();
    rightMotor->stop();
    return foundTape;
}

void PID::resetPID() {
    lastError = 0;
}


