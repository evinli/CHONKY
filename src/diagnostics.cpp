/**
 * @file      diagnostics.cpp
 * @author    Creators of CHONKY 
 * @brief     Class for running diagnostics on the Bluepill
 */

#include "diagnostics.h"

/////////////////// Methods ///////////////////
void Diagnostics::blinkLED() {
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < BLINK_COUNT; i++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
    } 
}

void Diagnostics::writeDigital() {
    for (int i = PA0; i <= PB15; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, HIGH);
    }
}

void Diagnostics::writePWM() {
    PinName PWM_pins[NUM_PWM_PINS] = {PA_0, PA_1, PA_2, PA_3, PA_6,\
                                      PA_7, PA_8, PA_9, PA_10, PA_11,\
                                      PB_0, PB_1, PB_6, PB_7, PB_8, PB_9};
    for (int i = 0; i < NUM_PWM_PINS; i++) {
        pinMode(PWM_pins[i], OUTPUT);
        pwm_start(PWM_pins[i], MOTOR_FREQ, 2000, RESOLUTION_12B_COMPARE_FORMAT);
    }
}

void Diagnostics::readDigital() {
    Serial.begin(9600);
    for (int i = PA0; i <= PB15; i++) {
        if (i != PA9 && i != PA10) {
            pinMode(i, INPUT_PULLDOWN);
            Serial.print(i);
            Serial.print(": ");
            // Serial.println(digitalRead(i));
            delay(500);
        }
    }
    // pinMode(PB12, INPUT_PULLDOWN);
    // Serial.println(digitalRead(PB12));
    Serial.end();
}

void Diagnostics::readAnalog() {
    Serial.begin(9600);
    int analog_pins[NUM_ANALOG_PINS] = {PA0, PA1, PA2, PA3,\
                                        PA4, PA5, PA6, PA7, PB0, PB1};
    for (int i = 0; i < NUM_ANALOG_PINS; i++) {
        pinMode(analog_pins[i], INPUT_PULLDOWN);
        Serial.print(i);
        Serial.print(": ");
        Serial.print(analogRead(analog_pins[i]));
        Serial.println();
        delay(500);
    }
}