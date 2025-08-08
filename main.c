#include <Arduino.h>

// Pin assignments

// Stepper motor 1 (DRV8825)
const uint8_t STEP1_DIR_PIN = 2;    // output
const uint8_t STEP1_STEP_PIN = 3;   // output
const uint8_t STEP1_LIMIT_PIN = 4;  // optical sensor, uses pull-up

// Stepper motor 2
const uint8_t STEP2_DIR_PIN = 5;    // output
const uint8_t STEP2_STEP_PIN = 6;   // output
const uint8_t STEP2_LIMIT_PIN = 7;  // optical sensor, uses pull-up

// DC motor 1 with encoder (L298N)
const uint8_t DC1_DIR_PIN = 8;      // direction output
const uint8_t DC1_EN_PIN  = 9;      // enable output (pulsed)
const uint8_t DC1_ENC_PIN = 10;     // encoder input, pull-up

// DC motor 2 with encoder (L298N)
const uint8_t DC2_DIR_PIN = 11;     // direction output
const uint8_t DC2_EN_PIN  = 12;     // enable output (pulsed)
const uint8_t DC2_ENC_PIN = 13;     // encoder input, pull-up

// Solenoids driven through L298N
const uint8_t SOL1_IN_PIN = A0;     // output to IN3
const uint8_t SOL1_EN_PIN = A1;     // output to ENB
const uint8_t SOL2_IN_PIN = A2;     // output to IN4
const uint8_t SOL2_EN_PIN = A3;     // output to ENB

// Push buttons (using internal pull-ups)
const uint8_t START_BTN_PIN = A4;   // start button, pull-up
const uint8_t RESET_BTN_PIN = A5;   // reset button, pull-up
const uint8_t STOP_BTN_PIN  = A6;   // stop button, pull-up

// Stepper configuration
const float STEPS_PER_CM = 100.0;   // steps required for 1 cm
const unsigned int STEP_PULSE_US = 500; // microsecond delay for pulses

// State machine
enum State { IDLE, RESET, SEQUENCE, DONE };
volatile State currentState = IDLE;

// Flags set by interrupts
volatile bool startPressed = false;
volatile bool resetPressed = false;
volatile bool stopPressed  = false;

// Encoder counters
volatile long dc1Count = 0;
volatile long dc2Count = 0;

// --- Interrupt service routines ---
void startButtonISR() { startPressed = true; }
void resetButtonISR() { resetPressed = true; }
void stopButtonISR()  { stopPressed  = true; }

void dc1EncoderISR() { dc1Count++; }
void dc2EncoderISR() { dc2Count++; }

// --- Hardware helpers ---
void initStepper(uint8_t dirPin, uint8_t stepPin) {
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
}

// Step both X-axis steppers simultaneously for a given distance
void stepBothDistance(float cm, bool direction) {
    long steps = (long)(cm * STEPS_PER_CM);
    digitalWrite(STEP1_DIR_PIN, direction ? HIGH : LOW);
    digitalWrite(STEP2_DIR_PIN, direction ? HIGH : LOW);
    for (long i = 0; i < steps && !stopPressed; ++i) {
        digitalWrite(STEP1_STEP_PIN, HIGH);
        digitalWrite(STEP2_STEP_PIN, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(STEP1_STEP_PIN, LOW);
        digitalWrite(STEP2_STEP_PIN, LOW);
        delayMicroseconds(STEP_PULSE_US);
    }
}

void homeStepper(uint8_t dirPin, uint8_t stepPin, uint8_t limitPin, bool direction) {
    digitalWrite(dirPin, direction ? HIGH : LOW);
    while (digitalRead(limitPin) == HIGH && !stopPressed) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(STEP_PULSE_US);
    }
}

void initDcMotor(uint8_t dirPin, uint8_t enPin, uint8_t encPin, void (*isr)()) {
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(encPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encPin), isr, RISING);
    digitalWrite(dirPin, LOW);
    digitalWrite(enPin, LOW);
}

const float COUNTS_PER_MM = 1.0; // encoder counts per millimeter

void driveDcMotor(uint8_t dirPin, uint8_t enPin, volatile long *count,
                  float distance_mm, bool direction) {
    long target = (long)(distance_mm * COUNTS_PER_MM);
    *count = 0;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    while (*count < target) {
        digitalWrite(enPin, HIGH);
        delay(10);
        digitalWrite(enPin, LOW);
        delay(10);
        if (stopPressed) break;
    }
    digitalWrite(enPin, LOW);
}

// Drive both Y-axis DC motors concurrently for a distance in millimeters
void driveDcPair(float distance_mm, bool direction) {
    long target = (long)(distance_mm * COUNTS_PER_MM);
    dc1Count = 0;
    dc2Count = 0;
    digitalWrite(DC1_DIR_PIN, direction ? HIGH : LOW);
    digitalWrite(DC2_DIR_PIN, direction ? HIGH : LOW);
    while ((dc1Count < target || dc2Count < target) && !stopPressed) {
        if (dc1Count < target) {
            digitalWrite(DC1_EN_PIN, HIGH);
        }
        if (dc2Count < target) {
            digitalWrite(DC2_EN_PIN, HIGH);
        }
        delay(10);
        digitalWrite(DC1_EN_PIN, LOW);
        digitalWrite(DC2_EN_PIN, LOW);
        delay(10);
    }
    digitalWrite(DC1_EN_PIN, LOW);
    digitalWrite(DC2_EN_PIN, LOW);
}

void initSolenoid(uint8_t inPin, uint8_t enPin) {
    pinMode(inPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(inPin, LOW);
    digitalWrite(enPin, LOW);
}

void fireSolenoid(uint8_t inPin, uint8_t enPin, uint16_t pulse_ms) {
    digitalWrite(enPin, HIGH);
    digitalWrite(inPin, HIGH);
    delay(pulse_ms);
    digitalWrite(inPin, LOW);
    digitalWrite(enPin, LOW);
}

// Perform homing for both steppers until their optical switches trigger
void homing() {
    // Drive steppers toward their respective limit switches
    digitalWrite(STEP1_DIR_PIN, HIGH);   // home direction for stepper 1
    digitalWrite(STEP2_DIR_PIN, LOW);    // home direction for stepper 2

    bool step1Homed = false;
    bool step2Homed = false;

    while ((!step1Homed || !step2Homed) && !stopPressed) {
        if (!step1Homed) {
            digitalWrite(STEP1_STEP_PIN, HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP1_STEP_PIN, LOW);
            delayMicroseconds(STEP_PULSE_US);
            if (digitalRead(STEP1_LIMIT_PIN) == LOW) {
                step1Homed = true;
            }
        }

        if (!step2Homed) {
            digitalWrite(STEP2_STEP_PIN, HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP2_STEP_PIN, LOW);
            delayMicroseconds(STEP_PULSE_US);
            if (digitalRead(STEP2_LIMIT_PIN) == LOW) {
                step2Homed = true;
            }
        }
    }
}

// Test mode traverses the blade in a bidirectional raster pattern
void testMode() {
    const uint8_t X_TRAVEL_CM = 30;   // width to scan in X
    const uint8_t Y_TRAVEL_CM = 30;   // length to scan in Y
    bool xDirection = true;           // start moving in positive X

    for (uint8_t y = 0; y < Y_TRAVEL_CM && !stopPressed; ++y) {
        for (uint8_t x = 0; x < X_TRAVEL_CM && !stopPressed; ++x) {
            stepBothDistance(1.0, xDirection);     // move 1 cm in X
            fireSolenoid(SOL1_IN_PIN, SOL1_EN_PIN, 50);
            fireSolenoid(SOL2_IN_PIN, SOL2_EN_PIN, 50);
        }
        if (y < Y_TRAVEL_CM - 1) {
            driveDcPair(10.0, true);               // advance 1 cm in Y
        }
        xDirection = !xDirection;                  // reverse X direction
    }
}

// --- Setup ---
void setup() {
    // Stepper motors
    initStepper(STEP1_DIR_PIN, STEP1_STEP_PIN);
    initStepper(STEP2_DIR_PIN, STEP2_STEP_PIN);
    pinMode(STEP1_LIMIT_PIN, INPUT_PULLUP);
    pinMode(STEP2_LIMIT_PIN, INPUT_PULLUP);

    // DC motors
    initDcMotor(DC1_DIR_PIN, DC1_EN_PIN, DC1_ENC_PIN, dc1EncoderISR);
    initDcMotor(DC2_DIR_PIN, DC2_EN_PIN, DC2_ENC_PIN, dc2EncoderISR);

    // Solenoids
    initSolenoid(SOL1_IN_PIN, SOL1_EN_PIN);
    initSolenoid(SOL2_IN_PIN, SOL2_EN_PIN);

    // Push buttons
    pinMode(START_BTN_PIN, INPUT_PULLUP);
    pinMode(RESET_BTN_PIN, INPUT_PULLUP);
    pinMode(STOP_BTN_PIN,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(START_BTN_PIN), startButtonISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RESET_BTN_PIN), resetButtonISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(STOP_BTN_PIN),  stopButtonISR,  FALLING);
}

// --- Main loop ---
void loop() {
    while (1) {
        switch (currentState) {
            case IDLE:
                if (startPressed) {
                    startPressed = false;
                    currentState = SEQUENCE;
                } else if (resetPressed) {
                    resetPressed = false;
                    currentState = RESET;
                }
                break;

            case RESET:
                fireSolenoid(SOL1_IN_PIN, SOL1_EN_PIN, 100);
                homing();
                currentState = IDLE;
                break;

            case SEQUENCE:
                homing();
                testMode();
                currentState = DONE;
                break;

            case DONE:
                currentState = IDLE;
                break;
        }

        if (stopPressed) {
            stopPressed = false;
            digitalWrite(DC1_EN_PIN, LOW);
            digitalWrite(DC2_EN_PIN, LOW);
            currentState = IDLE;
        }

        delay(100);
    }
}

