#include <Arduino.h>
#include <stdio.h>

// Pin assignments

// Stepper motor 1 (top)
const uint8_t STEP1_DIR_PIN = 50;   // output
const uint8_t STEP1_STEP_PIN = 51;  // output
const uint8_t STEP1_LIMIT_PIN = 42; // top limit switch, pull-up

// Stepper motor 2 (bottom)
const uint8_t STEP2_DIR_PIN = 44;   // output
const uint8_t STEP2_STEP_PIN = 45;  // output
const uint8_t STEP2_LIMIT_PIN = 43; // bottom limit switch, pull-up

// DC motors driven by time (no encoders)
const uint8_t DC1_PIN = 49;         // motor 1 control
const uint8_t DC2_PIN = 48;         // motor 2 control

// Solenoids (single-pin control)
const uint8_t SOL1_PIN = 52;        // top solenoid
const uint8_t SOL2_PIN = 53;        // bottom solenoid

// Push buttons (using internal pull-ups)
const uint8_t STOP_BTN_PIN  = 39;   // stop button
const uint8_t START_BTN_PIN = 40;   // start button
const uint8_t RESET_BTN_PIN = 41;   // reset button

// Stepper configuration
const float STEPS_PER_CM = 100.0;   // steps required for 1 cm
const unsigned int STEP_PULSE_US = 500; // microsecond delay for pulses
const unsigned long DC_RUN_MS = 1000;   // run time in ms for ~1 cm

// State machine
enum State { IDLE, RESET, SEQUENCE, DONE };

volatile State currentState = IDLE;

// Helper to print state names
const char* stateToStr(State s) {
    switch (s) {
        case IDLE: return "IDLE";
        case RESET: return "RESET";
        case SEQUENCE: return "SEQUENCE";
        case DONE: return "DONE";
        default: return "UNKNOWN";
    }
}

// Flags set by interrupts
volatile bool startPressed = false;
volatile bool resetPressed = false;
volatile bool stopPressed  = false;

// --- Interrupt service routines ---
void startButtonISR() {
    startPressed = true;
    printf("startPressed set to %d\n", startPressed);
}
void resetButtonISR() {
    resetPressed = true;
    printf("resetPressed set to %d\n", resetPressed);
}
void stopButtonISR()  {
    stopPressed  = true;
    printf("stopPressed set to %d\n", stopPressed);
}

// --- Hardware helpers ---
void initStepper(uint8_t dirPin, uint8_t stepPin) {
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
}

// Step both X-axis steppers simultaneously for a given distance
void stepBothDistance(float cm, bool direction) {
    long steps = (long)(cm * STEPS_PER_CM);
    printf("steps calculated as %ld for cm=%f\n", steps, cm);
    digitalWrite(STEP1_DIR_PIN, direction ? HIGH : LOW);
    digitalWrite(STEP2_DIR_PIN, direction ? HIGH : LOW);
    for (long i = 0; i < steps && !stopPressed; ++i) {
        printf("stepping both motors, step %ld of %ld\n", i + 1, steps);
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

void initDcMotor(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void runDcMotor(uint8_t pin, unsigned long duration_ms) {
    digitalWrite(pin, HIGH);
    unsigned long start = millis();
    printf("DC motor on pin %u started at %lu\n", pin, start);
    while (millis() - start < duration_ms && !stopPressed) {
        delay(1);
    }
    digitalWrite(pin, LOW);
    printf("DC motor on pin %u stopped after %lu ms\n", pin, millis() - start);
}

// Drive both Y-axis DC motors concurrently for a specified time
void runDcPair(unsigned long duration_ms) {
    digitalWrite(DC1_PIN, HIGH);
    digitalWrite(DC2_PIN, HIGH);
    unsigned long start = millis();
    printf("DC pair started at %lu\n", start);
    while (millis() - start < duration_ms && !stopPressed) {
        delay(1);
    }
    digitalWrite(DC1_PIN, LOW);
    digitalWrite(DC2_PIN, LOW);
    printf("DC pair stopped after %lu ms\n", millis() - start);
}

void initSolenoid(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void fireSolenoid(uint8_t pin, uint16_t pulse_ms) {
    digitalWrite(pin, HIGH);
    delay(pulse_ms);
    digitalWrite(pin, LOW);
}

// Perform homing for both steppers until their optical switches trigger
void homing() {
    // Drive steppers toward their respective limit switches
    digitalWrite(STEP1_DIR_PIN, HIGH);   // home direction for stepper 1
    digitalWrite(STEP2_DIR_PIN, LOW);    // home direction for stepper 2

    bool step1Homed = false;
    bool step2Homed = false;
    printf("step1Homed initialized to %d\n", step1Homed);
    printf("step2Homed initialized to %d\n", step2Homed);

    while ((!step1Homed || !step2Homed) && !stopPressed) {
        if (!step1Homed) {
            digitalWrite(STEP1_STEP_PIN, HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP1_STEP_PIN, LOW);
            delayMicroseconds(STEP_PULSE_US);
            if (digitalRead(STEP1_LIMIT_PIN) == LOW) {
                step1Homed = true;
                printf("step1Homed changed to %d\n", step1Homed);
            }
        }

        if (!step2Homed) {
            digitalWrite(STEP2_STEP_PIN, HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP2_STEP_PIN, LOW);
            delayMicroseconds(STEP_PULSE_US);
            if (digitalRead(STEP2_LIMIT_PIN) == LOW) {
                step2Homed = true;
                printf("step2Homed changed to %d\n", step2Homed);
            }
        }
    }
}

// Test mode traverses the blade in a bidirectional raster pattern
void testMode() {
    const uint8_t X_TRAVEL_CM = 30;   // width to scan in X
    const uint8_t Y_TRAVEL_CM = 30;   // length to scan in Y
    bool xDirection = true;           // start moving in positive X
    printf("xDirection initialized to %d\n", xDirection);

    for (uint8_t y = 0; y < Y_TRAVEL_CM && !stopPressed; ++y) {
        printf("y incremented to %u\n", y);
        for (uint8_t x = 0; x < X_TRAVEL_CM && !stopPressed; ++x) {
            printf("x incremented to %u\n", x);
            stepBothDistance(1.0, xDirection);     // move 1 cm in X
            fireSolenoid(SOL1_PIN, 50);
            fireSolenoid(SOL2_PIN, 50);
        }
        if (y < Y_TRAVEL_CM - 1) {
            runDcPair(DC_RUN_MS);                  // advance 1 cm in Y
        }
        xDirection = !xDirection;                  // reverse X direction
        printf("xDirection toggled to %d\n", xDirection);
    }
}

// --- Setup ---
void setup() {
    Serial.begin(9600);
    printf("currentState initialized to %s\n", stateToStr(currentState));
    printf("startPressed initialized to %d\n", startPressed);
    printf("resetPressed initialized to %d\n", resetPressed);
    printf("stopPressed initialized to %d\n", stopPressed);
    // Stepper motors
    initStepper(STEP1_DIR_PIN, STEP1_STEP_PIN);
    initStepper(STEP2_DIR_PIN, STEP2_STEP_PIN);
    pinMode(STEP1_LIMIT_PIN, INPUT_PULLUP);
    pinMode(STEP2_LIMIT_PIN, INPUT_PULLUP);

    // DC motors
    initDcMotor(DC1_PIN);
    initDcMotor(DC2_PIN);

    // Solenoids
    initSolenoid(SOL1_PIN);
    initSolenoid(SOL2_PIN);

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
                    printf("startPressed set to %d\n", startPressed);
                    currentState = SEQUENCE;
                    printf("currentState changed to %s\n", stateToStr(currentState));
                } else if (resetPressed) {
                    resetPressed = false;
                    printf("resetPressed set to %d\n", resetPressed);
                    currentState = RESET;
                    printf("currentState changed to %s\n", stateToStr(currentState));
                }
                break;

            case RESET:
                fireSolenoid(SOL1_PIN, 100);
                homing();
                currentState = IDLE;
                printf("currentState changed to %s\n", stateToStr(currentState));
                break;

            case SEQUENCE:
                homing();
                testMode();
                currentState = DONE;
                printf("currentState changed to %s\n", stateToStr(currentState));
                break;

            case DONE:
                currentState = IDLE;
                printf("currentState changed to %s\n", stateToStr(currentState));
                break;
        }

        if (stopPressed) {
            stopPressed = false;
            printf("stopPressed set to %d\n", stopPressed);
            digitalWrite(DC1_PIN, LOW);
            digitalWrite(DC2_PIN, LOW);
            currentState = IDLE;
            printf("currentState changed to %s\n", stateToStr(currentState));
        }

        delay(100);
    }
}

