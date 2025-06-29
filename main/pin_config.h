#pragma once

// Solenoid control pins
#define ENB 4
#define IN3 38
#define IN4 39

// DC motor control pins
#define DC_MOTOR_IN1 18
#define DC_MOTOR_IN2 8
#define DC_MOTOR_IN3 3
#define DC_MOTOR_IN4 46

// Microphone I2S pins
#define MIC_I2S_CLK_GPIO   40
#define MIC_I2S_DATA_GPIO  19
#define MIC_I2S_LRCL        5

// SPI bus pins for SD card
#define PIN_NUM_MISO    13
#define PIN_NUM_MOSI    11
#define PIN_NUM_CLK     12
#define PIN_NUM_CS      10

// Stepper motor control pins
#define STEP_MOTOR_GPIO_EN   6
#define STEP_MOTOR_GPIO_DIR  15
#define STEP_MOTOR_GPIO_STEP 16

// Sensor and push button pins
#define TOP_END_LIMIT_GPIO     7
#define STOP_PB_GPIO           21
#define START_PB_GPIO          47
#define CARRIER_RESET_PB_GPIO  45
#define TAPBOT_RESET_PB_GPIO   48
