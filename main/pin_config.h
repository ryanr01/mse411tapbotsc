#pragma once

// Solenoid control pins
#define ENB 4
#define IN3 17
#define IN4 8

// Microphone I2S pins
#define MIC_I2S_CLK_GPIO 40
#define MIC_I2S_DATA_GPIO 19
#define MIC_I2S_LRCL 5

// SPI bus pins for SD card
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// Stepper motor control pins
#define STEP_MOTOR_GPIO_EN   6
#define STEP_MOTOR_GPIO_DIR  20
#define STEP_MOTOR_GPIO_STEP 19

// Sensor and push button pins
#define TOP_END_LIMIT_GPIO     7
#define STOP_PB_GPIO           47
#define START_PB_GPIO          48
#define CARRIER_RESET_PB_GPIO  35
#define TAPBOT_RESET_PB_GPIO   36
