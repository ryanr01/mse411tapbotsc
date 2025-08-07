#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
// Solenoid pins
#define ENB 3
#define IN3 5
#define IN4 4

// Wheel & Encoder Constants

#define COUNTS_PER_REV 2525
#define WHEEL_DIAMETER_MM 80.0
#define PI 3.14159265359

// DC motor control pins
#define MOTOR_IN1 GPIO_NUM_8
#define MOTOR_IN2 GPIO_NUM_3
#define MOTOR_IN3 GPIO_NUM_18
#define MOTOR_IN4 GPIO_NUM_17

// Encorder pins
#define ENCODER_PIN_A GPIO_NUM_9
#define ENCODER_PIN_B GPIO_NUM_46

// Microphone I2S pins
#define MIC_I2S_CLK_GPIO  40 //  G
#define MIC_I2S_DATA_GPIO 19 // G
#define MIC_I2S_LRCL      GPIO_NUM_2 // A.k.A. WS "Word Select"    G

//SDMMC GPIO Pins
#define SDMMC_CLK   12
#define SDMMC_CMD   11
#define SDMMC_D0    36
#define SDMMC_D1    37
#define SDMMC_D2    13
#define SDMMC_D3_CS 10

// Stepper motor control pins
#define BOT_STEP_MOTOR_GPIO_DIR  38
#define BOT_STEP_MOTOR_GPIO_STEP 39
#define BOT_END_LIMIT_GPIO        7

#define TOP_STEP_MOTOR_GPIO_DIR  35
#define TOP_STEP_MOTOR_GPIO_STEP 0
#define TOP_END_LIMIT_GPIO       42

// Sensor and push button pins
#define STOP_PB_GPIO           21
#define START_PB_GPIO          47
#define TAPBOT_RESET_PB_GPIO   48
// // Lidar pins
// // Implementation is a little scuffed for now
// // So make sure then pin defs match these in lidar.h
// #define I2C_MASTER_SCL_IO 19
// #define I2C_MASTER_SDA_IO 18
// #define I2C_MASTER_PORT 0

#endif // CONFIG_H