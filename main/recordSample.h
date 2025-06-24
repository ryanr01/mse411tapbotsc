#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "format_wav.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include <time.h>

// Initialize solenoid pins
#define ENB 4
#define IN3 17
#define IN4 8

// MIC GPIO Pins
#define MIC_I2S_CLK_GPIO 40
#define MIC_I2S_DATA_GPIO 19
#define MIC_I2S_LRCL 5 // A.k.A. WS "Word Select"

// SPI2 GPIO pins
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// Recording configuration
#define MIC_SAMPLE_RATE 48000 // Do not exceed 64000 for SPH0645
#define MIC_BIT_SAMPLE 32
#define RECORD_TIME_MS 200
#define SAMPLES_TO_RECORD  ((MIC_SAMPLE_RATE * RECORD_TIME_MS) / 1000)
#define SPI_DMA_CHAN        SPI_DMA_CH_AUTO
#define NUM_CHANNELS        2
#define SAMPLE_SIZE         (MIC_BIT_SAMPLE * 1024) // RAM buffer size
#define BYTE_RATE           (MIC_SAMPLE_RATE * (MIC_BIT_SAMPLE / 8)) * NUM_CHANNELS
#define TAG "SD_SPI"
#define MOUNT_POINT "/s"

void RTOS_go(void);
void solenoid_init(void);
void fire_solenoid_once(void *pvParameters);
void record_wav(void *pvParameters);
void init_microphone(void);
void replace_char(char *str, char old_char, char new_char);
void record_sample(int record_time, char *data_label, float x_coordinate, float y_coordinate);

