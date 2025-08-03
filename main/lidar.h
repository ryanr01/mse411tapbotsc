#ifndef LIDAR_H
#define LIDAR_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_PORT 0
#define I2C_MASTER_FREQ_HZ 400000
#define VL53L0X_I2C_ADDR 0x29

void lidar_init(void);
float read_from_lidar(void);

#endif