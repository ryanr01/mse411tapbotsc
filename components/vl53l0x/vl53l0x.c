#include "vl53l0x.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_check.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

static const char *TAG = "VL53L0X";

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), -1);
}

static esp_err_t write_multi(i2c_master_dev_handle_t dev, uint16_t reg, const uint8_t *data, size_t len)
{
    uint8_t *buf = malloc(len + 2);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    memcpy(buf + 2, data, len);
    esp_err_t err = i2c_master_transmit(dev, buf, len + 2, -1);
    free(buf);
    return err;
}

static esp_err_t read_reg(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t *val)
{
    uint8_t regbuf[2] = { reg >> 8, reg & 0xFF };
    return i2c_master_transmit_receive(dev, regbuf, sizeof(regbuf), val, 1, -1);
}

static esp_err_t read_reg16(i2c_master_dev_handle_t dev, uint16_t reg, uint16_t *val)
{
    uint8_t regbuf[2] = { reg >> 8, reg & 0xFF };
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(dev, regbuf, sizeof(regbuf), data, 2, -1);
    if (err == ESP_OK) {
        *val = ((uint16_t)data[0] << 8) | data[1];
    }
    return err;
}

#include "core/inc/vl53l0x_tuning.h"

esp_err_t vl53l0x_init(i2c_master_dev_handle_t dev)
{
    // Load default tuning settings from ST header
    for (size_t i = 0; i + 2 < sizeof(DefaultTuningSettings); i += 3) {
        uint8_t type = DefaultTuningSettings[i];
        uint8_t reg_high = DefaultTuningSettings[i + 1];
        uint8_t val = DefaultTuningSettings[i + 2];
        if (type == 0 && reg_high == 0 && val == 0) {
            break;
        }
        uint16_t reg = ((uint16_t)type << 8) | reg_high;
        esp_err_t err = write_reg(dev, reg, val);
        if (err != ESP_OK) {
            return err;
        }
    }

    // ST data init sequence to release device from reset
    ESP_RETURN_ON_ERROR(write_reg(dev, 0x80, 0x01), TAG, "init stage 1");
    ESP_RETURN_ON_ERROR(write_reg(dev, 0xFF, 0x01), TAG, "init stage 1");
    ESP_RETURN_ON_ERROR(write_reg(dev, 0x00, 0x00), TAG, "init stage 1");
    uint8_t stop_var = 0;
    ESP_RETURN_ON_ERROR(read_reg(dev, 0x91, &stop_var), TAG, "read stop_var");
    ESP_RETURN_ON_ERROR(write_reg(dev, 0x00, 0x01), TAG, "init stage 1");
    ESP_RETURN_ON_ERROR(write_reg(dev, 0xFF, 0x00), TAG, "init stage 1");
    ESP_RETURN_ON_ERROR(write_reg(dev, 0x80, 0x00), TAG, "init stage 1");

    // start continuous ranging in back-to-back mode
    return write_reg(dev, 0x000, 0x02);
}

int vl53l0x_read_range_mm(i2c_master_dev_handle_t dev)
{
    uint8_t status = 0;
    int timeout = 20;
    while (timeout--) {
        if (read_reg(dev, 0x0013, &status) != ESP_OK) {
            return -1;
        }
        if (status & 0x07) {
            break;
        }
        vTaskDelay(10);
    }
    if (!(status & 0x07)) {
        return -1;
    }

    uint16_t range = 0;
    if (read_reg16(dev, 0x0014 + 10, &range) != ESP_OK) {
        return -1;
    }

    write_reg(dev, 0x0B, 0x01); // clear interrupt
    return range;
}
