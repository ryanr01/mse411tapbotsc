#include "vl53l0x_i2c_platform.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

VL53L0X_Error VL53L0X_write_multi(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    if (count >= 256) return VL53L0X_ERROR_INVALID_PARAMS;  // Safety check

    uint8_t buf[256 + 1];  // Max reasonable size
    buf[0] = index;
    memcpy(&buf[1], pdata, count);

    esp_err_t ret = i2c_master_transmit(Dev->i2c_handle, buf, count + 1, -1);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_read_multi(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    if (count >= 256) return VL53L0X_ERROR_INVALID_PARAMS;  // Safety check

    esp_err_t ret = i2c_master_transmit_receive(Dev->i2c_handle, &index, 1, pdata, count, -1);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_write_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    uint8_t buf[2] = {index, data};
    esp_err_t ret = i2c_master_transmit(Dev->i2c_handle, buf, 2, -1);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_write_word(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    uint8_t buf[3] = {index, (data >> 8) & 0xFF, data & 0xFF};
    esp_err_t ret = i2c_master_transmit(Dev->i2c_handle, buf, 3, -1);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_write_dword(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    uint8_t buf[5] = {index,
                      (data >> 24) & 0xFF,
                      (data >> 16) & 0xFF,
                      (data >> 8) & 0xFF,
                      data & 0xFF};
    esp_err_t ret = i2c_master_transmit(Dev->i2c_handle, buf, 5, -1);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_read_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata) {
    esp_err_t ret = i2c_master_transmit_receive(Dev->i2c_handle, &index, 1, pdata, 1, -1);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_read_word(VL53L0X_DEV Dev, uint8_t index, uint16_t *pdata) {
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(Dev->i2c_handle, &index, 1, buf, 2, -1);
    if (ret == ESP_OK) {
        *pdata = (buf[0] << 8) | buf[1];
    }
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_read_dword(VL53L0X_DEV Dev, uint8_t index, uint32_t *pdata) {
    uint8_t buf[4];
    esp_err_t ret = i2c_master_transmit_receive(Dev->i2c_handle, &index, 1, buf, 4, -1);
    if (ret == ESP_OK) {
        *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    }
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}