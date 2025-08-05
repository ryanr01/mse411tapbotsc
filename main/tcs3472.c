#include "tcs3472.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "pin_config.h"
#define TAG "TCS3472"

#define TCS3472_ADDR 0x29
#define TCS_CMD_BIT 0x80
#define I2C_FREQ_HZ 100000

#define ENABLE_REG 0x00
#define ATIME_REG  0x01
#define CONTROL_REG 0x0F
#define CDATAL 0x14

static i2c_port_t i2c_port_used;

static esp_err_t write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {TCS_CMD_BIT | reg, value};
    return i2c_master_write_to_device(i2c_port_used, TCS3472_ADDR, data, 2, pdMS_TO_TICKS(100));
}

esp_err_t tcs3472_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    i2c_port_used = port;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0));

    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(write_register(ENABLE_REG, 0x03));     // Power on, ADC enabled
    ESP_ERROR_CHECK(write_register(ATIME_REG, 0xD5));      // Integration time
    ESP_ERROR_CHECK(write_register(CONTROL_REG, 0x01));    // Gain

    return ESP_OK;
}

esp_err_t tcs3472_read_colors(tcs3472_rgbc_data_t *data) {
    uint8_t reg = TCS_CMD_BIT | CDATAL;
    uint8_t buf[8];

    esp_err_t err = i2c_master_write_read_device(i2c_port_used, TCS3472_ADDR, &reg, 1, buf, 8, pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;

    data->c = buf[1] << 8 | buf[0];
    data->r = buf[3] << 8 | buf[2];
    data->g = buf[5] << 8 | buf[4];
    data->b = buf[7] << 8 | buf[6];

    return ESP_OK;
}

const char* tcs3472_detect_color(tcs3472_rgbc_data_t d) {
    if (d.c < 100) return "No color";

    if (d.r > 700 && 600 > d.b) return "Red";
    if (d.r > 200 && d.g > 200 && d.b > 200) return "White";

    return "Unknown";
}
