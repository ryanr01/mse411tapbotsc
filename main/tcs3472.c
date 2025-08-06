#include "tcs3472.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define TAG "TCS3472"

#define TCS3472_ADDR 0x29
#define TCS_CMD_BIT 0x80
#define I2C_FREQ_HZ 100000

#define ENABLE_REG 0x00
#define ATIME_REG  0x01
#define CONTROL_REG 0x0F
#define CDATAL 0x14

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t i2c_dev;

static esp_err_t write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {TCS_CMD_BIT | reg, value};
    return i2c_master_transmit(i2c_dev, data, sizeof(data), -1);
}

esp_err_t tcs3472_init(gpio_num_t sda, gpio_num_t scl) {
    i2c_master_bus_config_t bus_conf = {
        
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &i2c_bus));

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS3472_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_conf, &i2c_dev));

    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(write_register(ENABLE_REG, 0x03));     // Power on, ADC enabled
    ESP_ERROR_CHECK(write_register(ATIME_REG, 0xD5));      // Integration time
    ESP_ERROR_CHECK(write_register(CONTROL_REG, 0x01));    // Gain

    return ESP_OK;
}

esp_err_t tcs3472_read_colors(tcs3472_rgbc_data_t *data) {
    uint8_t reg = TCS_CMD_BIT | CDATAL;
    uint8_t buf[8];

    esp_err_t err = i2c_master_transmit_receive(i2c_dev, &reg, 1, buf, 8, -1);
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
