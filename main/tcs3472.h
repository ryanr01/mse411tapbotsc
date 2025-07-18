#ifndef TCS3472_H
#define TCS3472_H

#include "driver/i2c.h"

typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
} tcs3472_rgbc_data_t;

esp_err_t tcs3472_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
esp_err_t tcs3472_read_colors(tcs3472_rgbc_data_t *data);
const char* tcs3472_detect_color(tcs3472_rgbc_data_t data);

#endif // TCS3472_H
