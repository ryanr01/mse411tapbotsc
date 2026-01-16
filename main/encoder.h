#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
float encoder_get_distance_mm(void);
void encoder_init(gpio_num_t pin_a, gpio_num_t pin_b);
int encoder_get_position(void);
void encoder_reset_position(void);

#endif // ENCODER_H
