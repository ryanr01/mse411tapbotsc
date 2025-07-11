#include "motor_driver.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include <math.h>
#include "pin_config.h"




static float calculate_distance(int position) {
    float circumference = PI * WHEEL_DIAMETER_MM;
    return ((float)position / COUNTS_PER_REV) * circumference;
}

void motor_driver_init(void) {
    gpio_reset_pin(MOTOR_IN1);
    gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_IN1, 0);

    gpio_reset_pin(MOTOR_IN2);
    gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_IN2, 0);

        gpio_reset_pin(MOTOR_IN3);
    gpio_set_direction(MOTOR_IN3, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_IN3, 0);

    gpio_reset_pin(MOTOR_IN4);
    gpio_set_direction(MOTOR_IN4, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_IN4, 0);
}

void motor_forward(void) {
    gpio_set_level(MOTOR_IN1, 1);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 1);
    gpio_set_level(MOTOR_IN4, 0);
}

void motor_stop(void) {
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);
}

void motor_reverse(void){

    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 1);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 1);

}

