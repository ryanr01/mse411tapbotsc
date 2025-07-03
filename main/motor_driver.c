#include "motor_driver.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"

#define MOTOR_IN1 GPIO_NUM_8
#define MOTOR_IN2 GPIO_NUM_3
#define MOTOR_IN3 GPIO_NUM_16
#define MOTOR_IN4 GPIO_NUM_17


#define COUNTS_PER_REV 2850
#define WHEEL_DIAMETER_MM 75.0
#define PI 3.14159265359

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




void DCmotordrive(float target_distance_mm, int direction) {
    // Reset encoder
    extern void encoder_reset_position(void);
    encoder_reset_position();
if(direction = 1){


    // Start motor
    motor_forward();

}

   if (direction = 0){
    // Reverse motor
    motor_reverse();

   }

    // Loop until target distance reached
    while (1) {
        float distance = calculate_distance(encoder_get_position());
        if (distance >= target_distance_mm) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // small delay for CPU efficiency
    }

    // Stop motor
    motor_stop();
}