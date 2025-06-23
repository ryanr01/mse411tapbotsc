#ifndef MOTOR_OPS_H
#define MOTOR_OPS_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "stepper_motor_encoder.h"
#include "recordSample.h"

// Stepper motor parameter definitions
#define STEP_MOTOR_ENABLE_LEVEL  0
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define STEP_MOTOR_RESOLUTION_HZ 1000000

typedef struct {
    int gpio_en;
    int gpio_dir;
    int gpio_step;
    rmt_channel_handle_t rmt_chan;
    rmt_encoder_handle_t accel_encoder;
    rmt_encoder_handle_t uniform_encoder;
    rmt_encoder_handle_t decel_encoder;
} stepper_motor_t;

typedef struct {
    float blade_width;
    float blade_lenght;
    int limit_switch;
    int tapper_gpio;
    uint32_t tap_duration;
    uint32_t recording_duration;
    bool direction;
} taptest_side_config;

typedef enum {
    STATE_IDLE,
    STATE_CARRIER_HOME,
    STATE_TAPBOT_RESET,
    STATE_TAPTEST_SEQUENCE,
    STATE_DONE
} state_t;

void setup_gpio_input(int gpio_num, bool pull_up, bool pull_down);
void setup_gpio_output(int gpio_num);
void stepper_motor_init(stepper_motor_t *motor, int gpio_en, int gpio_dir, int gpio_step,
                        int start_freq_hz, int end_freq_hz, int accel_points,
                        int decel_points, int uniform_speed_hz);
void carrier_home(stepper_motor_t *motor, uint32_t *uniform_speed_hz, gpio_num_t limit_gpio);
void tap_sequence(stepper_motor_t *motor, uint32_t *uniform_speed_hz, const taptest_side_config *cfg);
void IRAM_ATTR stop_button_isr_handler(void *arg);

#endif // MOTOR_OPS_H
