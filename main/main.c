#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h" 
#include "stepper_motor_encoder.h"
//Define GPIOs
#define STEP_MOTOR_GPIO_EN       6
#define STEP_MOTOR_GPIO_DIR      5
#define STEP_MOTOR_GPIO_STEP     4

//Sensor GPIO inputs
#define TOP_END_LIMIT_GPIO       7 // GPIO for end limit switch

// Push button GPIOs
#define STOP_PB_GPIO             8 // GPIO for stop push button
#define START_PB_GPIO            9 // GPIO for start push button
#define CARRIER_RESET_PB_GPIO    10 // GPIO for carrier home push button
#define TAPBOT_RESET_PB_GPIO    11 // GPIO for tapbot home push button  

// Define stepper motor parameters
#define STEP_MOTOR_ENABLE_LEVEL  0 // TMC2209 is enabled on low level(make sure to snip off Diag and one next to it)
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000  // 1 MHz RMT resolution

typedef struct {
    int gpio_en;
    int gpio_dir;
    int gpio_step;
    rmt_channel_handle_t rmt_chan;
    rmt_encoder_handle_t accel_encoder;
    rmt_encoder_handle_t uniform_encoder;
    rmt_encoder_handle_t decel_encoder;
} stepper_motor_t;

void setup_gpio_input(int gpio_num) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}



void stepper_motor_init(
    stepper_motor_t *motor,
    int gpio_en, int gpio_dir, int gpio_step,
    int start_freq_hz, int end_freq_hz,
    int accel_points, int decel_points, int uniform_speed_hz
) {
    // Store pin numbers in struct
    motor->gpio_en   = gpio_en;
    motor->gpio_dir  = gpio_dir;
    motor->gpio_step = gpio_step;

    // 1. Configure EN and DIR GPIOs
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << gpio_en) | (1ULL << gpio_dir),
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    // 2. Create RMT TX channel for STEP pin

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_step,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor->rmt_chan));

    // 3. Create encoders
    // a) Acceleration phase encoder
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution     = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points  = accel_points,
        .start_freq_hz  = start_freq_hz,
        .end_freq_hz    = end_freq_hz,
    };

    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &motor->accel_encoder));

    // b) Uniform speed encoder
    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &motor->uniform_encoder));

    // c) Deceleration phase encoder
    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution     = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points  = decel_points,
        .start_freq_hz  = end_freq_hz,
        .end_freq_hz    = start_freq_hz,
    };

    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &motor->decel_encoder));

    // 4. Enable RMT channel
    ESP_ERROR_CHECK(rmt_enable(motor->rmt_chan));
}

void carrier_home(stepper_motor_t *motor, uint32_t *accel_steps,
                  uint32_t *uniform_speed_hz, uint32_t *decel_steps,
                  gpio_num_t limit_gpio) {

    rmt_transmit_config_t tx_config = {
    .loop_count = 0,
    .flags = {
        .eot_level = 0
    }
    };


    gpio_set_level(motor->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    gpio_set_level(motor->gpio_en, STEP_MOTOR_ENABLE_LEVEL);

    ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->accel_encoder, accel_steps, sizeof(uint32_t), &tx_config));

    while (gpio_get_level(limit_gpio) != 1) {
        tx_config.loop_count = 1000;
        ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder, uniform_speed_hz, sizeof(uint32_t), &tx_config));
        vTaskDelay(1); // Prevent CPU starvation
    }

    tx_config.loop_count = 0;
    ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->decel_encoder, decel_steps, sizeof(uint32_t), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));
    gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
}

void app_main(void) {
    stepper_motor_t motor1;
    stepper_motor_init(&motor1, 6, 5, 4, 500, 1500, 500, 500, 1500);
    setup_gpio_input(TOP_END_LIMIT_GPIO);

    uint32_t accel_steps = 10;
    uint32_t uniform_speed_hz = 1500;
    uint32_t decel_steps = 10;

    while (1) {
        if (gpio_get_level(TOP_END_LIMIT_GPIO) == 0) {
            carrier_home(&motor1, &accel_steps, &uniform_speed_hz, &decel_steps, TOP_END_LIMIT_GPIO);
            break;
        }
        vTaskDelay(10); // check periodically
    }
}

