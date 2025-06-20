#include "freertos/FreeRTOS.h"
#include <math.h>
#include <stdbool.h>
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_common.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "stepper_motor_encoder.h"

static volatile bool stop_requested = false;

static void IRAM_ATTR stop_button_isr_handler(void *arg)
{
    stop_requested = true;
}
//Define GPIOs
//Motor Top GPIOs
#define STEP_MOTOR_GPIO_EN       6
#define STEP_MOTOR_GPIO_DIR      5
#define STEP_MOTOR_GPIO_STEP     4

// GPIOs for other outputs
#define TAPBOT_TAPPER_GPIO      13 // GPIO for tapbot tapper

//Sensor GPIO inputs
#define TOP_END_LIMIT_GPIO       7 // GPIO for end limit switch

// Push button GPIOs
#define STOP_PB_GPIO            12 // GPIO for stop push button
#define START_PB_GPIO            9 // GPIO for start push button
#define CARRIER_RESET_PB_GPIO   10 // GPIO for carrier home push button
#define TAPBOT_RESET_PB_GPIO    11 // GPIO for tapbot home push button  

// Define stepper motor parameters
#define STEP_MOTOR_ENABLE_LEVEL  0 // TMC2209 is enabled on low level(make sure to snip off Diag and one next to it)
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
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

void setup_gpio_input(int gpio_num, bool pull_up, bool pull_down) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pull_up,
        .pull_down_en = pull_down,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void setup_gpio_output(int gpio_num) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void stepper_motor_init(stepper_motor_t *motor, int gpio_en, int gpio_dir, int gpio_step, int start_freq_hz, int end_freq_hz, int accel_points, int decel_points, int uniform_speed_hz) {
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

void carrier_home(stepper_motor_t *motor, uint32_t *uniform_speed_hz, gpio_num_t limit_gpio) {
    
    if (gpio_get_level(limit_gpio) == 1) {
        ESP_LOGI("StepperMotor", "Limit switch already triggered, skipping homing.");
        return;
    }
    rmt_transmit_config_t tx_config = {
    .loop_count = 0,
    .flags = {
        .eot_level = 0
    }
    };


    gpio_set_level(motor->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    gpio_set_level(motor->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    tx_config.loop_count = 1000000; // Set a high loop count for acceleration phase
    ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder, uniform_speed_hz, sizeof(uint32_t), &tx_config));

    while (gpio_get_level(limit_gpio) != 1) {
        if (stop_requested) {
            rmt_disable(motor->rmt_chan);
            rmt_enable(motor->rmt_chan);
            gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
            stop_requested = false;
            return;
        }
        vTaskDelay(1);
    }

    rmt_disable(motor->rmt_chan);
    rmt_enable(motor->rmt_chan);
    ESP_LOGI("StepperMotor", "end limit reached.");
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));
    gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
}

void tap_sequence(stepper_motor_t *motor, uint32_t *uniform_speed_hz, const taptest_side_config *cfg) {

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0
        }
    };
    gpio_set_level(motor->gpio_en, STEP_MOTOR_ENABLE_LEVEL);

    bool direction = !cfg->direction;
    for (int j = 0; j < 5 && !stop_requested; j++) {
        gpio_set_level(motor->gpio_dir,
                       direction ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);

        for (int i = 0; i < (cfg->blade_width / 10) && !stop_requested; i++) {
            //add audio recording here
            if( (gpio_get_level(cfg->limit_switch) == 1) &&(i>1&&i<0.9*cfg->blade_lenght/10)) {
                ESP_LOGI("StepperMotor", "End limit switch triggered, stopping tap sequence.");
                gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
                break;
            }
            uint32_t n_steps = 1;
            tx_config.loop_count = 10000;
            ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder,
                                    uniform_speed_hz, n_steps * sizeof(uint32_t), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));
            // Put tap call here 
            vTaskDelay(pdMS_TO_TICKS(200)); // Delay to allow motor to move
            gpio_set_level(cfg->tapper_gpio, 1);
            vTaskDelay(pdMS_TO_TICKS(cfg->tap_duration));
            gpio_set_level(cfg->tapper_gpio, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI("StepperMotor", "Cord Position Y = %dmm", direction ? i * 10 : (int)(cfg->blade_width - i * 10));
            if (stop_requested) {
                rmt_disable(motor->rmt_chan);
                rmt_enable(motor->rmt_chan);
                gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
                stop_requested = false;
                return;
            }
        }
        // set direction
        direction = !direction;
        
        
        vTaskDelay(pdMS_TO_TICKS(cfg->recording_duration));
        if (stop_requested) {
            rmt_disable(motor->rmt_chan);
            gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
            stop_requested = false;
            return;
        }
    }
    gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
}


void app_main(void) {
    stepper_motor_t motor1;
    stepper_motor_init(&motor1, 6, 5, 4, 500, 1500, 500, 500, 1500);
    setup_gpio_input(TOP_END_LIMIT_GPIO, false, true);
    setup_gpio_input(STOP_PB_GPIO, false, true);
    setup_gpio_input(START_PB_GPIO, false, true);
    setup_gpio_input(TAPBOT_RESET_PB_GPIO, false, true);
    setup_gpio_input(CARRIER_RESET_PB_GPIO, false, true);

    gpio_set_intr_type(STOP_PB_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(STOP_PB_GPIO, stop_button_isr_handler, NULL);

    uint32_t uniform_speed_hz = 5000;

    taptest_side_config side_cfg = {100, 1000, TOP_END_LIMIT_GPIO, TAPBOT_RESET_PB_GPIO, 100, 1000, 1};

    state_t state = STATE_IDLE;

    while (1) {
        switch (state) {
            case STATE_IDLE:
                if (gpio_get_level(START_PB_GPIO)==1) state = STATE_TAPTEST_SEQUENCE;
                else if (gpio_get_level(CARRIER_RESET_PB_GPIO)==1) state = STATE_CARRIER_HOME;
                else if (gpio_get_level(TAPBOT_RESET_PB_GPIO)==1) state = STATE_TAPBOT_RESET;
                ESP_LOGI("StepperMotor", "Waiting for action...");
                break;

            case STATE_CARRIER_HOME:
                ESP_LOGI("StepperMotor", "Homing carrier...");
                if (gpio_get_level(TOP_END_LIMIT_GPIO) == 0) {
                    esp_log_level_set("*", ESP_LOG_INFO);
                    carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                }
                ESP_LOGI("StepperMotor", "Carrier homed.");
                state = STATE_IDLE;
                break;
                

            case STATE_TAPBOT_RESET:
                carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                
                ESP_LOGI("StepperMotor", "Resetting tapbot...");
                state = STATE_IDLE;
                break;

            case STATE_TAPTEST_SEQUENCE:
                carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                ESP_LOGI("Tap Test", "Starting tap sequence...");
                tap_sequence(&motor1, &uniform_speed_hz, &side_cfg);
                state = STATE_IDLE;
                break;

            case STATE_DONE:
                
                ESP_LOGI("Tap Test done", "Resetting tapbot...");
                break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

