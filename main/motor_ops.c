#include "motor_ops.h"
#include "esp_log.h"
#include "driver/rmt_common.h"

static volatile bool stop_requested = false;

void IRAM_ATTR stop_button_isr_handler(void *arg)
{
    stop_requested = true;
}

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

void stepper_motor_init(stepper_motor_t *motor, int gpio_en, int gpio_dir, int gpio_step,
                        int start_freq_hz, int end_freq_hz, int accel_points, int decel_points,
                        int uniform_speed_hz) {
    motor->gpio_en   = gpio_en;
    motor->gpio_dir  = gpio_dir;
    motor->gpio_step = gpio_step;

    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << gpio_en) | (1ULL << gpio_dir),
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_step,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor->rmt_chan));

    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution     = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points  = accel_points,
        .start_freq_hz  = start_freq_hz,
        .end_freq_hz    = end_freq_hz,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &motor->accel_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &motor->uniform_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution     = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points  = decel_points,
        .start_freq_hz  = end_freq_hz,
        .end_freq_hz    = start_freq_hz,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &motor->decel_encoder));

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
    tx_config.loop_count = 1000000;
    ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder, uniform_speed_hz,
                                sizeof(uint32_t), &tx_config));

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
        gpio_set_level(motor -> gpio_dir,
                       direction ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);

        for (int i = 0; i < (cfg->blade_width / 10) && !stop_requested; i++) {
            if( (gpio_get_level(cfg->limit_switch) == 1) && (i>1&&i<0.9*cfg->blade_lenght/10)) {
                ESP_LOGI("StepperMotor", "End limit switch triggered, stopping tap sequence.");
                gpio_set_level(motor->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
                break;
            }
            uint32_t n_steps = 1;
            tx_config.loop_count = 8000; // is 8 microsteps, 200 steps per revolution, 2mm pitch, and 10mm distance
            ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder,
                                        uniform_speed_hz, n_steps * sizeof(uint32_t), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));


            vTaskDelay(pdMS_TO_TICKS(200));
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
