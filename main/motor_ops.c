#include "motor_ops.h"
#include "motor_driver.h"
#include "encoder.h"
#include "esp_log.h"
#include "driver/rmt_common.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "pin_config.h"
#include "recordSample.h"
#include "stepper_motor_encoder.h"


static volatile bool stop_requested = false;

void IRAM_ATTR stop_button_isr_handler(void *arg){
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

void stepper_motor_init(stepper_motor_t *motor, int gpio_dir, int gpio_step,
                        int start_freq_hz, int end_freq_hz,
                        int accel_points, int decel_points,
                        int uniform_speed_hz)
{
    motor->gpio_dir  = gpio_dir;
    motor->gpio_step = gpio_step;

    gpio_config_t en_dir_gpio_config = {
        .pin_bit_mask = 1ULL << gpio_dir,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
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

bool carrier_home(stepper_motor_t *motor, uint32_t *uniform_speed_hz, const taptest_side_config *side_cfg) {
    if (gpio_get_level(side_cfg->limit_switch) == 1) {
        ESP_LOGI("StepperMotor", "Limit switch already triggered, skipping homing.");
        return true;
    }
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0
        }
    };

    gpio_set_level(motor->gpio_dir, side_cfg->direction);

    tx_config.loop_count = 1000000;
    ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder, uniform_speed_hz,
                                sizeof(uint32_t), &tx_config));

    while (gpio_get_level(side_cfg->limit_switch) != 1) {
        if (stop_requested) {
            rmt_disable(motor->rmt_chan);
            rmt_enable(motor->rmt_chan);

            stop_requested = false;
            return false;
        }
        vTaskDelay(1);
    }

    rmt_disable(motor->rmt_chan);
    rmt_enable(motor->rmt_chan);
    ESP_LOGI("StepperMotor", "end limit reached.");
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));

    return true;
}

void tap_sequence(stepper_motor_t *motor, uint32_t *uniform_speed_hz, const taptest_side_config *cfg) {
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0
        }
    };

    bool direction = !cfg->direction;
    for (int j = 0; j < 5 && !stop_requested; j++) {
        gpio_set_level(motor->gpio_dir,
                       direction);
// improve tap logic and add a proper emergency stop also make the home calibrate the total width of blade
        uint32_t n_steps = 1;
        tx_config.loop_count = 4000;
        ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder,
                                    uniform_speed_hz, n_steps * sizeof(uint32_t), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));

        for (int i = 0; i < (cfg->blade_width / 10) && !stop_requested; i++) {
            if( (gpio_get_level(cfg->limit_switch) == 1) &&(i>1&&i<0.9*cfg->blade_lenght/10)) {
                ESP_LOGI("StepperMotor", "End limit switch triggered, stopping tap sequence.");

            
                break;


                
            }
            uint32_t n_steps = 1;
            tx_config.loop_count = 4000;
            ESP_ERROR_CHECK(rmt_transmit(motor->rmt_chan, motor->uniform_encoder,
                                        uniform_speed_hz, n_steps * sizeof(uint32_t), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor->rmt_chan, -1));

            //vTaskDelay(pdMS_TO_TICKS(200));
            //gpio_set_level(cfg->tapper_gpio, 1);
            //vTaskDelay(pdMS_TO_TICKS(cfg->tap_duration));
            //gpio_set_level(cfg->tapper_gpio, 0);
            float x_coord = (float)j; // Taken as an incremented index for now
            float y_coord = (float)(direction ? i * 10 : (int)(cfg->blade_width - i * 10));
            record_sample(100, "T", 1.1, 2.3); // Call record_sample, swap the placeholder values for actual coordinates to save where the data was taken as part of the filename
            //vTaskDelay(pdMS_TO_TICKS(100));
            //Record colour

            if (stop_requested) {
                rmt_disable(motor->rmt_chan);
                rmt_enable(motor->rmt_chan);
                stop_requested = false;
                return;
            }




                //Encoder and DC Driver movement
                encoder_init(ENCODER_PIN_A, ENCODER_PIN_B);
                motor_driver_init();
                //Drive the motor to move 10 mm
                float target_distance_mm = 10.0f;
                //Direction, Forward is true, Reverse is false direction
                  bool spandir = true;
                    DCmotordrive(target_distance_mm, spandir);
                    //Print final distance
                    float final_distance = encoder_get_distance_mm();
                    int final_position = encoder_get_position();

                    ESP_LOGI("MAIN", "Target: %.2f mm", target_distance_mm);
                     ESP_LOGI("MAIN", "Final Position: %d counts", final_position);
                    ESP_LOGI("MAIN", "Final Distance: %.2f mm", final_distance);


        direction = !direction;

        //vTaskDelay(pdMS_TO_TICKS(cfg->recording_duration));
        if (stop_requested) {
            rmt_disable(motor->rmt_chan);

            stop_requested = false;
            return;
        }
    }
}
}
