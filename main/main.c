#include "freertos/FreeRTOS.h"
#include <math.h>
#include <stdbool.h>
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "motor_driver.h"
#include "encoder.h"
#include "motor_ops.h"
#include "pin_config.h"
#include "lidar.h"


void app_main(void) {
    stepper_motor_t motor1;
    stepper_motor_init(&motor1,
                       STEP_MOTOR_GPIO_DIR,
                       STEP_MOTOR_GPIO_STEP,
                       500,
                       1500,
                       500,
                       500,
                       1500);


                           stepper_motor_t steppermotorbottom;
    stepper_motor_init(&steppermotorbottom,
                       STEP_MOTOR_GPIO_DIR_BOTTOM,   
                       STEP_MOTOR_GPIO_STEP_BOTTOM,   
                       500,
                       1500,
                       500,
                       500,
                       1500);
    setup_gpio_input(TOP_END_LIMIT_GPIO, false, true);
    setup_gpio_input(STOP_PB_GPIO, true, false);
    setup_gpio_input(START_PB_GPIO, true, false);
    setup_gpio_input(TAPBOT_RESET_PB_GPIO, true, false);
    setup_gpio_input(CARRIER_RESET_PB_GPIO, true, false);
    setup_gpio_output(ENB);
    setup_gpio_output(IN3);
    setup_gpio_output(IN4);
    gpio_set_level(ENB, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);

    gpio_set_intr_type(STOP_PB_GPIO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(STOP_PB_GPIO, stop_button_isr_handler, NULL);

    uint32_t uniform_speed_hz = 5000;

    taptest_side_config side_cfg = {160, 340, TOP_END_LIMIT_GPIO, IN3, 50, 1000, 1};

    taptest_side_config side_cfg_bottom = {160, 340, BOTTOM_END_LIMIT_GPIO, IN5, 50, 1000, 1}; 
    


    state_t state = STATE_IDLE;

    while (1) {
        switch (state) {
            case STATE_IDLE:
                if (gpio_get_level(START_PB_GPIO)==1) state = STATE_TAPTEST_SEQUENCE;
                else if (gpio_get_level(CARRIER_RESET_PB_GPIO)==1) state = STATE_CARRIER_HOME;
                else if (gpio_get_level(TAPBOT_RESET_PB_GPIO)==1) state = STATE_TAPBOT_RESET;
                ESP_LOGI("TapBot", "Waiting for action...");
                break;

            case STATE_CARRIER_HOME:
                ESP_LOGI("TapBot", "Homing carrier...");
                if (gpio_get_level(TOP_END_LIMIT_GPIO) == 0) {
                    esp_log_level_set("*", ESP_LOG_INFO);
                    carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                }
                ESP_LOGI("TapBot", "Carrier homed.");
                state = STATE_IDLE;
                break;
                

            case STATE_TAPBOT_RESET:
                ESP_LOGI("TapBot", "Resetting tapbot...");
                carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                
                ESP_LOGI("TapBot", "Resetting tapbot...");
                state = STATE_IDLE;
                break;

            case STATE_TAPTEST_SEQUENCE:
                ESP_LOGI("Tap Test", "Homing carrier...");
                carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                ESP_LOGI("Tap Test", "Starting tap sequence...");
                tap_sequence_dual(&motor1, &steppermotorbottom, &uniform_speed_hz, &uniform_speed_hz, &side_cfg, &side_cfg_bottom);
                state = STATE_IDLE;
                break;

            case STATE_DONE:
                
                ESP_LOGI("Tap Test done", "Resetting tapbot...");
                break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

