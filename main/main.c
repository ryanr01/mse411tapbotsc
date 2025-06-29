#include "freertos/FreeRTOS.h"
#include <math.h>
#include <stdbool.h>
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "recordSample.h"
#include "motor_ops.h"
#include "pin_config.h"


void app_main(void) {
    stepper_motor_t motor1;
    stepper_motor_init(&motor1,
                       STEP_MOTOR_GPIO_EN,
                       STEP_MOTOR_GPIO_DIR,
                       STEP_MOTOR_GPIO_STEP,
                       500,
                       1500,
                       500,
                       500,
                       1500);
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
                ESP_LOGI("StepperMotor", "Resetting tapbot...");
                carrier_home(&motor1, &uniform_speed_hz, TOP_END_LIMIT_GPIO);
                
                ESP_LOGI("StepperMotor", "Resetting tapbot...");
                state = STATE_IDLE;
                break;

            case STATE_TAPTEST_SEQUENCE:
                ESP_LOGI("Tap Test", "Homing carrier...");
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

