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


void app_main(void) {
    stepper_motor_t motorbot;
    stepper_motor_init(&motorbot,
                       BOT_STEP_MOTOR_GPIO_DIR,
                       BOT_STEP_MOTOR_GPIO_STEP,
                       BOT_END_LIMIT_GPIO,
                       IN3,
                       STEP_MOTOR_SPIN_DIR_CLOCKWISE,
                       500,
                       1500,
                       500,
                       500,
                       1500);

    stepper_motor_t motortop;
    stepper_motor_init(&motortop,
                       TOP_STEP_MOTOR_GPIO_DIR,
                       TOP_STEP_MOTOR_GPIO_STEP,
                       TOP_END_LIMIT_GPIO,
                       IN4,
                       STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE,
                       500,
                       1500,
                       500,
                       500,
                       1500);
    setup_gpio_input(TOP_END_LIMIT_GPIO, false, true);                  
    setup_gpio_input(BOT_END_LIMIT_GPIO, false, true);
    setup_gpio_input(STOP_PB_GPIO, true, false);
    setup_gpio_input(START_PB_GPIO, true, false);
    setup_gpio_input(TAPBOT_RESET_PB_GPIO, true, false);
    setup_gpio_output(ENB);
    setup_gpio_output(IN3);
    setup_gpio_output(IN4);
    gpio_set_level(ENB, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);

    gpio_set_intr_type(STOP_PB_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(STOP_PB_GPIO, stop_button_isr_handler, NULL);

    uint32_t uniform_speed_hz = 5000;

    taptest_blade_config side_cfg = {160, 340, 50, 1000};

    state_t state = STATE_IDLE;

    while (1) {
        switch (state) {
            case STATE_IDLE:
                if (gpio_get_level(START_PB_GPIO)==0) state = STATE_TAPTEST_SEQUENCE;
                else if (gpio_get_level(TAPBOT_RESET_PB_GPIO)==0) state = STATE_TAPBOT_RESET;
                ESP_LOGI("TapBot", "Waiting for action...");
                break;
                

            case STATE_TAPBOT_RESET:
                ESP_LOGI("TapBot", "Resetting tapbot...");
                carrier_home(&motorbot,&motortop, &uniform_speed_hz);

                ESP_LOGI("TapBot", "Resetting tapbot...");
                state = STATE_IDLE;
                break;

            case STATE_TAPTEST_SEQUENCE:
                ESP_LOGI("Tap Test", "Homing carrier...");
                carrier_home(&motorbot,&motortop, &uniform_speed_hz);
                ESP_LOGI("Tap Test", "Starting tap sequence...");
                tap_sequence(&motorbot, &uniform_speed_hz, &side_cfg);
                state = STATE_IDLE;
                break;

            case STATE_DONE:
                
                ESP_LOGI("Tap Test done", "Resetting tapbot...");
                break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

