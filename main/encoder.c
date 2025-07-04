#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "config.h"

#define TAG "ENCODER"


static volatile int encoder_position = 0;
static gpio_num_t encoder_pin_a;
static gpio_num_t encoder_pin_b;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR encoder_isr_handler(void *arg) {
    int level_a = gpio_get_level(encoder_pin_a);
    int level_b = gpio_get_level(encoder_pin_b);

    portENTER_CRITICAL_ISR(&mux);
    if (level_a == level_b) {
        encoder_position++;
    } else {
        encoder_position--;
    }
    portEXIT_CRITICAL_ISR(&mux);
}

void encoder_init(gpio_num_t pin_a, gpio_num_t pin_b) {
    encoder_pin_a = pin_a;
    encoder_pin_b = pin_b;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin_a),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << pin_b);
    gpio_config(&io_conf);

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service");
    }

    gpio_isr_handler_add(pin_a, encoder_isr_handler, NULL);

    ESP_LOGI(TAG, "Encoder initialized on GPIO %d and %d", pin_a, pin_b);
}

int encoder_get_position(void) {
    int pos;
    portENTER_CRITICAL(&mux);
    pos = encoder_position;
    portEXIT_CRITICAL(&mux);
    return pos;
}

void encoder_reset_position(void) {
    portENTER_CRITICAL(&mux);
    encoder_position = 0;
    portEXIT_CRITICAL(&mux);
}

float encoder_get_distance_mm(void) {
    float circumference = PI * WHEEL_DIAMETER_MM;
    int position = encoder_get_position();
    return ((float)position / COUNTS_PER_REV) * circumference;
}
    
