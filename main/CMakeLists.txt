idf_component_register(SRCS "main.c" "motor_ops.c" "stepperexample.c" "stepper_motor_encoder.c"
                       PRIV_REQUIRES esp_driver_rmt esp_driver_gpio
                       INCLUDE_DIRS ".")