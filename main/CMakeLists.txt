idf_component_register(
    SRCS "main.c" "motor_ops.c" "stepper_motor_encoder.c" "recordSample.c"
    PRIV_REQUIRES esp_driver_rmt esp_driver_gpio fatfs sdmmc
    INCLUDE_DIRS "."
)
