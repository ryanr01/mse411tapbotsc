idf_component_register(
    SRCS
        "main.c"
        "motor_ops.c"
        "stepper_motor_encoder.c"
        "motor_driver.c"
        "encoder.c"
        "recordSample.c"
        "lidar.c"
        INCLUDE_DIRS "."
        REQUIRES "vl53l0x"
    PRIV_REQUIRES
        esp_driver_rmt
        esp_driver_gpio
        esp_driver_i2s
        fatfs
        sdmmc
        esp_timer
    INCLUDE_DIRS
        "."
)
