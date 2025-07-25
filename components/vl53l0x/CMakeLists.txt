idf_component_register(SRCS "vl53l0x.c"
                            "core/src/vl53l0x_api.c"
                            "core/src/vl53l0x_api_core.c"
                            "core/src/vl53l0x_api_calibration.c"
                            "core/src/vl53l0x_api_strings.c"
                            "core/src/vl53l0x_api_ranging.c"  # Add any other .c files from the API
                            "platform/src/vl53l0x_platform.c"  # Your platform impl
                            "platform/src/vl53l0x_i2c_platform.c"
                       INCLUDE_DIRS "include" "core/inc" "platform/inc"
                       REQUIRES "driver" "freertos" "nvs_flash")
