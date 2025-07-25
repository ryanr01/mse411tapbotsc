#include "lidar.h"

// To use this function, call lidar_init(), then read_from_lider() to return the reading in mm
// Ensure that the lidar has adequate time to send, receive, and process signal over I2C with a 100 ms or so task delay

static const char *LIDAR_TAG = "LIDAR";

static void i2c_master_init(i2c_master_bus_handle_t *bus, i2c_master_dev_handle_t *i2c_dev)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &dev_config, i2c_dev));
}

void lidar_init(void){
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_init(&bus, &i2c_dev);
    VL53L0X_Dev_t api_dev;
    api_dev.I2cDevAddr = VL53L0X_I2C_ADDR;
    // To fix the error, add this line to VL53L0X_Dev_t in vl53l0x_platform.h after the comment '/* user specific field */':
    // i2c_master_dev_handle_t i2c_handle;
    api_dev.i2c_handle = i2c_dev;
    VL53L0X_DEV dev = &api_dev;
    ESP_ERROR_CHECK(VL53L0X_DataInit(dev));
    ESP_ERROR_CHECK(VL53L0X_StaticInit(dev));
    // Perform reference SPAD management
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    ESP_ERROR_CHECK(VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads));
    // Perform reference calibration
    uint8_t vhvSettings;
    uint8_t phaseCal;
    ESP_ERROR_CHECK(VL53L0X_PerformRefCalibration(dev, &vhvSettings, &phaseCal));
    // Set to long range mode (optional)
    VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 33000);
    VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    // Set device mode
    VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    // Start continuous measurement
    VL53L0X_StartMeasurement(dev);
}

float read_from_lidar(void) {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_init(&bus, &i2c_dev);
    VL53L0X_Dev_t api_dev;
    api_dev.I2cDevAddr = VL53L0X_I2C_ADDR;
    // To fix the error, add this line to VL53L0X_Dev_t in vl53l0x_platform.h after the comment '/* user specific field */':
    // i2c_master_dev_handle_t i2c_handle;
    api_dev.i2c_handle = i2c_dev;
    VL53L0X_DEV dev = &api_dev;
    ESP_ERROR_CHECK(VL53L0X_DataInit(dev));
    ESP_ERROR_CHECK(VL53L0X_StaticInit(dev));
    uint8_t dataReady = 0;
    VL53L0X_GetMeasurementDataReady(dev, &dataReady);
    VL53L0X_RangingMeasurementData_t measurement;
    if (dataReady) {
        VL53L0X_GetRangingMeasurementData(dev, &measurement);
        if (measurement.RangeStatus == 0) {
            ESP_LOGI(LIDAR_TAG, "Distance: %d mm", measurement.RangeMilliMeter);
        } else {
            ESP_LOGI(LIDAR_TAG, "No Lidar Data");
        }
        VL53L0X_ClearInterruptMask(dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    }
    // vTaskDelay(pdMS_TO_TICKS(100)); // Add this task delay somewhere else to ensure lidar has time to read
    return measurement.RangeMilliMeter;
}