#ifndef _VL53L0X_I2C_PLATFORM_H_
#define _VL53L0X_I2C_PLATFORM_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

VL53L0X_Error VL53L0X_write_multi(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
VL53L0X_Error VL53L0X_read_multi(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
VL53L0X_Error VL53L0X_write_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_write_word(VL53L0X_DEV Dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_write_dword(VL53L0X_DEV Dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_read_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata);
VL53L0X_Error VL53L0X_read_word(VL53L0X_DEV Dev, uint8_t index, uint16_t *pdata);
VL53L0X_Error VL53L0X_read_dword(VL53L0X_DEV Dev, uint8_t index, uint32_t *pdata);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L0X_I2C_PLATFORM_H_ */