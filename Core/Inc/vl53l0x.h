/**
 * @file vl53l0x.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief simplify function of VL53L0X api (STSW-IMG005)
 * @date 2023-05-15
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

#define VL53L0X_ADDR    0x29

enum VL53L0X_MODE {
        VL53L0X_SENSE_DEFAULT,
        VL53L0X_SENSE_LONG_RANGE,
        VL53L0X_SENSE_HIGH_SPEED,
        VL53L0X_SENSE_HIGH_ACCURACY,
};

int vl53l0x_init(enum VL53L0X_MODE mode, uint16_t period_ms);
int vl53l0x_is_range_ready(void);
int vl53l0x_get_distance(uint16_t *dist);

#endif