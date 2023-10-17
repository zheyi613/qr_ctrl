/**
 * @file icm20948.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief icm20948 driver
 * @date 2023-03-15
 */

#ifndef ICM20948_H
#define ICM20948_H

#include "stdint.h"

#define ICM20948_AD0    0
#define ICM20948_ADDR   (0b1101000 | ICM20948_AD0)

#define ACCEL_CALIBRATION

#define ACCEL_CALIBRATION_BIAS
#define ACCEL_X_BIAS    -0.0115F
#define ACCEL_Y_BIAS    0.F
#define ACCEL_Z_BIAS    -0.0075F
#define ACCEL_CALIBRATION_SCALE
#define ACCEL_X_SCALE   0.9985F
#define ACCEL_Y_SCALE   1.F
#define ACCEL_Z_SCALE   0.9926F

#define MAG_I2C_BYPASS

/* 
 * When data rate is higher than read data frequency,
 * the read data function will get multiple data from FIFO
 * and use average filter.
 * Note that the frequency of calling read data function
 * must not lower than data rate / 200.
 * 4096 bytes(buffer size) / 12(6-axis data) = 341
 * ex: data rate: 200, read freq > 1 Hz
 */
// #define ICM20948_FIFO_EN

enum gyro_fs {
        GYRO_250_DPS,
        GYRO_500_DPS,
        GYRO_1000_DPS,
        GYRO_2000_DPS
};

enum accel_fs {
        ACCEL_2G,
        ACCEL_4G,
        ACCEL_8G,
        ACCEL_16G
};

/* Gyro low pass freqency */
enum gyro_low_pass {
        GYRO_LP_196HZ,
        GYRO_LP_151HZ,
        GYRO_LP_119HZ,
        GYRO_LP_51HZ,
        GYRO_LP_23HZ,
        GYRO_LP_11HZ,
        GYRO_LP_5HZ,
        GYRO_LP_361HZ
};

/* Accel low pass frequency */
enum accel_low_pass {
        ACCEL_LP_246HZ = 1,
        ACCEL_LP_111HZ,
        ACCEL_LP_50HZ,
        ACCEL_LP_23HZ,
        ACCEL_LP_11HZ,
        ACCEL_LP_5HZ,
        ACCEL_LP_473HZ
};


int icm20948_init(uint16_t rate, enum gyro_fs g_fs, enum accel_fs a_fs,
                  enum gyro_low_pass gyro_lp, enum accel_low_pass accel_lp);
int icm20948_read_axis6(float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz);
#endif /* ICM20948_H */