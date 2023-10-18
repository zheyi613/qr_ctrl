#ifndef MPU9250_H
#define MPU9250_H

#include "stdint.h"

#define ACCEL_CALIBRATION
#define ACCEL_CALIBRATION_PARAM

#ifdef ACCEL_CALIBRATION_PARAM
#define ACCEL_X_BIAS  -0.07F
#define ACCEL_Y_BIAS  -0.026F
#define ACCEL_Z_BIAS  -0.01F
#define ACCEL_X_SCALE 1.F
#define ACCEL_Y_SCALE 0.999F
#define ACCEL_Z_SCALE 0.98F
#endif

#define MAG_X_BIAS    -6.5116F
#define MAG_Y_BIAS    20.9513F
#define MAG_Z_BIAS    -43.9649F 

#define MPU9250_AD0   0

enum accel_fs {
  ACCEL_FS_2G = 0,
  ACCEL_FS_4G,
  ACCEL_FS_8G,
  ACCEL_FS_16G
};

enum gyro_fs {
  GYRO_FS_250DPS = 0,
  GYRO_FS_500DPS,
  GYRO_FS_1000DPS,
  GYRO_FS_2000DPS
};

enum accel_lp {
  ACCEL_LP_250HZ = 0,
  ACCEL_LP_184HZ,
  ACCEL_LP_92HZ,
  ACCEL_LP_41HZ,
  ACCEL_LP_20HZ,
  ACCEL_LP_10HZ,
  ACCEL_LP_5HZ,
};

enum gyro_lp {
  GYRO_LP_218HZ = 1,
  GYRO_LP_99HZ,
  GYRO_LP_44HZ,
  GYRO_LP_21HZ,
  GYRO_LP_10HZ,
  GYRO_LP_5HZ,
};

int mpu9250_init(uint16_t rate, enum gyro_fs g_fs, enum accel_fs a_fs,
                 enum gyro_lp g_lp, enum accel_lp a_lp);
int mpu9250_read_imu(float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz);
int mpu9250_read_mag(float *mx, float *my, float *mz);

#endif