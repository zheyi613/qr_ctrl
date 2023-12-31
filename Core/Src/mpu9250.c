/**
 * @file mpu9250.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief a portable driver for mpu9250
 * @date 2023-10-16
 */
#include "mpu9250.h"
#include "rtos_bus.h"
#include "stm32f4xx_hal.h"

#define read_reg_multi(addr, reg, pData, size)               \
        i2c_read_multi_dma(&hi2c2, addr, reg, I2C_REG_8BIT,  \
                           pData, size)
#define write_reg_multi(addr, reg, pData, size)              \
        i2c_write_multi_dma(&hi2c2, addr, reg, I2C_REG_8BIT, \
                            pData, size)
#define delay_ms(val)   HAL_Delay(val)

/* MPU9250 address is 110100 for ADO = 0 and 110101 for ADO = 1 */
#define MPU9250_ADDRESS  ((0x68 | MPU9250_AD0) << 1)

/* Magnetometer address */
#define AK8963_ADDRESS   (0x0C << 1)

/* Magnetometer registers */
#define AK8963_WHO_AM_I  0x00  /* should return 0x48 */
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  /* bit 0: data ready, bit 1: data overrun */
#define AK8963_HXL       0x03
#define AK8963_HXH       0x04
#define AK8963_HYL       0x05
#define AK8963_HYH       0x06
#define AK8963_HZL       0x07
#define AK8963_HZH       0x08
#define AK8963_ST2       0x09  /* bit 3: overflow, bit 4: output (0: 14bit, 1: 16bit) */
#define AK8963_CNTL      0x0A  /* See 8.3.6 in AK8963 */
#define AK8963_CNTL2     0x0B  /* Write bit 0 to 1 to reset all registers */
#define AK8963_ASTC      0x0C  /* Self test */
#define AK8963_I2CDIS    0x0F  /* I2C disable */
#define AK8963_ASAX      0x10  /* Fuse ROM x sensitivity adjustment value */
#define AK8963_ASAY      0x11  /* Fuse ROM y sensitivity adjustment value */
#define AK8963_ASAZ      0x12  /* Fuse ROM z sensitivity adjustment value */

/* IMU registers */
#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75  // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

static float accel_res;
static float gyro_res;
static float mag_adj[3];

static inline uint8_t read_reg(uint8_t addr, uint8_t reg)
{
        uint8_t val;

        read_reg_multi(addr, reg, &val, 1);

        return val;
}

static inline void write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
        write_reg_multi(addr, reg, &val, 1);
}

static void calibration(void)
{
        uint8_t data[12];
        uint16_t i, j, fifo_count, packet_count;
        int16_t tmp;
        int32_t bias[6] = {0}; /* accel[0:2], gyro[3:5] */

        /* Reset FIFO and DMP */
        write_reg(MPU9250_ADDRESS, USER_CTRL, 0x0C);

        /* Configure gyro and accel for bias calculation */
        write_reg(MPU9250_ADDRESS, CONFIG, 0x01); /* gyro low pass 188HZ */

        /* Enable FIFO */
        write_reg(MPU9250_ADDRESS, USER_CTRL, 0x40);
        /* Enable gyro and accel sensors for FIFO */
        write_reg(MPU9250_ADDRESS, FIFO_EN, 0x78);
        delay_ms(40); /* Wait 40 samples (1kHz) (480 bytes < 512 bytes) */

        /* Disable FIFO */
        write_reg(MPU9250_ADDRESS, FIFO_EN, 0x00);

        /* Read FIFO count */
        read_reg_multi(MPU9250_ADDRESS, FIFO_COUNTH, data, 2);
        fifo_count = ((uint16_t)data[0] << 8) | data[1];
        packet_count = fifo_count / 12;

        for (i = 0; i < packet_count; i++) {
                read_reg_multi(MPU9250_ADDRESS, FIFO_R_W, data, 12);

                for (j = 0; j < 6; j++) {
                        tmp = ((int16_t)data[j * 2] << 8) | data[(j * 2) + 1];
                        bias[j] += (int32_t)tmp;
                }
        }
        for (i = 0; i < 6; i++) {
                bias[i] /= (int32_t)packet_count;
        }
        bias[2] -= 16384;
#ifdef GYRO_CALIBRATION_PARAM
        bias[3] = (int32_t)(GYRO_X_BIAS * 131.F);
        bias[4] = (int32_t)(GYRO_Y_BIAS * 131.F);
        bias[5] = (int32_t)(GYRO_Z_BIAS * 131.F);
        for (i = 0; i < 3; i++) {
                data[2 * i] = ((-bias[3 + i]) >> 10) & 0xFF; /* 32.9 LSB deg/s */
                data[(2 * i) + 1] = ((-bias[3 + i]) >> 2) & 0xFF;
        }
#else
        for (i = 0; i < 3; i++) {
                data[2 * i] = ((-bias[3 + i]) >> 10) & 0xFF; /* 32.9 LSB deg/s */
                data[(2 * i) + 1] = ((-bias[3 + i]) >> 2) & 0xFF;
        }
#endif
        /* Write gyro offset to MPU9250 */
        write_reg_multi(MPU9250_ADDRESS, XG_OFFSET_H, data, 6);
#ifdef ACCEL_CALIBRATION_PARAM
        bias[0] = (int32_t)(ACCEL_X_BIAS * 16384.F);
        bias[1] = (int32_t)(ACCEL_Y_BIAS * 16384.F);
        bias[2] = (int32_t)(ACCEL_Z_BIAS * 16384.F);
#endif
#ifdef ACCEL_CALIBRATION
        /* Read accel offset from MPU9250 */
        uint8_t mask_bit;
        for (i = 0; i < 3; i++) {
                read_reg_multi(MPU9250_ADDRESS, XA_OFFSET_H + (3 * i), data, 2);
                tmp = ((int16_t)data[0] << 8) | data[1];
                mask_bit = tmp & 0x1UL;
                tmp -= (int16_t)(bias[i] >> 3); /* 2048 LSB/g */
                data[0] = (tmp >> 8) & 0xFF;
                data[1] = (tmp & 0xFE) | mask_bit;
                write_reg_multi(MPU9250_ADDRESS, XA_OFFSET_H + (3 * i), data, 2);
        }
#endif
}

/**
 * @brief Initialize MPU9250 and AK8963
 * 
 * @param rate sample rate
 * @param g_fs 250/500/100/2000 DPS
 * @param a_fs 2/4/8/16 G
 * @param g_lp see gyro_lp
 * @param a_lp see accel_lp
 * @return int 0: successful / 1: mpu9250 not found / 2: ak8963 not found
 */
int mpu9250_init(uint16_t rate, enum gyro_fs g_fs, enum accel_fs a_fs,
                 enum gyro_lp g_lp, enum accel_lp a_lp)
{
        uint8_t data[3];

        /* Initialize MPU9250 */

        /* Reset */
        delay_ms(200);
        write_reg(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
        delay_ms(100);
        
        /* Check who am I */
        if (read_reg(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 0x71)
                return 1;
        
        /* Wake up MPU9250 and configure clock source to PLL */
        write_reg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
        delay_ms(50);

        calibration();

        /* Configure gyro low pass filter (disable FIFO and ext sync) */
        write_reg(MPU9250_ADDRESS, CONFIG, g_lp);

        /* Configure sample rate */
        if (rate > 1000)
                rate = 1000;
        else if (rate < 5)
                rate = 5;
        write_reg(MPU9250_ADDRESS, SMPLRT_DIV, 1000 / rate - 1);

        /* Configure gyro full scale range and disable bypass low pass */
        write_reg(MPU9250_ADDRESS, GYRO_CONFIG, (g_fs << 3) | 0x03);
        gyro_res = (float)(1U << g_fs) / 131.F;

        /* Configure accel full scale ragne */
        write_reg(MPU9250_ADDRESS, ACCEL_CONFIG, a_fs << 3);
        accel_res = (float)(1U << a_fs) / 16384.F;

        /* Configure accel low pass filter */
        write_reg(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x08 | a_lp);

        /* Enable I2C bypass mode */
        write_reg(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);

        /* Initialize AK8963 */

        /* Reset */
        write_reg(AK8963_ADDRESS, AK8963_CNTL2, 0x01);
        
        /* Check who am I */
        if (read_reg(AK8963_ADDRESS, AK8963_WHO_AM_I) != 0x48)
                return 2;

        /* Power down */
        write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x00);
        delay_ms(10);

        /* Enter fuse ROM access mode */
        write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
        delay_ms(10);

        /* Read x, y, z axis calibration values */
        read_reg_multi(AK8963_ADDRESS, AK8963_ASAX, data, 3);
        
        mag_adj[0] = ((float)((int16_t)data[0] - 128) * 0.00390625F) + 1.F;
        mag_adj[1] = ((float)((int16_t)data[1] - 128) * 0.00390625F) + 1.F;
        mag_adj[2] = ((float)((int16_t)data[2] - 128) * 0.00390625F) + 1.F;
        
        /* Power down */
        write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x00);
        delay_ms(10);

        /* Set mode[0:3] to 100Hz and output data bit[4] to 16 bit */
        write_reg(AK8963_ADDRESS, AK8963_CNTL, 0b0110 | (1 << 4));
        delay_ms(10);

        return 0;
}

/**
 * @brief Read MPU9250 6-axis imu data
 * 
 * @param ax 
 * @param ay 
 * @param az 
 * @param gx 
 * @param gy 
 * @param gz 
 * @return int 0: successful / 1: i2c error
 */
int mpu9250_read_imu(float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz)
{
        uint8_t data[14];

        if (read_reg_multi(MPU9250_ADDRESS, ACCEL_XOUT_H, data, 14))
                return 1;

        *ax = (float)((int16_t)(data[0] << 8) | (data[1])) * accel_res;
        *ay = (float)((int16_t)(data[2] << 8) | (data[3])) * accel_res;
        *az = (float)((int16_t)(data[4] << 8) | (data[5])) * accel_res;
#ifdef ACCEL_CALIBRATION_PARAM
        *ax *= ACCEL_X_SCALE;
        *ay *= ACCEL_Y_SCALE;
        *az *= ACCEL_Z_SCALE;
#endif
        *gx = (float)((int16_t)(data[8] << 8) | (data[9])) * gyro_res;
        *gy = (float)((int16_t)(data[10] << 8) | (data[11])) * gyro_res;
        *gz = (float)((int16_t)(data[12] << 8) | (data[13])) * gyro_res;

        return 0;
}

/**
 * @brief Read AK8963 3-axis magnetometer data
 * 
 * @param mx
 * @param my
 * @param mz
 * @return int 0: successful / 1: data not ready / 2: i2c error /
 *             3: data overrun
 */
int mpu9250_read_mag(float *mx, float *my, float *mz)
{
        uint8_t data[7];
        int16_t *ptr = (int16_t *)data;
        float tmp, x, y, z;
        
        /* Check data ready */
        if (!(read_reg(AK8963_ADDRESS, AK8963_ST1) & 0x01))
                return 1;
        
        /* Check i2c error */
        if (read_reg_multi(AK8963_ADDRESS, AK8963_HXL, data, 7))
                return 2;
        
        /* Check data overrun */
        if (data[6] & 0x08)
                return 3;

        x = ((float)(*ptr++) * 0.15F * mag_adj[0]) - MAG_X_BIAS;
        y = ((float)(*ptr++) * 0.15F * mag_adj[1]) - MAG_Y_BIAS;
        z = ((float)(*ptr) * 0.15F * mag_adj[2]) - MAG_Z_BIAS;

        *mx = (x * MAG_XX_SCALE) + (y * MAG_XY_SCALE) + (z * MAG_XZ_SCALE);
        *my = (x * MAG_XY_SCALE) + (y * MAG_YY_SCALE) + (z * MAG_YZ_SCALE);
        *mz = (x * MAG_XZ_SCALE) + (y * MAG_YZ_SCALE) + (z * MAG_ZZ_SCALE);

#ifdef MAG_NED2ENU
        /* Transform axes of compass from NED to ENU */
        tmp = *mx;
        *mx = *my;
        *my = tmp;
        *mz = -(*mz);
#endif
        return 0;
}
