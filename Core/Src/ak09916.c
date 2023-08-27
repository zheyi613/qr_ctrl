/**
 * @file ak09916.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief AK09916 driver
 * @date 2023-03-17
 */

#include "ak09916.h"
#include "rtos_bus.h"
#include "stm32f4xx_hal.h"

#define read_reg_multi(reg, pData, size)                                \
        i2c_read_multi(&hi2c2, AK09916_ADDR << 1, reg, I2C_REG_8BIT,    \
                       pData, size)
#define write_reg_multi(reg, pData, size)                               \
        i2c_write_multi(&hi2c2, AK09916_ADDR << 1, reg, I2C_REG_8BIT,   \
                        pData, size)
#define delay_ms(t)     HAL_Delay(t)

/* AK09916 registers */
#define AK09916_REG_WIA1        0x00
#define AK09916_REG_WIA2        0x01
#define AK09916_REG_ST1         0x10
#define AK09916_REG_HXL         0x11
#define AK09916_REG_HXH         0x12
#define AK09916_REG_HYL         0x13
#define AK09916_REG_HYH         0x14
#define AK09916_REG_HZL         0x15
#define AK09916_REG_HZH         0x16
#define AK09916_REG_ST2         0x18
#define AK09916_REG_CNTL2       0x31
#define AK09916_REG_CNTL3       0x32

/**
 * @brief initialize ak09916
 * 
 * @param mode 
 * @return int 0: successful / 1: failed
 */
int ak09916_init(enum ak09916_mode mode)
{
        uint8_t status;
        uint8_t val = 0x01;
        
        delay_ms(50);
        status = write_reg_multi(AK09916_REG_CNTL3, &val, 1); /* reset */
        if (status)
                return 1;
        status = read_reg_multi(AK09916_REG_WIA2, &val, 1); /* get who I am */
        if (status || (val != 0x09))
                return 1;
        val = mode;
        status = write_reg_multi(AK09916_REG_CNTL2, &val, 1); /* set mode */
        delay_ms(10);
        
        return status;
}

/**
 * @brief get magnetometer data from ak09916
 * 
 * @param mx 
 * @param my 
 * @param mz 
 * @return int 0: successful / 1: not ready or overflow / 2: i2c error
 */
int ak09916_read_data(float *mx, float *my, float *mz)
{
        uint8_t status;
        uint8_t data[8];
        int16_t *ptr = (int16_t *)data;

        status = read_reg_multi(AK09916_REG_ST1, data, 1);
        if (status)
                return 2;
        if (!(data[0] & 0x01)) /* check data ready */
                return 1;
        status = read_reg_multi(AK09916_REG_HXL, data, 8);
        if (status)
                return 2;
        if (data[7] & 0x08) /* check data overflow */
                return 1;
        *mx = (float)(*ptr++) * 0.15;
        *mx -= MAG_CX;
        *my = -(float)(*ptr++) * 0.15;
        *my -= MAG_CY;
        *mz = -(float)(*ptr++) * 0.15;
        *mz -= MAG_CZ;

        return 0;
}

/**
 * @brief set single mode
 * 
 * @return int 0: successful / else: failed
 */
int ak09916_set_single_mode(void)
{
        uint8_t val = 0x01;

        return write_reg_multi(AK09916_REG_CNTL2, &val, 1);
}