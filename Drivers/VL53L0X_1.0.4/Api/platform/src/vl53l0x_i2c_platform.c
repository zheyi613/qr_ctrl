/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

#include "rtos_bus.h"

// #define DEBUG_MSG

#ifdef DEBUG_MSG
#include "stdio.h"
#endif

#define    BYTES_PER_WORD        2
#define    BYTES_PER_DWORD       4

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata,
                            int32_t count)
{
    int status = 0;

#ifdef DEBUG_MSG
    printf("Writing %ld to addr 0x%x:\n\r", count, index);
#endif

    status = i2c_write_multi(&hi2c3, address << 1, index, pdata,
                             (int16_t)count);
#ifdef DEBUG_MSG
    for (uint8_t i = 0; i < count; i++) {
        printf("0x%x: %d\n\r", index + i, *(pdata + i));
    }
    if(status != 0)
        printf("unsuccessful!\n\r");
#endif  
    return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = 0;

#ifdef DEBUG_MSG
    printf("Reading %ld to addr 0x%x:\n\r", count, index);
#endif
    status = i2c_read_multi(&hi2c3, address << 1, index, pdata,
                             (int16_t)count);
#ifdef DEBUG_MSG
    for (uint8_t i = 0; i < count; i++) {
        printf("0x%x: %d\n\r", index + i, *(pdata + i));
    }
    if(status != 0)
        printf("unsuccessful!\n\r");
#endif      
    return status;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = 0;
    const int32_t cbyte_count = 1;

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = 0;

    uint8_t  buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);

    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = 0;
    uint8_t  buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = 0;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

    return status;

}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = 0;
	uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;

}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = 0;
	uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}