/**
 * @file rtos_bus.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief manage blocking mode and interrupt mode when RTOS is running
 * @date 2023-08-10
 */
#ifndef RTOS_BUS_H
#define RTOS_BUS_H

#include <stdint.h>
#include "i2c.h"
#include "spi.h"

enum bus_mode {
        BUS_POLLING_MODE,
        BUS_INTERRUPT_MODE
};

#define I2C_REG_8BIT    I2C_MEMADD_SIZE_8BIT
#define I2C_REG_16BIT   I2C_MEMADD_SIZE_16BIT

void set_bus_mode(enum bus_mode mode);
int i2c_write_multi(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t reg,
                    uint16_t reg_size, uint8_t *data, uint16_t size);
int i2c_read_multi(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t reg,
                   uint16_t reg_size, uint8_t *data, uint16_t size);
int spi_tx(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size);
int spi_rx(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size);
int spi_txrx(SPI_HandleTypeDef *hspi, uint8_t *tx_data, uint8_t *rx_data,
             uint16_t size);

#endif /* RTOS_BUS_H */