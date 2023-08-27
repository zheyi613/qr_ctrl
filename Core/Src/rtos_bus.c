/**
 * @file rtos_bus.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief manage blocking mode and interrupt mode when RTOS is running
 * @date 2023-08-10
 */
#include "rtos_bus.h"
#include "FreeRTOS.h"
#include "task.h"

#define MAX_SPI_POLLING_DATA_SIZE       64

uint8_t bus_int_mode;

/**
 * @brief set bus mode
 * 
 * @param mode 0: polling (blocking) / 1: interrupt (non-blocking)
 */
void set_bus_mode(enum bus_mode mode)
{
        bus_int_mode = (uint8_t)mode;
}

int i2c_write_multi(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t reg,
                    uint16_t reg_size, uint8_t *data, uint16_t size)
{
        int status;

        if (bus_int_mode) {
                status = HAL_I2C_Mem_Write_DMA(hi2c, address, reg, reg_size,
                                               data, size);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        } else {
                status = HAL_I2C_Mem_Write(hi2c, address, reg, reg_size,
                                           data, size, 1);
        }
        return status;
}

int i2c_read_multi(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t reg,
                   uint16_t reg_size, uint8_t *data, uint16_t size)
{
        int status;

        if (bus_int_mode) {
                status = HAL_I2C_Mem_Read_DMA(hi2c, address, reg, reg_size,
                                              data, size);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        } else {
                status = HAL_I2C_Mem_Read(hi2c, address, reg, reg_size,
                                          data, size, 1);
        }
        return status;
}

int spi_tx(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size)
{
        int status;

        if (bus_int_mode) {
                if (size <= MAX_SPI_POLLING_DATA_SIZE) {
                        status = HAL_SPI_Transmit(hspi, data, size, 1);
                } else {
                        status = HAL_SPI_Transmit_DMA(hspi, data, size);
                        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
                }
        } else {
                status = HAL_SPI_Transmit(hspi, data, size, 100);
        }
        return status;
}

int spi_rx(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size)
{
        int status;

        if (bus_int_mode) {
                if (size <= MAX_SPI_POLLING_DATA_SIZE) {
                        status = HAL_SPI_Receive(hspi, data, size, 1);
                } else {
                        status = HAL_SPI_Receive_DMA(hspi, data, size);
                        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
                }
        } else {
                status = HAL_SPI_Receive(hspi, data, size, 100);
        }
        return status;
}

int spi_txrx(SPI_HandleTypeDef *hspi, uint8_t *tx_data, uint8_t *rx_data,
             uint16_t size)
{
        int status = 0;

        if (bus_int_mode) {
                if (size <= MAX_SPI_POLLING_DATA_SIZE) {
                        status = HAL_SPI_TransmitReceive(hspi, tx_data,
                                                         rx_data, size, 1);
                } else {
                        status = HAL_SPI_TransmitReceive_DMA(hspi, tx_data,
                                                             rx_data, size);
                        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
                }
        } else {
                status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data,
                                                 size, 100);
        }
        return status;
}
