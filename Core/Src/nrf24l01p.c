/**
 * @file nrf24l01p.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief NRF24L01+ driver for stm32
 * @date 2023-03-05
 */

#include "nrf24l01p.h"
#include "dwt_delay.h"
#include "gpio.h"
#include "rtos_bus.h"

/* Define user spi function */
#define nrf_spi_txrx(pTxData,pRxData)   \
        spi_txrx(&hspi2, pTxData, pRxData, 1)
#define nrf_spi_rx(pData, size)         \
        spi_rx(&hspi2, pData, size)
#define nrf_spi_tx(pData, size)         \
        spi_tx(&hspi2, pData, size)
#define cs_high()       \
        HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_SET)
#define cs_low()        \
        HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_RESET)
#define ce_high()       \
        HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)
#define ce_low()        \
        HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
#define delay_us(val)   DWT_Delay(val)

/* NRF24L01+ command */
#define NRF24L01P_CMD_R_REGISTER                  0b00000000
#define NRF24L01P_CMD_W_REGISTER                  0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111

/* NRF24L01+ register */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08
#define NRF24L01P_REG_RPD               0x09
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D

/* Timing information us */
#define NRF24L01P_T_PD2STBY_IN          1500
#define NRF24L01P_T_PD2STBY_EX          150
#define NRF24L01P_T_STBY2A              130
#define NRF24L01P_T_HCE                 10
#define NRF24L01P_T_PECE2CSN            4

/* Static variable */
static uint8_t address_width = 5;

static uint8_t send_cmd_only(uint8_t command)
{
        uint8_t status;

        cs_low();
        nrf_spi_txrx(&command, &status);
        cs_high();

        return status;
}

#define get_status()            \
        send_cmd_only(NRF24L01P_CMD_NOP)

#define flush_rx_fifo()         \
        send_cmd_only(NRF24L01P_CMD_FLUSH_RX)

#define flush_tx_fifo()         \
        send_cmd_only(NRF24L01P_CMD_FLUSH_TX)

static uint8_t read_multi(uint8_t command, uint8_t *data, uint8_t size)
{
        uint8_t status;

        cs_low();
        nrf_spi_txrx(&command, &status);
        nrf_spi_rx(data, size);
        cs_high();

        return status;
}

static uint8_t write_multi(uint8_t command, uint8_t *data, uint8_t size)
{
        uint8_t status;

        cs_low();
        nrf_spi_txrx(&command, &status);
        nrf_spi_tx(data, size);
        cs_high();

        return status;
}

#define read_reg_multi(reg, data, size)        \
        read_multi(NRF24L01P_CMD_R_REGISTER | (reg), data, size)

static inline uint8_t read_reg(uint8_t reg)
{
        uint8_t val;

        read_reg_multi(reg, &val, 1);
        return val;
}

#define write_reg_multi(reg, data, size)       \
        write_multi(NRF24L01P_CMD_W_REGISTER | (reg), data, size)

static inline void write_reg(uint8_t reg, uint8_t val)
{
        write_reg_multi(reg, &val, 1);
}

#define get_rx_payload_width(p_width)                  \
        read_multi(NRF24L01P_CMD_R_RX_PL_WID, p_width, 1)

#define read_rx_payload(payload, width)         \
        read_multi(NRF24L01P_CMD_R_RX_PAYLOAD, payload, width)

#define write_tx_payload(payload, width)        \
        write_multi(NRF24L01P_CMD_W_TX_PAYLOAD, payload, width)

static inline void power_up(void)
{
        uint8_t reg_val = read_reg(NRF24L01P_REG_CONFIG);
        reg_val |= 1 << 1;

        write_reg(NRF24L01P_REG_CONFIG, reg_val);

        delay_us(NRF24L01P_T_PD2STBY_IN);
}

int set_tx_address(uint8_t *address)
{
        uint8_t check_address[5];
        uint8_t i;

        write_reg_multi(NRF24L01P_REG_TX_ADDR, address, address_width);
        read_reg_multi(NRF24L01P_REG_TX_ADDR, check_address, address_width);

        for (i = 0; i < address_width; i++) {
            if (address[i] != check_address[i])
                break;
        }
        if (i != 5)
                return 1;
        return 0;
}

static int set_rx_address(uint8_t *address, uint8_t pipe)
{
        uint8_t check_address[5];
        uint8_t i;

        if (pipe < 2) {
            write_reg_multi(NRF24L01P_REG_RX_ADDR_P0 + pipe, address,
                            address_width);
            read_reg_multi(NRF24L01P_REG_RX_ADDR_P0 + pipe, check_address,
                            address_width);
            for (i = 0; i < address_width; i++) {
                if (address[i] != check_address[i])
                    break;
            }
            if (i != address_width)
                return 1;
        } else {
            write_reg_multi(NRF24L01P_REG_RX_ADDR_P0 + pipe, address, 1);
            read_reg_multi(NRF24L01P_REG_RX_ADDR_P0 + pipe, check_address, 1);
            if (address[0] != check_address[0])
                return 1;
        }
        
        return 0;
}

/**
 * @brief reset nrf24l01+
 * 
 * @return int 0: successful / 1: maybe lost device 
 */
static int soft_reset(void)
{
        uint8_t default_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        
        ce_low();
        /* Set RX address reset value and checking */
        if (set_rx_address(default_address, 0))
            return 1;

        write_reg(NRF24L01P_REG_CONFIG, 0x08);
        write_reg(NRF24L01P_REG_EN_AA, 0x3F);
        write_reg(NRF24L01P_REG_EN_RXADDR, 0x03);
        write_reg(NRF24L01P_REG_SETUP_AW, 0x03);
        write_reg(NRF24L01P_REG_SETUP_RETR, 0x03);
        write_reg(NRF24L01P_REG_RF_CH, 0x02);
        write_reg(NRF24L01P_REG_RF_SETUP, 0x0E);
        write_reg(NRF24L01P_REG_STATUS, 0x70);
        write_reg(NRF24L01P_REG_RX_PW_P0, 0x00);
        write_reg(NRF24L01P_REG_RX_PW_P1, 0x00);
        write_reg(NRF24L01P_REG_RX_PW_P2, 0x00);
        write_reg(NRF24L01P_REG_RX_PW_P3, 0x00);
        write_reg(NRF24L01P_REG_RX_PW_P4, 0x00);
        write_reg(NRF24L01P_REG_RX_PW_P5, 0x00);
        write_reg(NRF24L01P_REG_DYNPD, 0x00);
        write_reg(NRF24L01P_REG_FEATURE, 0x00);

        flush_rx_fifo();
        flush_tx_fifo();

        return 0;
}

/**
 * @brief initialize nrf24l01+
 * 
 * @param param 
 * @return int 0: successful / 1: failed (see soft_reset())
 */
int nrf24l01p_init(struct nrf24l01p_cfg *param)
{
        uint8_t reg_val;

        delay_us(150000); /* Wait > 100ms to power up */

        if (soft_reset())
            return 1;
        power_up();

        reg_val = read_reg(NRF24L01P_REG_CONFIG) & 0xFA;
        reg_val |= param->mode | param->crc_len << 2;
        write_reg(NRF24L01P_REG_CONFIG, reg_val);

        address_width = param->address_width;
        write_reg(NRF24L01P_REG_SETUP_AW, param->address_width - 2);

        reg_val = (param->auto_retransmit_count & 0x0F) |
                  (((param->auto_retransmit_delay / 250 - 1) << 4) & 0xF0);
        write_reg(NRF24L01P_REG_SETUP_RETR, reg_val);

        write_reg(NRF24L01P_REG_RF_CH, param->channel - 2400);

        reg_val = read_reg(NRF24L01P_REG_RF_SETUP) & 0xD1;
        reg_val |= (param->air_data_rate << 3) | (param->output_power << 1);
        write_reg(NRF24L01P_REG_RF_SETUP, reg_val);

#ifdef NRF24L01P_ACK_PAYLOAD
        write_reg(NRF24L01P_REG_DYNPD, 0x01);
        write_reg(NRF24L01P_REG_FEATURE, 0x06);
#else
        write_reg(NRF24L01P_REG_RX_PW_P0, NRF24L01P_PAYLOAD_WIDTH);
#endif
        return 0;
}

/**
 * @brief Go RX mode and Start receiving package
 * Note:  Minimum time within CE high and CSN low: 4 us
 */
void nrf24l01p_start_rx(void)
{
        ce_high();
}

/**
 * @brief receive one payload in prx mode (p59 c,d)
 * 
 * @param payload 
 */
void nrf24l01p_receive(uint8_t *payload)
{
#ifdef NRF24L01P_ACK_PAYLOAD
        uint8_t width;

        get_rx_payload_width(&width);
        read_rx_payload(payload, width);
        /* Clear RX_DR and TX_DS interrupt bit to reset IRQ pin */
        write_reg(NRF24L01P_REG_STATUS, 0x60);
#else
        uint8_t status = get_status();
     
        if (!(status & 0x40))
                return;

        read_rx_payload(payload, NRF24L01P_PAYLOAD_WIDTH);
        /* Clear RX_DR interrupt bit to reset IRQ pin */
        write_reg(NRF24L01P_REG_STATUS, 0x40);
#endif
}

#ifdef NRF24L01P_ACK_PAYLOAD
void nrf24l01p_transmit(uint8_t *payload, uint8_t width)
#else
void nrf24l01p_transmit(uint8_t *payload)
#endif
{
        flush_tx_fifo();

#ifdef NRF24L01P_ACK_PAYLOAD
        write_tx_payload(payload, width);
#else
        write_tx_payload(payload, NRF24L01P_PAYLOAD_WIDTH);
#endif
        ce_high();
        delay_us(NRF24L01P_T_HCE);
        ce_low();
}

/**
 * @brief clear tx interrupt bit of status (get ack payload)
 * 
 * @return int 0: transmit successfully / 1: up to max retransmittion count
 */
#ifdef NRF24L01P_ACK_PAYLOAD
int nrf24l01p_tx_irq(uint8_t *payload)
#else
int nrf24l01p_tx_irq(void)
#endif
{
        uint8_t status = get_status();
        /* Clear TX_DS and MAX_RT interrupt bit */
        write_reg(NRF24L01P_REG_STATUS, status);

        if (!(status & 0x20))
                return 1;

#ifdef NRF24L01P_ACK_PAYLOAD
        uint8_t width;
        get_rx_payload_width(&width);
        read_rx_payload(payload, width);
#endif
        return 0;
}

#ifdef NRF24L01P_ACK_PAYLOAD
/* Immediately upload ACK payload before entering TX mode
 * Note: All the transmission must be completed within 130 us (max: 170us)
 *       after irq pin active low. Otherwise the module will
 *       go unexpected state.
 */

/**
 * @brief Write ACK payload to TX FIFO (use in PRX RX mode)
 * Note:  If ACK payload is uploaded immediately when irq pin is actived low.
 *        All the SPI transfer must be completed within PLL clock (130 us)
 *        (max : 170 us). Otherwise the module will go unexpected state
 *        when SPI transmission is ongoing.
 * 
 * @param payload ACK payload pointer
 * @param width ACK payload width
 */
void nrf24l01p_write_ack_payload(uint8_t *payload, uint8_t width)
{
        write_multi(NRF24L01P_CMD_W_ACK_PAYLOAD, payload, width);
}
#endif