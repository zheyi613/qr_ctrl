/**
 * @file nrf24l01p.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief NRF24L01+ driver for stm32
 * @date 2023-03-05
 */

#ifndef NRF24L01P_H
#define NRF24L01P_H

#include <stdint.h>

/* 1 - 32 bytes
 * If Enable ACK payload, don't care about it
 */
#define NRF24L01P_PAYLOAD_WIDTH        8

/* Enable NRF24L01+ ACK payload
 * If enable ACK payload, must concern ARD setting.
 * See specification in page 59 a.
 */
#define NRF24L01P_ACK_PAYLOAD

enum nrf24l01p_mode {
        PTX_MODE,
        PRX_MODE
};

enum nrf24l01p_air_data_rate {
        _1Mbps   = 0,
        _2Mbps   = 1,
        _250kbps = 4
};

enum nrf24l01p_output_power {
        MINUS_18dBm,
        MINUS_12dBm,
        MINUS_6dBm,
        _0dBm
};

enum nrf24l01p_crc_len {
        CRC_ONE_BYTE,
        CRC_TWO_BYTES
};

struct nrf24l01p_cfg {
        enum nrf24l01p_mode mode;
        enum nrf24l01p_crc_len crc_len;
        enum nrf24l01p_air_data_rate air_data_rate;
        enum nrf24l01p_output_power output_power;
        uint16_t channel;               /* 2400 - 2525 MHz */
        uint8_t address_width;          /* 3 - 5 bytes */
        uint8_t auto_retransmit_count;  /* 0 - 15 */
        uint16_t auto_retransmit_delay; /* 250, 500, 700..., 4000us */
};

int nrf24l01p_init(struct nrf24l01p_cfg *param);
uint32_t nrf24l01p_wait_rx(uint32_t timeout);
void nrf24l01p_receive(uint8_t *payload);

#ifdef NRF24L01P_ACK_PAYLOAD
void nrf24l01p_transmit(uint8_t *payload, uint8_t width);
int nrf24l01p_tx_irq(uint8_t *payload);
void nrf24l01p_write_ack_payload(uint8_t *payload, uint8_t width);
#else
void nrf24l01p_transmit(uint8_t *payload);
int nrf24l01p_tx_irq(void);
#endif

void nrf24l01p_start_rx(void);

#endif /* NRF24L01P_H */