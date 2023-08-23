/**
 * @file nrf_payload.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief nrf payload variable format, encoding, decoding definition
 * @date 2023-08-23
 */
#ifndef NRF_PAYLOAD_H
#define NRF_PAYLOAD_H

#include <stdio.h>

/* Payload definition */
struct payload {
    uint16_t throttle;
    int16_t roll_target;    /* +-180deg/32768 LSB */
    int16_t pitch_target;
    int16_t yaw_target;
    uint16_t height_target; /* 0.01 mm LSB */
    uint8_t P;              /* 0.05 LSB */
    uint8_t I;
    uint8_t D;
    uint8_t mode;
    uint8_t unlock;
    uint8_t dummy[17];
};
#define PAYLOAD_WIDTH           sizeof(struct payload)

struct ack_payload {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        uint16_t throttle;
        uint16_t motor[4];
        int16_t height;    /* 0.01 m LSB, range: +-327.68 m */
        uint8_t voltage;   /* 0.1 V LSB, range: 0 ~ 17.5 V */
        uint8_t event;  
};
#define ACK_PAYLOAD_WIDTH       sizeof(struct ack_payload)

/* Decode for payload */
#define DECODING_RADIUS         9.587378e-5F
#define DECODING_HEIGHT         0.001F
#define DECODING_CTRL_GAIN      0.05F

#define decode_payload_throttle(src, dst)       \
        dst = src
#define decode_payload_radius(src, dst)         \
        dst = (float)(src) * DECODING_RADIUS
#define decode_payload_height(src, dst)         \
        dst = (float)(src) * DECODING_HEIGHT
#define decode_payload_ctrl_gain(src, dst)      \
        dst = (float)(src) * DECODING_CTRL_GAIN

/* Encode for payload */
#define ENCODING_RADIUS         10430.374
#define ENCODING_HEIGHT         100.F
#define ENCODING_VOLTAGE        10.F

#define encode_payload_throttle(src, dst)       \
        dst = src
#define encode_payload_motor(src, dst)          \
        dst = src
#define encode_payload_height(src, dst)         \
        dst = (int16_t)(src * ENCODING_HEIGHT)
#define encode_payload_voltage(src, dst)        \
        dst = (uint8_t)(src * ENCODING_VOLTAGE)

#endif /* NRF_PAYLOAD_H */