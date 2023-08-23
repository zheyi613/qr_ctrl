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

#define DECODE_PAYLOAD_THROTTLE(x)      x
#define DECODE_PAYLOAD_RADIUS(x)        (float)(x) * DECODING_RADIUS
#define DECODE_PAYLOAD_HEIGHT(x)        (float)(x) * DECODING_HEIGHT
#define DECODE_PAYLOAD_CTRL_GAIN(x)     (float)(x) * DECODING_CTRL_GAIN
#define DECODE_PAYLOAD_CTRL_MODE(x)     x

/* Encode for payload */
#define ENCODING_RADIUS         10430.378F
#define ENCODING_HEIGHT         100.F
#define ENCODING_VOLTAGE        10.F

#define ENCODE_PAYLOAD_THROTTLE(x)      x
#define ENCODE_PAYLOAD_MOTOR(x)         x
#define ENCODE_PAYLOAD_RADIUS(x)        (int16_t)(x * ENCODING_RADIUS)
#define ENCODE_PAYLOAD_HEIGHT(x)        (int16_t)(x * ENCODING_HEIGHT)
#define ENCODE_PAYLOAD_VOLTAGE(x)       (uint8_t)(x * ENCODING_VOLTAGE)

#endif /* NRF_PAYLOAD_H */