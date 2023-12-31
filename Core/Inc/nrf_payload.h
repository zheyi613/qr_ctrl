/**
 * @file nrf_payload.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief nrf payload variable format, encoding, decoding definition
 * @date 2023-08-23
 */
#ifndef NRF_PAYLOAD_H
#define NRF_PAYLOAD_H

#include <stdio.h>

/* Mission mode */
#define NORMAL_MODE                0
#define TEST_FAULT_MODE            1

/* Payload definition */
struct payload {
        uint16_t throttle;
        int16_t sp_roll;    /* +-180deg/32768 LSB */
        int16_t sp_pitch;
        int16_t sp_yaw_rate;
        uint16_t sp_height; /* 0.01 mm LSB */
        uint8_t P;          /* 0.025 LSB */
        uint8_t I;          /* 0.025 LSB */
        uint8_t D;          /* 1 LSB */
        uint8_t mode;
        uint8_t fault_ratio;
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
        uint8_t current;   /* 0.1 A LSB, range: 0 ~ 25.5 A */
        uint8_t rec_status;    /* see sd_record.h */
        uint8_t gps_sv_status; /* 0:cksum not pass or numSV = 0, else:numSV */
        uint16_t gps_pAcc; /* 1 cm LSB, range: 0 ~ 65535 (overflow = 65535) */
};
#define ACK_PAYLOAD_WIDTH       sizeof(struct ack_payload)

/* Decode for payload */
#define DECODING_RADIUS         9.587378e-5F
#define DECODING_DEGREE         0.005493164F
#define DECODING_HEIGHT         0.01F
#define DECODING_VOLTAGE        0.1F
#define DECODING_CURRENT        0.1F
#define DECODING_CTRL_PI_GAIN   0.025F
#define DECODING_FAULT_RATIO    0.1F

#define DECODE_PAYLOAD_THROTTLE(src, dst)       \
        dst = (src)
#define DECODE_PAYLOAD_MOTOR(src, dst)          \
        dst = (src)
#define DECODE_PAYLOAD_RADIUS(src, dst)         \
        dst = (float)(src) * DECODING_RADIUS
#define DECODE_PAYLOAD_DEGREE(src, dst)         \
        dst = (float)(src) * DECODING_DEGREE
#define DECODE_PAYLOAD_HEIGHT(src, dst)         \
        dst = (float)(src) * DECODING_HEIGHT
#define DECODE_PAYLOAD_VOLTAGE(src, dst)        \
        dst = (float)(src) * DECODING_VOLTAGE
#define DECODE_PAYLOAD_CURRENT(src, dst)        \
        dst = (float)(src) * DECODING_CURRENT
#define DECODE_PAYLOAD_REC_STATUS(src, dst)     \
        dst = (src)
#define DECODE_PAYLOAD_GPS_SV_STATUS(src, dst)  \
        dst = (src)
#define DECODE_PAYLOAD_GPS_PACC(src, dst)       \
        dst = (src)
#define DECODE_PAYLOAD_CTRL_PI_GAIN(src, dst)   \
        dst = (float)(src) * DECODING_CTRL_PI_GAIN
#define DECODE_PAYLOAD_CTRL_D_GAIN(src, dst)    \
        dst = (float)src
#define DECODE_PAYLOAD_MODE(src, dst)      \
        dst = (src)
#define DECODE_PAYLOAD_FAULT_RATIO(src, dst)    \
        dst = (float)(src) * DECODING_FAULT_RATIO

/* Encode for payload */
#define ENCODING_RADIUS         10430.378F
#define ENCODING_DEGREE         182.044444F
#define ENCODING_HEIGHT         100.F
#define ENCODING_VOLTAGE        10.F
#define ENCODING_CURRENT        10.F
#define ENCODING_CTRL_PI_GAIN   40.F
#define ENCODING_FAULT_RATIO    10.F

#define ENCODE_PAYLOAD_THROTTLE(src, dst)       \
        dst = (uint16_t)(src)
#define ENCODE_PAYLOAD_MOTOR(src, dst)          \
        dst = (uint16_t)(src)
#define ENCODE_PAYLOAD_RADIUS(src, dst)         \
        dst = (int16_t)(src * ENCODING_RADIUS)
#define ENCODE_PAYLOAD_DEGREE(src, dst)         \
        dst = (int16_t)(src * ENCODING_DEGREE)
#define ENCODE_PAYLOAD_HEIGHT(src, dst)         \
        dst = (int16_t)(src * ENCODING_HEIGHT)
#define ENCODE_PAYLOAD_VOLTAGE(src, dst)        \
        dst = (uint8_t)(src * ENCODING_VOLTAGE)
#define ENCODE_PAYLOAD_CURRENT(src, dst)        \
        do {                                    \
                if ((src >= 0) && (src < 25.4F))                        \
                        dst = (uint8_t)(src * ENCODING_CURRENT);        \
                else if (src > 25.4)                                    \
                        dst = 255;                                      \
                else                                                    \
                        dst = 0;                                        \
        } while (0)
#define ENCODE_PAYLOAD_REC_STATUS(src, dst)     \
        dst = (uint8_t)(src)
#define ENCODE_PAYLOAD_GPS_SV_STATUS(src, dst)  \
        dst = (uint8_t)(src)
#define ENCODE_PAYLOAD_GPS_PACC(src, dst)       \
        do {                                    \
                if (src >= 65535)               \
                        dst = 65535;            \
                else                            \
                        dst = (uint16_t)(src);  \
        } while (0)
#define ENCODE_PAYLOAD_CTRL_PI_GAIN(src, dst)   \
        dst = (uint8_t)(src * ENCODING_CTRL_PI_GAIN)
#define ENCODE_PAYLOAD_CTRL_D_GAIN(src, dst)    \
        dst = (uint8_t)src
#define ENCODE_PAYLOAD_MODE(src, dst)           \
        dst = (uint8_t)(src)
#define ENCODE_PAYLOAD_FAULT_RATIO(src, dst)    \
        dst = (uint8_t)(src * ENCODING_FAULT_RATIO)

#endif /* NRF_PAYLOAD_H */