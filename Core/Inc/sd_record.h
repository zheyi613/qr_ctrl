/**
 * @file sd_record.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief definition of sd card recording event
 * @date 2023-08-26
 */
#ifndef SD_RECORD_H
#define SD_RECORD_H

#include "stdint.h"

#define REC_STATUS_PROCESS_MASK             (1U << 0 | 1U << 1)
#define REC_STATUS_PROCESS_START            (0U << 0)
#define REC_STATUS_PROCESS_UNDONE           (1U << 0)
#define REC_STATUS_PROCESS_END              (2U << 0)
#define REC_STATUS_PROCESS_IDLE             (3U << 0)

#define REC_STATUS_FILESYSTEM_ERROR         (1U << 2)
#define REC_STATUS_BUFFER_CREATE_ERROR      (1U << 3)
#define REC_STATUS_WRITE_BUFFER_MISSING     (1U << 4)
#define REC_STATUS_WRITE_SD_ERROR           (1U << 5)


#define REC_MARK_IMU                        1
#define REC_MARK_MAG                        2
#define REC_MARK_BARO                       3
#define REC_MARK_TOF                        4
#define REC_MARK_BATTERY                    5
#define REC_MARK_ATT                        6
#define REC_MARK_CTRL                       7

struct rec_imu {
        uint32_t mark;
        uint32_t tick;
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
};

struct rec_mag {
        uint32_t mark;
        uint32_t tick;
        float mx;
        float my;
        float mz;
};

struct rec_baro {
        uint32_t mark;
        uint32_t tick;
        float press;
        float temp;
        float height;
};

struct rec_tof {
        uint32_t mark;
        uint32_t tick;
        uint32_t distance;
};

struct rec_battery {
        uint32_t mark;
        uint32_t tick;
        float voltage;
        float current;
};

struct rec_att {
        uint32_t mark;
        uint32_t tick;
        float q[4];
        float roll;
        float pitch;
        float yaw;
};

struct rec_ctrl {
        uint32_t mark;
        uint32_t tick;
        uint32_t throttle;
        uint16_t motor[4];
};


#endif /* SD_RECORD_H */