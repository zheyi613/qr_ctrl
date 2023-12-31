/**
 * @file sd_record.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief definition of sd recording status and data structure
 * @date 2023-08-26
 */
#ifndef SD_RECORD_H
#define SD_RECORD_H

#include "stdint.h"

#define REC_STATUS_PROCESS_MASK             (1U << 0 | 1U << 1 | 1U << 2)
#define REC_STATUS_PROCESS_INIT             (0U << 0)
#define REC_STATUS_PROCESS_IDLE             (1U << 0)
#define REC_STATUS_PROCESS_START            (2U << 0)
#define REC_STATUS_PROCESS_UNDONE           (3U << 0)
#define REC_STATUS_PROCESS_END              (4U << 0)

/* Low layer error in diskio */
#define REC_STATUS_FILESYSTEM_ERROR         (1U << 3)
/* Reset buffer error (send/receive data is ongoing) */
#define REC_STATUS_BUFFER_RESET_ERROR       (1U << 4)
/* Data size not match when writing to buffer */
#define REC_STATUS_WRITE_BUFFER_ERROR       (1U << 5)
/* Data size not match when writing to SD card */
#define REC_STATUS_WRITE_SD_ERROR           (1U << 6)


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
        float world_linear_az;
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
        float roll;
        float pitch;
        float yaw;
        float r_sp;
        float p_sp;
        float y_sp;
};

struct rec_ctrl {
        uint32_t mark;
        uint32_t tick;
        uint32_t throttle;
        uint16_t motor[4];
        float ex;
        float ey;
        float ez;
        uint32_t fault_motor;
};


#endif /* SD_RECORD_H */