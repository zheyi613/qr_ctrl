/**
 * @file lps22hb.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief LPS22HB library
 * @date 2022-10-24
 */

#ifndef LPS22HB_H
#define LPS22HB_H

#define LPS22HB_I2C     hi2c1     
#define LPS22HB_SA0     0
#define LPS22HB_ADDR    (0b1011100 | LPS22HB_SA0) 
#define LPS22HB_ID      0xB1

#define LPS22HB_PRESSURE_OFS    4.67 /* hPa */

/* 
 * Bypass Mode:
 * The data will be receive without using FIFO. To avoid mixing data, the BDU
 * (Block Data Update) will be set. Output data registers not updated until
 * MSB and LSB have been read. Meanwhile, IF_ADD_INC bit must be set, so you
 * will use a single-byte read.
 * 
 * Stream Mode (recommended):
 * The data will be receive using FIFO. Your data rate can lower than ODR.
 * According to FIFO stored data level, you will get the average data in once
 * read.
 */
enum lps22hb_fifo {
        LPS22HB_BYPASS_MODE,
        LPS22HB_STREAM_MODE = 2,
};

enum lps22hb_odr {
        LPS22HB_POWER_DOWN,
        LPS22HB_1HZ,
        LPS22HB_10HZ,
        LPS22HB_25HZ,
        LPS22HB_50HZ,
        LPS22HB_75HZ
};

enum lps22hb_lpf {
        LPS22HB_BW_ODR_DIV_2,
        LPS22HB_BW_ODR_DIV_9 = 2,
        LPS22HB_BW_ODR_DIV_20
};

/*
 * is_spi: 1: spi / 0: i2c (TO DO)
 * mode:   fifo_mode refer to enum lps22hb_fifo
 * odr:    output data rate refer to enum lps22hb_odr
 * lpf:    low pass filter refer to enum lps22hb_lpf
 * ref_press: reference pressure
 */
struct lps22hb_cfg {
        enum lps22hb_fifo mode;
        enum lps22hb_odr odr;
        enum lps22hb_lpf lpf;
        float ref_press;
};

int lps22hb_init(struct lps22hb_cfg lps22hb);
extern int (*lps22hb_read_data)(float *press, float *temp);

#endif