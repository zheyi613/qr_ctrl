/**
 * @file lps22hb.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief LPS22HB library
 * @date 2022-10-24
 */
#include "lps22hb.h"
#include "rtos_bus.h"

/* User serial interface */
#define read_reg_multi(reg, pData, size)        			 \
        i2c_read_multi_dma(&hi2c2, LPS22HB_ADDR << 1, reg, I2C_REG_8BIT, \
		           pData, size)
#define write_reg_multi(reg, pData, size)       			 \
        i2c_write_multi_dma(&hi2c2, LPS22HB_ADDR << 1, reg, I2C_REG_8BIT,\
			    pData, size)

int (*lps22hb_read_data)(float *press, float *temp);

enum lps22hb_reg {
	LPS22HB_INTERRUPT_CFG = 0x0B,
	LPS22HB_THS_P_L,
	LPS22HB_THS_P_H,
	LPS22HB_WHO_AM_I = 0x0F,
	LPS22HB_CTRL_REG1,
	LPS22HB_CTRL_REG2,
	LPS22HB_CTRL_REG3,
	LPS22HB_FIFO_CTRL = 0x14,
	LPS22HB_REF_P_XL,
	LPS22HB_REF_P_L,
	LPS22HB_REF_P_H,
	LPS22HB_RPDS_L,
	LPS22HB_RPDS_H,
	LPS22HB_RES_CONF,
	LPS22HB_INT_SOURCE = 0x25,
	LPS22HB_FIFO_STATUS,
	LPS22HB_STATUS,
	LPS22HB_PRESS_OUT_XL,
	LPS22HB_PRESS_OUT_L,
	LPS22HB_PRESS_OUT_H,
	LPS22HB_TEMP_OUT_L,
	LPS22HB_TEMP_OUT_H,
	LPS22HB_LPFP_RES = 0x33
};

static inline int lps22hb_get_whoami(uint8_t *whoami)
{
	return read_reg_multi(LPS22HB_WHO_AM_I, whoami, 1);
}

static inline int lps22hb_srst(void)
{
	uint8_t val = 0x04;

	return write_reg_multi(LPS22HB_CTRL_REG2, &val, 1);
}

/**
 * @brief Set threshold value for interrupt generation
 *
 * @param ths (hPa), min unit = 1/16 hPa
 * @return int
 */
static inline int lps22hb_set_press_threshold(float ths)
{
	int16_t val = (int16_t)(ths * 16);

	return write_reg_multi(LPS22HB_THS_P_L, (uint8_t *)&val, 2);
}

/**
 * @brief Set reference pressure
 *
 * @param ref (hPa), min unit = 1/4096 hPa
 * @return int
 */
static inline int lps22hb_set_ref_press(float ref)
{
	int32_t val = (int32_t)(ref * 4096);

	return write_reg_multi(LPS22HB_REF_P_XL, (uint8_t *)val, 3);
}

/**
 * @brief Set pressure offset
 *
 * @param ofs (hPa), min unit = 1/4096 hPa
 * @return int
 */
static int lps22hb_set_press_ofs(float ofs)
{
	int16_t val = (int16_t)(ofs * 16);

	return write_reg_multi(LPS22HB_RPDS_L, (uint8_t *)&val, 2);
}

static inline uint8_t lps22hb_is_data_rdy(void)
{
	uint8_t status = 0;

	read_reg_multi(LPS22HB_STATUS, &status, 1);
	return ((status & 0x03) == 0x03) ? 1 : 0;
}

static inline const int lps22hb_get_fifo_cnt(void)
{
	uint8_t status = 0;

	read_reg_multi(LPS22HB_FIFO_STATUS, &status, 1);
	return status & 0x3F;
}

#pragma pack(1)
typedef union {
	uint8_t bytes[5];
	struct {
		uint8_t press[3];
		int16_t temp;
	} data;
} lps22hb_data_t;
#pragma pack()

int lps22hb_read_bypass(float *press, float *temp)
{
	int result;
	lps22hb_data_t get_data;
	int32_t intpress = 0;

	if (!lps22hb_is_data_rdy())
		return 1;
	for (uint8_t i = 0; i < 5; i++) {
		result = read_reg_multi(LPS22HB_PRESS_OUT_XL + i,
					get_data.bytes + i, 1);
		if (result)
			return result;
	}

	if (get_data.data.press[2] & 0x80) /* minus case */
		intpress = 0xFF000000UL;
	intpress |= get_data.data.press[2] << 16 | get_data.data.press[1] << 8 |
		    get_data.data.press[0];
	*press = (float)intpress / 4096;
	*temp = (float)get_data.data.temp / 100;

	return 0;
}

int lps22hb_read_stream(float *press, float *temp)
{
	const int cnt = lps22hb_get_fifo_cnt();
	if (cnt < 1)
		return 1;
	int result;
	lps22hb_data_t get_data[cnt];
	uint32_t intpress = 0;
	float press_total = 0;
	float temp_total = 0;

	result = read_reg_multi(LPS22HB_PRESS_OUT_XL, get_data[0].bytes,
				cnt * 5);
	if (result)
		return result;

	for (int i = 0; i < cnt; i++) {
		intpress = 0;
		if (get_data[i].data.press[2] & 0x80) /* minus case */
			intpress = 0xFF000000UL;
		intpress |= get_data[i].data.press[2] << 16 |
			    get_data[i].data.press[1] << 8 |
			    get_data[i].data.press[0];
		press_total += (float)intpress / 4096;
		temp_total += (float)get_data[i].data.temp / 100;
	}

	*press = press_total / cnt;
	*temp = temp_total / cnt;

	return 0;
}

int lps22hb_init(struct lps22hb_cfg lps22hb)
{
	int result;
	uint8_t val = 0, is_bypass;

	result = lps22hb_get_whoami(&val);
	if (val != LPS22HB_ID)
		return result;

	/* Soft reset */
	result = lps22hb_srst();
	if (result)
		return result;

	result = lps22hb_set_press_ofs(LPS22HB_PRESSURE_OFS);
	if (result)
		return result;

	result = lps22hb_set_ref_press(lps22hb.ref_press);
	if (result)
		return result;

	/* Set FIFO mode */
	val = lps22hb.mode << 5;
	result = write_reg_multi(LPS22HB_FIFO_CTRL, &val, 1);
	if (result)
		return result;

	/* Set ODR, LPF, bypass */
	if (lps22hb.mode == LPS22HB_BYPASS_MODE) {
		lps22hb_read_data = &lps22hb_read_bypass;
		is_bypass = 1;
	}
	else {
		lps22hb_read_data = &lps22hb_read_stream;
		is_bypass = 0;
	}

	val = lps22hb.odr << 4 | lps22hb.lpf << 2 | is_bypass << 1;
	result = write_reg_multi(LPS22HB_CTRL_REG1, &val, 1);
	if (result)
		return result;

	/* Reset LPF */
	result = write_reg_multi(LPS22HB_LPFP_RES, &val, 1);
	if (result)
		return result;

	/*
	 * bypass mode: disable fifo and register address automatically
	 * incremented during a multiple byte access whith a serial interface.
	 * stream mode: otherwise
	 */
	val = !is_bypass << 6 | !is_bypass << 4;
	result = write_reg_multi(LPS22HB_CTRL_REG2, &val, 1);
	if (result)
		return result;

	return 0;
}