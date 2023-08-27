
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/


#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"
#include <string.h>
#include "vl53l1_error_codes.h"

#include "rtos_bus.h"

#define _I2CWrite(addr, reg, pData, size)     \
    i2c_write_multi(&hi2c3, addr, reg, I2C_REG_16BIT, pData, size)
#define _I2CRead(addr, reg, pData, size)      \
    i2c_read_multi(&hi2c3, addr, reg, I2C_REG_16BIT, pData, size)

#ifdef VL53L1_LOG_ENABLE
#define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif

/* when not customized by application define dummy one */
#ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L1_GetI2cBus(...) (void)0
#endif

#ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L1_PutI2cBus(...) (void)0
#endif

/* I2C temporary array to exchange byte order and transfer by write/read */
static uint8_t tmp[4];

VL53L1_Error VL53L1_WriteMulti(uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_GetI2cBus();
    if (_I2CWrite(Dev, index, pdata, count)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_GetI2cBus();
    if (_I2CRead(Dev, index, pdata, count)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_WrByte(uint16_t Dev, uint16_t index, uint8_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_GetI2cBus();
    if (_I2CWrite(Dev, index, &data, 1)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_WrWord(uint16_t Dev, uint16_t index, uint16_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    tmp[0] = data >> 8;
    tmp[1] = data & 0x00FF;

    VL53L1_GetI2cBus();
    if (_I2CWrite(Dev, index, tmp, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_WrDWord(uint16_t Dev, uint16_t index, uint32_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    tmp[0] = (data >> 24) & 0xFF;
    tmp[1] = (data >> 16) & 0xFF;
    tmp[2] = (data >> 8)  & 0xFF;
    tmp[3] = (data >> 0 ) & 0xFF;

    VL53L1_GetI2cBus();
    if (_I2CWrite(Dev, index, tmp, 4)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_UpdateByte(uint16_t Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L1_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L1_Error VL53L1_RdByte(uint16_t Dev, uint16_t index, uint8_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_GetI2cBus();
    if (_I2CRead(Dev, index, &data, 1)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_RdWord(uint16_t Dev, uint16_t index, uint16_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_GetI2cBus();
    if (_I2CRead(Dev, index, tmp, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    *data = ((uint16_t)tmp[0] << 8) + (uint16_t)tmp[1];
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_RdDWord(uint16_t Dev, uint16_t index, uint32_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;

    VL53L1_GetI2cBus();
    if (_I2CRead(Dev, index, tmp, 4)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    *data = ((uint32_t)tmp[0] << 24) + ((uint32_t)tmp[1] << 16) +
            ((uint32_t)tmp[2] << 8) + (uint32_t)tmp[3];
    VL53L1_PutI2cBus();

    return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{

    /* Returns current tick count in [ms] */

	VL53L1_Error status  = VL53L1_ERROR_NONE;

	//*ptick_count_ms = timeGetTime();
	*ptick_count_ms = 0;

#ifdef VL53L1_LOG_ENABLE
	trace_print(
		VL53L1_TRACE_LEVEL_DEBUG,
		"VL53L1_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);
#endif

	return status;
}











#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)














VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;

	trace_print(VL53L1_TRACE_LEVEL_INFO, "VL53L1_GetTimerFrequency: Freq : %dHz\n", *ptimer_freq_hz);
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	(void)dev;
	HAL_Delay(wait_ms);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(uint16_t dev, int32_t wait_us){
	(void)dev;
	HAL_Delay(wait_us/1000);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	uint16_t 			dev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{

	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms = 0;
	uint32_t     current_time_ms = 0;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint8_t      trace_functions = VL53L1_TRACE_FUNCTION_NONE;
#endif

//	char   register_name[VL53L1_MAX_STRING_LENGTH];

    /* look up register name */
#ifdef PAL_EXTENDED
	VL53L1_get_register_name(
			index,
			register_name);
#else
//	VL53L1_COPYSTRING(register_name, "");
#endif

	/* Output to I2C logger for FMT/DFT  */

    /*trace_i2c("WaitValueMaskEx(%5d, 0x%04X, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, index, value, mask, poll_delay_ms); */
    trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, register_name, value, mask, poll_delay_ms);

	/* calculate time limit in absolute time */

	 VL53L1_GetTickCount(&start_time_ms);

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = VL53L1_get_trace_functions();
	VL53L1_set_trace_functions(VL53L1_TRACE_FUNCTION_NONE);
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (polling_time_ms < timeout_ms) &&
		   (found == 0)) {

		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_RdByte(
							dev,
							index,
							&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
					dev,
					poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		polling_time_ms = current_time_ms - start_time_ms;

	}

#ifdef VL53L1_LOG_ENABLE
	/* Restore function logging */
	VL53L1_set_trace_functions(trace_functions);
#endif

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}




