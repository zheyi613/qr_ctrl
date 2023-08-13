/**
 * @file vl53l0x.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief simplify function of VL53L0X api (STSW-IMG005)
 * @date 2023-05-15
 */

#include "vl53l0x.h"
#include "vl53l0x_api.h"

// #define DEBUG_MSG

#ifdef DEBUG_MSG
#include "stdio.h"
#endif

VL53L0X_Dev_t my_device;

/**
 * @brief Initialize VL53L0X in selected mode
 * 
 * @param mode 
 * @param period_ms > 30 ms (default) / 200 ms (high accuracy)
 *                    33 ms (long range) / 20 ms (high speed)
 * @return int 0: successful / else: see VL53L0X_Error
 */
int vl53l0x_init(enum VL53L0X_MODE mode, uint16_t period_ms)
{
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        VL53L0X_DeviceInfo_t device_info;
        uint32_t ref_spad_count;
        uint8_t is_aperture_spads;
        uint8_t vhv_settings;
        uint8_t phase_cal;

        my_device.I2cDevAddr = VL53L0X_ADDR;
        my_device.comms_type = 1;
        my_device.comms_speed_khz = 400;
#ifdef DEBUG_MSG
        printf("VL53L0X: DataInit\n\r");
#endif
        Status = VL53L0X_DataInit(&my_device);
        if (Status != VL53L0X_ERROR_NONE)
                return Status;

        my_device.I2cDevAddr = VL53L0X_ADDR;

        Status = VL53L0X_GetDeviceInfo(&my_device, &device_info);

        if (Status == VL53L0X_ERROR_NONE) {
#ifdef DEBUG_MSG
                printf("VL53L0X Infomation:\n\r");
                printf("Device Name: %s\n\r", device_info.Name);
                printf("Type: %s\n\r", device_info.Type);
                printf("ID: %d\n\r", device_info.ProductType);
                printf("Rev Major: %d\n\r", device_info.ProductRevisionMajor);
                printf("Minor: %d\n\r", device_info.ProductRevisionMinor);
#endif
                if ((device_info.ProductRevisionMajor != 1) ||
                    (device_info.ProductRevisionMinor != 1)) {
#ifdef DEBUG_MSG
                        printf("Expect cut 1.1 but not found!\n\r");
#endif
                        Status = VL53L0X_ERROR_NOT_SUPPORTED;
                        return Status;
                }
        } else {
                return Status;
        }
#ifdef DEBUG_MSG
        printf("VL53L0X: StaticInit:\n\r");
#endif
        Status = VL53L0X_StaticInit(&my_device); /* Device initialization */
        if (Status != VL53L0X_ERROR_NONE)
                return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: PerformRefCalibration\n\r");
#endif
        Status = VL53L0X_PerformRefCalibration(&my_device, &vhv_settings,
                                               &phase_cal);
        if (Status != VL53L0X_ERROR_NONE)
                return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: PerformRefSpadManagement\n\r");
#endif  /* Device initialization */
        Status = VL53L0X_PerformRefSpadManagement(&my_device,
                                        &ref_spad_count, &is_aperture_spads);
#ifdef DEBUG_MSG
        printf("refSpadCount = %ld, isApertureSpads = %d\n\r",
               ref_spad_count, is_aperture_spads);
#endif
        if (Status != VL53L0X_ERROR_NONE)
                return Status;
// #ifdef DEBUG_MSG
//         printf("VL53L0X: SetDeviceMode\n\r");
// #endif
//         Status = VL53L0X_SetDeviceMode(&my_device,
//                                        VL53L0X_DEVICEMODE_SINGLE_RANGING);
//         if (Status != VL53L0X_ERROR_NONE)
//                 return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: SetLimitCheckEnable\n\r");
#endif
        Status = VL53L0X_SetLimitCheckEnable(&my_device,
                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        if (Status != VL53L0X_ERROR_NONE)
                return Status;
        Status = VL53L0X_SetLimitCheckEnable(&my_device,
                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        if (Status != VL53L0X_ERROR_NONE)
                return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: SetSensorMode\n\r");
#endif
        switch (mode) {
        case VL53L0X_SENSE_DEFAULT:
                Status = VL53L0X_SetLimitCheckEnable(&my_device,
                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                        (FixPoint1616_t)(1.5f * 0.023f * 655536.f));
                break;
        case VL53L0X_SENSE_LONG_RANGE:
                 Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                        (FixPoint1616_t)(0.1f * 655536.f));
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                        (FixPoint1616_t)(60.f * 655536.f));
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
                                &my_device, 33000);
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetVcselPulsePeriod(&my_device,
                        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetVcselPulsePeriod(&my_device,
                        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
                break;
        case VL53L0X_SENSE_HIGH_SPEED:
                Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                        (FixPoint1616_t)(0.25f * 65536.f));
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                        (FixPoint1616_t)(32.f * 65536.f));
                break;
        case VL53L0X_SENSE_HIGH_ACCURACY:
                Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                        (FixPoint1616_t)(0.25f * 65536.f));
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetLimitCheckValue(&my_device,
                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                        (FixPoint1616_t)(18.f * 65536.f));
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
                                &my_device, 200000);
                if (Status != VL53L0X_ERROR_NONE)
                        return Status;
                Status = VL53L0X_SetLimitCheckEnable(&my_device,
                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
                break;
        }
        if (Status != VL53L0X_ERROR_NONE)
                        return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: SetDeviceMode\n\r");
#endif                        
        Status = VL53L0X_SetDeviceMode(&my_device,
                        VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
        if (Status != VL53L0X_ERROR_NONE)
                        return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: SetInterMeasurementPeriodMilliSeconds\n\r");
#endif
        /* Use continuous measurement mode */
        Status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(&my_device,
                        period_ms);
        if (Status != VL53L0X_ERROR_NONE)
                        return Status;
#ifdef DEBUG_MSG
        printf("VL53L0X: StartMeasurement\n\r");
#endif
        Status = VL53L0X_StartMeasurement(&my_device);

        return Status;
}

/**
 * @brief Check is VL53L0X range ready
 * 
 * @return int 1: ready / 0: not ready
 */
int vl53l0x_is_range_ready(void)
{
        uint8_t new_data_rd = 0;
        
        VL53L0X_GetMeasurementDataReady(&my_device, &new_data_rd);

        return new_data_rd;
}

/**
 * @brief Get measurement data in milimeter
 * 
 * @param dist 
 * @return int 0: successful / else: see VL53L0X_Error
 */
int vl53l0x_get_distance(uint16_t *dist)
{
        VL53L0X_Error Status;
        VL53L0X_RangingMeasurementData_t RangingMeasurementData;

        Status = VL53L0X_GetRangingMeasurementData(&my_device,
                                                   &RangingMeasurementData);
        if (Status != VL53L0X_ERROR_NONE)
                return Status;
        Status = VL53L0X_ClearInterruptMask(&my_device,
                        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        if (Status != VL53L0X_ERROR_NONE)
                return Status;

        *dist = RangingMeasurementData.RangeMilliMeter;

        return Status;
}