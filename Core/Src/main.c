/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "dwt_delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "rtos_bus.h"
#include "lps22hb.h"
#include "icm20948.h"
#include "ak09916.h"
#include "VL53L1X_api.h"
#include "nrf24l01p.h"
#include "nrf_payload.h"
#include "math.h"
#include "ahrs.h"
#include "diskio.h"
#include "sd_record.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Configure tasks to enable */
#define RADIO_TASK
#define SENSOR_TASK
// #define TOF_TASK
#define GPS_TASK
#define SD_TASK
#define ADC_TASK
#define CTRL_TASK
#define MSG_TASK

/* ESC calibration mode */
// #define ESC_CALIBRATION

/* Module initialize failed number */
enum module_init_failed_id {
  MODULE_FAILED_NRF24L01P,
  MODULE_FAILED_TOF,
  MODULE_FAILED_LPS22HB,
  MODULE_FAILED_ICM20948,
  MODULE_FAILED_AK09916
};

#define VL53L1X_ADDR        0x52

#define MOTOR_DUTY_RANGE    2000 /* esc standard: 1-2ms, pwm arr: 2000-3999 */
#define MOTOR_CTRL_RANGE    MOTOR_DUTY_RANGE * 0.2
#define MOTOR_INITIAL_DUTY  0  /* max: 1999, must more than zero to lock ESC */
#define MAX_THROTTLE        MOTOR_DUTY_RANGE * 0.9 /* max:  90% duty */

#define MIN_THRUST          0.35F
#define MAX_THRUST          7.0F

enum {
  NORMAL_MODE,
  ALTITUDE_MODE
};

#define TIME_PER_TICK       0.002F

#define RAD2DEG             57.29577F
#define DEG2RAD             0.017453292F

/* GPS buffer size */
#define GPS_BUFFER_SIZE     256

/* SD card recording buffer size */
#define SD_BUFFER_SIZE      1024
#define SD_BUFFER_LEVEL     512

/* SD recording mode configuration */
#define REC_ONE_CYCLE_MODE
// #define REC_THROTTLE_TRIGGER_MODE
#define REC_CYCLE_MS        10000

/* Select SD recording data */
#define REC_IMU
#define REC_MAG
// #define REC_BARO

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENU2NED(enu_x, enu_y, enu_z)  \
  do {                                \
    float tmp = enu_x;                \
                                      \
    enu_x = enu_y;                    \
    enu_y = tmp;                      \
    enu_z = -enu_z;                   \
  } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t radio_handler;
TaskHandle_t sensor_handler;
TaskHandle_t tof_handler;
TaskHandle_t gps_handler;
TaskHandle_t sd_handler;
TaskHandle_t adc_handler;
TaskHandle_t ctrl_handler;
TaskHandle_t msg_handler;

StreamBufferHandle_t sd_buf_handler;
/* Use sensor task(same as systick) to store current tick */
TickType_t current_tick;

struct payload pl;
struct ack_payload ack_pl;

struct sensor_data {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
  float pressure;
  float temperature;
  uint16_t distance;
} sensor;

struct attitude {
  float q[4];
  float roll;
  float pitch;
  float yaw;
  float roll_target;
  float pitch_target;
  float yaw_target;
} att;

struct position {
  float distance;
  float height;
  float height_target;
} pos;

struct battery {
  float voltage;
  float current;
} bat;

uint16_t throttle;
uint16_t motor[4];

struct ctrl_parameter {
  float P;
  float I;
  float D;
  uint8_t mode;
} ctrl_param;

/* Variable of sd spi timer (ms) */
WORD Timer1, Timer2;

/* SD card recording status */
#if defined(REC_ONE_CYCLE_MODE) 
  uint8_t rec_status = REC_STATUS_PROCESS_START;
#elif defined(REC_THROTTLE_TRIGGER_MODE)
  uint8_t rec_status = REC_STATUS_PROCESS_IDLE;
#endif
/* GPS buffer (transfer by double buffer) */
char gps_buffer[2][GPS_BUFFER_SIZE];
char *gps_rx_buf_ptr;
uint8_t gps_idle_buf_id;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void radio_task(void *param);
static void sensor_task(void *param);
static void tof_task(void *param);
static void gps_task(void *param);
static void sd_task(void *param);
static void adc_task(void *param);
static void ctrl_task(void *param);
static void msg_task(void *param);

void module_init_failed(enum module_init_failed_id id)
{
  char msg[100], len, count;

  switch (id) {
  case MODULE_FAILED_NRF24L01P:
    len = snprintf(msg, 100, "Initialize NRF24L01+ failed...\n");
    break;
  case MODULE_FAILED_TOF:
    len = snprintf(msg, 100, "Initialize TOF module failed...\n");
    break;
  case MODULE_FAILED_LPS22HB:
    len = snprintf(msg, 100, "Initialize LPS22HB failed...\n");
    break;
  case MODULE_FAILED_ICM20948:
    len = snprintf(msg, 100, "Initialize ICM-20948 failed...\n");
    break;
  case MODULE_FAILED_AK09916:
    len = snprintf(msg, 100, "Initialize AK-09916 failed...\n");
    break;
  default:
    len = snprintf(msg, 100, "Initialize not defined failed...\n");
  }
  while (1) {
    CDC_Transmit_FS((uint8_t *)msg, len);
    count = id + 1;

    do {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(250);
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(250);
    } while (--count);

    HAL_Delay(1000);
  }
}
extern void attitude_err(float q[4], float sp_r, float sp_p, float sp_y,
                         float err[3]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	BaseType_t status;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* initialize global variable */
  memset(&sensor, 0, sizeof(struct sensor_data));
  memset(&att, 0, sizeof(struct attitude));
  memset(&pos, 0, sizeof(struct position));
  memset(&bat, 0, sizeof(struct battery));
  memset(&pl, 0, PAYLOAD_WIDTH);
  memset(&ack_pl, 0, ACK_PAYLOAD_WIDTH);
  memset(&ctrl_param, 0, sizeof(struct ctrl_parameter));
  memset(gps_buffer, 0, 2 * GPS_BUFFER_SIZE);

	#ifdef ESC_CALIBRATION
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + 1999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + 1999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + 1999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + 1999);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  
  HAL_Delay(5000);
  HAL_Delay(5000);

  throttle = MOTOR_INITIAL_DUTY;
  for (uint8_t i = 0; i < 4; i++) {
    motor[i] = throttle;
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + motor[0]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + motor[1]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + motor[2]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + motor[3]);
  HAL_Delay(5000);
  HAL_Delay(5000);
  #endif
  
  HAL_Delay(200);

  set_bus_mode(BUS_POLLING_MODE); /* set bus to blocking mode */

#ifdef RADIO_TASK
  struct nrf24l01p_cfg nrf24l01p_param = {
    .mode = PRX_MODE,
    .crc_len = CRC_TWO_BYTES,
    .air_data_rate = _2Mbps,
    .output_power = _0dBm,
    .channel = 2502,
    .address_width = 5,
    .auto_retransmit_count = 6,
    .auto_retransmit_delay = 750
  };
  if (nrf24l01p_init(&nrf24l01p_param))
    module_init_failed(MODULE_FAILED_NRF24L01P);
#endif
#ifdef TOF_TASK
  uint8_t try_count = 10, tof_state = 0;
  int8_t st = 0;
  do {
    st = VL53L1X_BootState(VL53L1X_ADDR, &tof_state);
  } while (--try_count && !tof_state);
  if (st != 0)
    module_init_failed(MODULE_FAILED_TOF);

  if (!tof_state)
    module_init_failed(MODULE_FAILED_TOF);
  if (VL53L1X_SensorInit(VL53L1X_ADDR))
    module_init_failed(MODULE_FAILED_TOF);
  if (VL53L1X_SetDistanceMode(VL53L1X_ADDR, 2))
    module_init_failed(MODULE_FAILED_TOF);
  if (VL53L1X_SetTimingBudgetInMs(VL53L1X_ADDR, 20))
    module_init_failed(MODULE_FAILED_TOF);
  if (VL53L1X_SetInterMeasurementInMs(VL53L1X_ADDR, 20))
    module_init_failed(MODULE_FAILED_TOF);
  if (VL53L1X_StartRanging(VL53L1X_ADDR))
    module_init_failed(MODULE_FAILED_TOF);
#endif
#ifdef SENSOR_TASK
	struct lps22hb_cfg lps22hb = {
   .mode = LPS22HB_STREAM_MODE,
		.odr = LPS22HB_75HZ,
		.lpf = LPS22HB_BW_ODR_DIV_9,
		.ref_press = 0.0
	};
	if (lps22hb_init(lps22hb))
    module_init_failed(MODULE_FAILED_LPS22HB);
  
	if (icm20948_init(1125, GYRO_2000_DPS, ACCEL_16G, LP_BW_119HZ))
    module_init_failed(MODULE_FAILED_ICM20948);

  if (ak09916_init(AK09916_100HZ_MODE))
    module_init_failed(MODULE_FAILED_AK09916);

  icm20948_read_axis6(&sensor.ax, &sensor.ay, &sensor.az,
                      &sensor.gx, &sensor.gy, &sensor.gz);
  ak09916_read_data(&sensor.mx, &sensor.my, &sensor.mz);
  ENU2NED(sensor.ax, sensor.ay, sensor.az);
  ENU2NED(sensor.mx, sensor.my, sensor.mz);
  ahrs_init(sensor.ax, sensor.ay, sensor.az,
            sensor.mx, sensor.my, sensor.mz);
  // ahrs_init_imu(sensor.ax, sensor.ay, sensor.az);
  ahrs2euler(&att.roll, &att.pitch, &att.yaw);
  att.yaw_target = att.yaw;
#endif

  set_bus_mode(BUS_INTERRUPT_MODE); /* set bus to non blocking mode */

#ifndef ESC_CALIBRATION  
  throttle = MOTOR_INITIAL_DUTY;
  for (uint8_t i = 0; i < 4; i++) {
    motor[i] = throttle;
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + motor[0]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + motor[1]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + motor[2]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + motor[3]);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
#endif
	vSetVarulMaxPRIGROUPValue();
	SEGGER_UART_init(1500000);
	SEGGER_SYSVIEW_Conf();
#ifdef RADIO_TASK
  status = xTaskCreate(radio_task, "radio_task", 200, NULL, 6, &radio_handler);
	configASSERT(status == pdPASS);
#endif
#ifdef SENSOR_TASK
  status = xTaskCreate(sensor_task, "sensor_task", 300, NULL, 4, &sensor_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef TOF_TASK
  status = xTaskCreate(tof_task, "tof_task", 400, NULL, 3, &tof_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef GPS_TASK
  status = xTaskCreate(gps_task, "gps_task", 200, NULL, 3, &gps_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef SD_TASK
  status = xTaskCreate(sd_task, "sd_task", 700, NULL, 3, &sd_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef ADC_TASK
	status = xTaskCreate(adc_task, "adc_task", 200, NULL, 4, &adc_handler);
	configASSERT(status == pdPASS);
#endif
#ifdef CTRL_TASK
	status = xTaskCreate(ctrl_task, "ctrl_task", 400, NULL, 2, &ctrl_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef MSG_TASK
	status = xTaskCreate(msg_task, "msg_task", 500, NULL, 1, &msg_handler);
	configASSERT(status == pdPASS);
#endif
	vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Interrupt Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin == NRF_IRQ_Pin) {
    nrf24l01p_rxtx((uint8_t *)&pl, (uint8_t *)&ack_pl, ACK_PAYLOAD_WIDTH);
    vTaskNotifyGiveFromISR(radio_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hi2c->Instance == hi2c2.Instance) {
    vTaskNotifyGiveFromISR(sensor_handler, &xHigherPriorityTaskWoken);
  } else if (hi2c->Instance == hi2c3.Instance) {
    vTaskNotifyGiveFromISR(tof_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hi2c->Instance == hi2c2.Instance) {
    vTaskNotifyGiveFromISR(sensor_handler, &xHigherPriorityTaskWoken);
  } else if (hi2c->Instance == hi2c3.Instance) {
    vTaskNotifyGiveFromISR(tof_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi->Instance == hspi1.Instance) {
    vTaskNotifyGiveFromISR(sd_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi->Instance == hspi1.Instance) {
    vTaskNotifyGiveFromISR(sd_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveFromISR(adc_handler, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART2_IRQHandler(void)
{
  traceISR_ENTER();

  if (USART2->SR & USART_SR_RXNE) {
    *gps_rx_buf_ptr = (uint8_t)USART2->DR;

    if (*gps_rx_buf_ptr == '\n') {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      gps_idle_buf_id = !gps_idle_buf_id;
      gps_rx_buf_ptr = gps_buffer[!gps_idle_buf_id];

      vTaskNotifyGiveFromISR(gps_handler, &xHigherPriorityTaskWoken);

      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
      gps_rx_buf_ptr++;
    }
  }
  traceISR_EXIT();
}

/* Task Function */
static void radio_task(void *param)
{
  uint8_t start_flag = 1;

  while (1) {
    if (start_flag) {
      nrf24l01p_start_rx();
      start_flag = 0;
    }
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    /* decode payload */
    if (pl.throttle < MAX_THROTTLE && pl.throttle >= 0)
      throttle = DECODE_PAYLOAD_THROTTLE(pl.throttle);
    att.roll_target = DECODE_PAYLOAD_RADIUS(pl.roll_target);
    att.pitch_target = DECODE_PAYLOAD_RADIUS(pl.pitch_target);
    // if (pl->yaw_target > 1.f || pl->yaw_target < -1.f)
    //   att->yaw_target += DECODE_PAYLOAD_RADIUS(pl.yaw_target);
    pos.height_target = DECODE_PAYLOAD_HEIGHT(pl.height_target);
    ctrl_param.P = DECODE_PAYLOAD_CTRL_GAIN(pl.P);
    ctrl_param.I = DECODE_PAYLOAD_CTRL_GAIN(pl.I);
    ctrl_param.D = DECODE_PAYLOAD_CTRL_GAIN(pl.D);
    ctrl_param.mode = DECODE_PAYLOAD_CTRL_MODE(pl.mode);
  }
}

void rec_data(void *data)
{
  uint32_t mark;
  uint32_t *tick_ptr;
  size_t size = 0, write_size;

  if ((rec_status & REC_STATUS_PROCESS_MASK) == REC_STATUS_PROCESS_UNDONE) {
#ifdef REC_ONE_CYCLE_MODE
    if (current_tick > pdMS_TO_TICKS(REC_CYCLE_MS)) {
      rec_status &= ~REC_STATUS_PROCESS_MASK;
      rec_status |= REC_STATUS_PROCESS_END;
      return;
    }
#endif
    mark = *((uint32_t *)data);
    tick_ptr = (uint32_t *)(data + 4);
    *tick_ptr = current_tick;

    switch (mark) {
    case REC_MARK_IMU:
      size = sizeof(struct rec_imu);
      break;
    case REC_MARK_MAG:
      size = sizeof(struct rec_mag);
      break;
    case REC_MARK_BARO:
      size = sizeof(struct rec_baro);
      break;
    case REC_MARK_TOF:
      size = sizeof(struct rec_tof);
      break;
    case REC_MARK_BATTERY:
      size = sizeof(struct rec_battery);
      break;
    case REC_MARK_ATT:
      size = sizeof(struct rec_att);
      break;
    case REC_MARK_CTRL:
      size = sizeof(struct rec_ctrl);
      break;
    }
    write_size = xStreamBufferSend(sd_buf_handler, data, size, 0);
    if (write_size < size) {
      rec_status &= ~REC_STATUS_PROCESS_MASK;
      rec_status |= REC_STATUS_WRITE_BUFFER_MISSING |
                    REC_STATUS_PROCESS_END;
    }
  }
}

static void sensor_task(void *param)
{
  TickType_t mag_tick = 0, baro_tick = 0;
  float dt = TIME_PER_TICK;
  float ax, ay, az, gx, gy, gz;
  float mx, my, mz, mag_square;
  float press, temp;
  uint8_t mag_err = 1;
  struct rec_imu imu_data = {
    REC_MARK_IMU, 0, 0, 0, 0, 0, 0, 0,
  };
  struct rec_mag mag_data = {
    REC_MARK_MAG, 0, 0, 0, 0
  };
  struct rec_baro baro_data = {
    REC_MARK_BARO, 0, 0, 0, 0
  };

  while (1) {
    current_tick = xTaskGetTickCount();
    if (!icm20948_read_axis6(&ax, &ay, &az, &gx, &gy, &gz)) {
      gx *= DEG2RAD;
      gy *= DEG2RAD;
      gz *= DEG2RAD;
      ENU2NED(gx, gy, gz);
      ENU2NED(ax, ay, az);     
      sensor.ax = ax;
      sensor.ay = ay;
      sensor.az = az;
      sensor.gx = gx;
      sensor.gy = gy;
      sensor.gz = gz;
#ifdef REC_IMU
      memcpy(&imu_data.ax, &sensor.ax, 24);
      rec_data(&imu_data);
#endif
    }
    if (mag_tick == pdMS_TO_TICKS(10)) {
      if (!ak09916_read_data(&mx, &my, &mz)) {
        mag_square = mx * mx + my * my + mz * mz;
        /* check if disturbed or not by square */
        /* 25 uT < norm(mag) < 65 uT */
        if (mag_square > 625.f && mag_square < 4225.f) {
          /* transform axis to attitude form */
          ENU2NED(mx, my, mz);
          sensor.mx = mx;
          sensor.my = my;
          sensor.mz = mz;
          mag_err = 0;
#ifdef REC_MAG
          memcpy(&mag_data.mx, &sensor.mx, 12);
          rec_data(&mag_data);
#endif
        } else {
          mag_err = 1;
        }
      } else {
        mag_err = 1;
      }
      mag_tick = 0;
    }
    if (baro_tick == pdMS_TO_TICKS(20)) {
      if (!(*lps22hb_read_data)(&press, &temp)) {
        sensor.pressure = press;
        sensor.temperature = temp;
        pos.height = (powf(1013.25 / press, 1 / 5.257) - 1.0) *
                     (temp + 273.15) / 0.0065;
#ifdef REC_BARO
        memcpy(&baro_data.press, &sensor.press, 8);
        baro_data.height = pos.height;
        rec_data(&baro_data);
#endif
      }
      baro_tick = 0;
    }
    mag_tick++;
    baro_tick++;
    /* Update attitude */
    if (!mag_err)
      ahrs_update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    else
      ahrs_update_imu(gx, gy, gz, ax, ay, az, dt);
    ahrs2euler(&att.roll, &att.pitch, &att.yaw);
    ahrs2quat(att.q);

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void tof_task(void *param)
{
  uint8_t is_ready;
  uint8_t range_status;
  uint16_t d;

  while (1) {
    VL53L1X_CheckForDataReady(VL53L1X_ADDR, &is_ready);

    if (is_ready) {
      is_ready = 0;
      VL53L1X_GetRangeStatus(VL53L1X_ADDR, &range_status);
      
      if (!range_status) {
        VL53L1X_GetDistance(VL53L1X_ADDR, &d);
        sensor.distance = d;
        pos.distance = (float)d * 0.001;
      }
      VL53L1X_ClearInterrupt(VL53L1X_ADDR);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void gps_task(void *param)
{
  uint8_t start_flag = 1;

  while (1) {
    if (start_flag) {
      gps_idle_buf_id = 0;
      gps_rx_buf_ptr = gps_buffer[!gps_idle_buf_id];
      __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
      start_flag = 0;
    }
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  }
}

static void sd_task(void *param)
{
  FATFS fs;
  FIL fil;
  FRESULT fres;
  char buffer[512];
  UINT size, rm;
  uint8_t process;

  while(1) {
    process = rec_status & REC_STATUS_PROCESS_MASK;
    if (process == REC_STATUS_PROCESS_START) {
      f_mount(&fs, "", 0);
      fres = f_open(&fil, "uav_rec.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

      if (fres == FR_OK) {
        sd_buf_handler = xStreamBufferCreate(SD_BUFFER_SIZE, SD_BUFFER_LEVEL);

        if (sd_buf_handler == NULL) {
          f_close(&fil);
          f_mount(NULL, "", 0);
          rec_status = REC_STATUS_PROCESS_IDLE |
                       REC_STATUS_BUFFER_CREATE_ERROR;
        } else {

          rec_status = REC_STATUS_PROCESS_UNDONE;
        }
      } else {
        rec_status = REC_STATUS_PROCESS_IDLE | REC_STATUS_FILESYSTEM_ERROR;
      }
    } else if (process == REC_STATUS_PROCESS_UNDONE) {
      size = xStreamBufferReceive(sd_buf_handler, buffer, 512,
                                  pdMS_TO_TICKS(100));
      f_write(&fil, buffer, size, &rm);

      if (size != rm) {
        rec_status &= ~REC_STATUS_PROCESS_MASK;
        rec_status |= REC_STATUS_PROCESS_END | REC_STATUS_WRITE_SD_ERROR;
      }
    } else if (process == REC_STATUS_PROCESS_END) {
      rec_status &= ~REC_STATUS_PROCESS_MASK;
      rec_status |= REC_STATUS_PROCESS_IDLE;
      vStreamBufferDelete(sd_buf_handler);
      f_close(&fil);
      f_mount(NULL, "", 0);
      SEGGER_SYSVIEW_Print("sd_close");
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

static void adc_task(void *param)
{
  uint16_t data[2];
  uint8_t start_flag = 1;

  while (1) {
    if (start_flag) {
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 2);
      start_flag = 0;
    }
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    /* voltage = adc / 4096 * 3.3 / 10k * 51k */
    bat.voltage = (float)data[0] * 0.0041088867f;
    /* current = ((adc / 4096 * 3.3) - 2.4677) / 0.0328 */
    bat.current = (float)data[1] * 0.00080566406f - 2.4677f;
    bat.current /= 0.0328f;
  }
}

struct PID_param {
  float P;
  float I;
  float D;
  float last_err;
  float int_err;
  float output;
};

void set_PID(struct PID_param *PID, float P, float I, float D)
{
  if (P >= 0 && P < 3)
    PID->P = P;
  if (I >= 0 && I < 3)
    PID->I = I;
  if (D >= 0 && D < 3)
    PID->D = D;
}

void PID_control(struct PID_param *PID, float err, float dt)
{
  float new_int_err = PID->int_err + err * dt;
  float deriv = (err - PID->last_err) / dt;

  if (fabsf(new_int_err) < MAX_THRUST * 0.3f) {
    PID->int_err = new_int_err;
  }
  PID->output = (PID->P * err) + (PID->I * PID->int_err) + (PID->D * deriv);
  PID->last_err = err;
}

uint16_t thrust2duty(float thrust)
{
  float duty;

  if (thrust < MIN_THRUST) /* limit min thrust */
    thrust = 0.35f;
  else if (thrust > MAX_THRUST) /* limit max thrust */
    thrust = 7.f;
  // duty = 762.99 * sqrtf(thrust) - 284.9;
  duty = (sqrtf(thrust * 3420.f) - 18) * 12.5f;

  return (uint16_t)duty;
}

static void ctrl_task(void *param)
{
  uint16_t thro_duty;
  float thro_thrust;
  float q[4];
  float sp_r, sp_p, sp_y;
  float err[3];
  float dt = 0.02;
  float P = 1.2f;
  float I = 0.1f;
  float D = 0.2f;
  uint8_t mode = NORMAL_MODE;
  struct PID_param PID_roll;
  struct PID_param PID_pitch;
  struct PID_param PID_yaw;
  /* M1 (CCW) M2 (CW)
   * M4 (CW)  M3 (CCW) */
  /* M1 (CCW) M3 (CW)
   * M2 (CW)  M4 (CCW) */
  while (1) {
    vTaskSuspendAll();
    thro_duty = throttle;
    memcpy(q, att.q, 4 * sizeof(float));
    sp_r = att.roll_target;
    sp_p = att.pitch_target;
    sp_y = att.yaw_target;
    P = ctrl_param.P;
    I = ctrl_param.I;
    D = ctrl_param.D;
    mode = ctrl_param.mode;
    xTaskResumeAll();

    if (thro_duty < 200) {
      motor[0] = thro_duty;
      motor[1] = thro_duty;
      motor[2] = thro_duty;
      motor[3] = thro_duty;
      PID_roll.last_err = 0.f;
      PID_pitch.last_err = 0.f;
      PID_yaw.last_err = 0.f;
      PID_roll.int_err = 0.f;
      PID_pitch.int_err = 0.f;
      PID_yaw.int_err = 0.f;
    } else {
      // thro_thrust = ((float)(thro_duty) + 284.9f) / 726.99f; /* throttle to thrust */
      // thro_thrust *= thro_thrust;
      thro_thrust = (float)thro_duty * 0.08f + 18.2f;
      thro_thrust *= thro_thrust * 0.0002924f;
      set_PID(&PID_roll, P, I, D);
      set_PID(&PID_pitch, P, I, D);
      set_PID(&PID_yaw, P, I, D);
      attitude_err(q, sp_r, sp_p, sp_y, err);
      PID_control(&PID_roll, err[0], dt);
      PID_control(&PID_pitch, err[1], dt);
      PID_control(&PID_yaw, err[2], dt);
      motor[0] = thrust2duty(thro_thrust + PID_roll.output + PID_pitch.output
                                        - PID_yaw.output);
      motor[3] = thrust2duty(thro_thrust + PID_roll.output - PID_pitch.output
                                        + PID_yaw.output);
      motor[1] = thrust2duty(thro_thrust - PID_roll.output + PID_pitch.output
                                        + PID_yaw.output);
      motor[2] = thrust2duty(thro_thrust - PID_roll.output - PID_pitch.output
                                        - PID_yaw.output);
    }
    /* arr range for ESC: 2000 - 3999 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + motor[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + motor[1]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + motor[2]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + motor[3]);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void msg_task(void *param)
{
  TickType_t current_tick, last_tick = 0, pass_tick = 0;
  BaseType_t radio_wm = 0, sensor_wm = 0, tof_wm = 0, gps_wm = 0;
  BaseType_t sd_wm = 0, adc_wm = 0, ctrl_wm = 0, msg_wm = 0;
  BaseType_t min_remaining, remaining;
  char msg[512];
  uint16_t size;
  uint8_t watch_rec_status;

	while (1) {
    current_tick = xTaskGetTickCount();
    pass_tick = current_tick - last_tick;
    last_tick = current_tick;
    /* encode ack payload */
    vTaskSuspendAll();
    ack_pl.throttle = ENCODE_PAYLOAD_THROTTLE(throttle);
    ack_pl.motor[0] = ENCODE_PAYLOAD_MOTOR(motor[0]);
    ack_pl.motor[1] = ENCODE_PAYLOAD_MOTOR(motor[1]);
    ack_pl.motor[2] = ENCODE_PAYLOAD_MOTOR(motor[2]);
    ack_pl.motor[3] = ENCODE_PAYLOAD_MOTOR(motor[3]);
    ack_pl.roll = ENCODE_PAYLOAD_RADIUS(att.roll);
    ack_pl.pitch = ENCODE_PAYLOAD_RADIUS(att.pitch);
    ack_pl.yaw = ENCODE_PAYLOAD_RADIUS(att.yaw);
    ack_pl.height = ENCODE_PAYLOAD_HEIGHT(pos.height);
    ack_pl.voltage = ENCODE_PAYLOAD_VOLTAGE(bat.voltage);
    watch_rec_status = rec_status;
    xTaskResumeAll();

    radio_wm = uxTaskGetStackHighWaterMark(radio_handler);
    sensor_wm = uxTaskGetStackHighWaterMark(sensor_handler);
    tof_wm = uxTaskGetStackHighWaterMark(tof_handler);
    gps_wm = uxTaskGetStackHighWaterMark(gps_handler);
    sd_wm = uxTaskGetStackHighWaterMark(sd_handler);
    adc_wm = uxTaskGetStackHighWaterMark(adc_handler);
    ctrl_wm = uxTaskGetStackHighWaterMark(ctrl_handler);
    msg_wm = uxTaskGetStackHighWaterMark(msg_handler);

    min_remaining = xPortGetMinimumEverFreeHeapSize();
    remaining = xPortGetFreeHeapSize();

		size = snprintf(msg, 512, "r: %.3f, p: %.3f, y: %.3f, "
                    "rsp: %.3f, psp: %.3f, ysp: %.3f\r\n"
                    "h: %.2f, d: %d, V: %.2f, I: %.2F\r\n"
                    "throttle: %d, m0: %d, m1: %d, m2: %d, m3: %d\r\n"
                    "P: %.2f, I: %.2f, D: %.2f, crash: %d\r\n"
                    "stack wm:\r\n"
                    "radio: %ld, sensor: %ld, tof: %ld, gps: %ld\r\n"
                    "sd: %ld, adc: %ld, ctrl: %ld, msg: %ld\r\n"
                    "min rm: %ld, cur rm: %ld, rec status: %d\r\n"
                    "pass tick: %ld\r\n"
                    "%s",
			              att.roll, att.pitch, att.yaw,
                    att.roll_target, att.pitch_target, att.yaw_target,
             		    pos.height, sensor.distance, bat.voltage, bat.current,
                    throttle, motor[0], motor[1], motor[2], motor[3],
                    ctrl_param.P, ctrl_param.I, ctrl_param.D, ack_pl.event,
                    radio_wm, sensor_wm, tof_wm, gps_wm,
                    sd_wm, adc_wm, ctrl_wm, msg_wm,
                    min_remaining, remaining, watch_rec_status, pass_tick,
                    gps_buffer[gps_idle_buf_id]);
    CDC_Transmit_FS((uint8_t *)msg, size);
    vTaskDelay(pdMS_TO_TICKS(200));
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM9) {
    uint16_t count = Timer1;

    if (count)
      Timer1 = --count;
    count = Timer2;
    if (count)
      Timer2 = --count;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return
	 * state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line
	   number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
	   file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
