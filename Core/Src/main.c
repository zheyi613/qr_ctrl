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
// #include "icm20948.h"
#include "ak09916.h"
#include "VL53L1X_api.h"
#include "nrf24l01p.h"
#include "nrf_payload.h"
#include "math.h"
#include "ahrs.h"
#include "diskio.h"
#include "sd_record.h"
#include "mpu9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Enable/Disable tasks */
#define RADIO_TASK
#define SENSOR_TASK
// #define TOF_TASK
#define GPS_TASK
#define SD_TASK
#define ADC_TASK
#define CTRL_TASK
#define MSG_TASK

/* ESC calibration mode */
#define ESC_CALIBRATION

/* Module initialize failed number */
enum module_init_failed_id {
  MODULE_FAILED_NRF24L01P,
  MODULE_FAILED_TOF,
  MODULE_FAILED_LPS22HB,
  MODULE_FAILED_MPU9250,
  MODULE_FAILED_AK8963
};

#define VL53L1X_ADDR        0x52

#define CTRL_FREQ           100
#define MOTOR_DUTY_RANGE    2000 /* esc standard: 1-2ms, pwm arr: 2000-3999 */
#define MOTOR_CTRL_RANGE    MOTOR_DUTY_RANGE * 0.2
#define MOTOR_INITIAL_DUTY  0  /* max: 1999, must more than zero to lock ESC */
#define MAX_THROTTLE        MOTOR_DUTY_RANGE * 0.9 /* max:  90% duty */

#define MIN_THRUST          0.35F
#define MAX_THRUST          7.0F

#define TIME_PER_TICK       0.002F

#define RAD2DEG             57.29577F
#define DEG2RAD             0.017453292F

/* GPS buffer size */
#define GPS_BUFFER_SIZE     128

/* SD card recording buffer size */
#define SD_BUFFER_SIZE      2048
#define SD_BUFFER_LEVEL     512

/* SD recording mode configuration */
// #define REC_ONE_CYCLE_MODE
#define REC_CYCLE_MS        10000
#define REC_THROTTLE_TRIGGER_MODE

/* Select SD recording data */
#define REC_IMU
#define REC_MAG
#define REC_BARO
// #define REC_TOF
// #define REC_BATTERY
#define REC_ATT
#define REC_CTRL

/* GPS header and address ID of NAV_SOL */
#define GPS_HEADER_1        0xB5 /* Sync char 1 */
#define GPS_HEADER_2        0x62 /* Sync char 2 */
#define GPS_NAV             0x01 /* Class */
#define GPS_NAV_SOL         0x06 /* ID */
/* GPS get symbol status */
#define GPS_RECEIVE_SYNC_1    0
#define GPS_RECEIVE_SYNC_2    1
#define GPS_RECEIVE_DATA      2

/* ADC moving average window size */
#define ADC_MA_WINDOW_SIZE    10
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
  float sp_roll;
  float sp_pitch;
  float sp_yaw;
} att;

struct position {
  float distance;
  float height;
  float sp_height;
} pos;

float world_linear_az;
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

float fault_ratio;
int motor_fault_id;

uint8_t motor_bias[4];

/* Variable of sd spi timer (ms) */
WORD Timer1, Timer2;

/* SD card recording status (don't need suspend task) */
uint8_t rec_status = REC_STATUS_PROCESS_INIT;

/* GPS buffer (transfer by double buffer) */
uint8_t gps_buffer[2][GPS_BUFFER_SIZE];
uint8_t gps_rx_buf_id;
uint8_t *gps_rx_ptr;
uint16_t gps_rx_cnt;
uint8_t gps_receive_status;
uint8_t gps_sv_status; /* 0: cksum not pass or svNum = 0, else: svNum */
uint16_t gps_data_length;
struct gps_nav_sol {
  uint32_t iTOW;
  int32_t fTOW;
  int16_t week;
  uint8_t gpsFix;
  int8_t flags;
  int32_t ecefX;
  int32_t ecefY;
  int32_t ecefZ;
  uint32_t pAcc;
  int32_t ecefVX;
  int32_t ecefVY;
  int32_t ecefVZ;
  uint32_t sAcc;
  uint16_t pDOP;
  uint8_t reserved1;
  uint8_t numSV;
  uint32_t reserved2;
};
struct gps_nav_sol *gps_nav_sol_data;
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
  case MODULE_FAILED_MPU9250:
    len = snprintf(msg, 100, "Initialize MPU9250 failed...\n");
    break;
  case MODULE_FAILED_AK8963:
    len = snprintf(msg, 100, "Initialize AK8963 failed...\n");
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
  HAL_TIM_Base_Start_IT(&htim9);

  /* initialize global variable */
  memset(&sensor, 0, sizeof(struct sensor_data));
  memset(&att, 0, sizeof(struct attitude));
  memset(&pos, 0, sizeof(struct position));
  memset(&bat, 0, sizeof(struct battery));
  memset(&pl, 0, PAYLOAD_WIDTH);
  memset(&ack_pl, 0, ACK_PAYLOAD_WIDTH);
  memset(&ctrl_param, 0, sizeof(struct ctrl_parameter));
  memset(gps_buffer, 0, 2 * GPS_BUFFER_SIZE);
  fault_ratio = 0;
  motor_fault_id = 0;
  memset(motor_bias, 0, sizeof(motor_bias));

	#ifdef ESC_CALIBRATION
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + 1999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + 1999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + 1999);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + 1999);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  
  HAL_Delay(3000);

  throttle = MOTOR_INITIAL_DUTY;
  for (uint8_t i = 0; i < 4; i++) {
    motor[i] = throttle;
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + motor[0]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + motor[1]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + motor[2]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + motor[3]);
  #endif
  /* Wait 1 sec to power up and mechanical vibration */
  HAL_Delay(3000);

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
  if (VL53L1X_SetTimingBudgetInMs(VL53L1X_ADDR, 33))
    module_init_failed(MODULE_FAILED_TOF);
  if (VL53L1X_SetInterMeasurementInMs(VL53L1X_ADDR, 50))
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

  status = mpu9250_init(1000, GYRO_FS_2000DPS, ACCEL_FS_16G,
                        GYRO_LP_99HZ, ACCEL_LP_92HZ);
  if (status == 1)
    module_init_failed(MODULE_FAILED_MPU9250);
  else if (status == 2)
    module_init_failed(MODULE_FAILED_AK8963);

  mpu9250_read_imu(&sensor.ax, &sensor.ay, &sensor.az,
                   &sensor.gx, &sensor.gy, &sensor.gz);
  mpu9250_read_mag(&sensor.mx, &sensor.my, &sensor.mz);
  ENU2NED(sensor.ax, sensor.ay, sensor.az);
  ENU2NED(sensor.mx, sensor.my, sensor.mz);
  ahrs_init(sensor.ax, sensor.ay, sensor.az,
            sensor.mx, sensor.my, sensor.mz);
  // ahrs_init_imu(sensor.ax, sensor.ay, sensor.az);
  ahrs2euler(&att.roll, &att.pitch, &att.yaw);
  att.sp_yaw = att.yaw;
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
#if (SEGGER_UART_REC == 1)
	SEGGER_UART_init(1500000);
	SEGGER_SYSVIEW_Conf();
#endif
#ifdef RADIO_TASK
  status = xTaskCreate(radio_task, "radio_task", 200, NULL, 4, &radio_handler);
	configASSERT(status == pdPASS);
#endif
#ifdef SENSOR_TASK
  status = xTaskCreate(sensor_task, "sensor_task", 500, NULL, 4, &sensor_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef TOF_TASK
  status = xTaskCreate(tof_task, "tof_task", 200, NULL, 3, &tof_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef GPS_TASK
  status = xTaskCreate(gps_task, "gps_task", 200, NULL, 3, &gps_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef SD_TASK
  status = xTaskCreate(sd_task, "sd_task", 600, NULL, 3, &sd_handler);
  configASSERT(status == pdPASS);
  sd_buf_handler = xStreamBufferCreate(SD_BUFFER_SIZE, SD_BUFFER_LEVEL);
  configASSERT(sd_buf_handler);
#endif
#ifdef ADC_TASK
	status = xTaskCreate(adc_task, "adc_task", 200, NULL, 3, &adc_handler);
	configASSERT(status == pdPASS);
#endif
#ifdef CTRL_TASK
	status = xTaskCreate(ctrl_task, "ctrl_task", 500, NULL, 2, &ctrl_handler);
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
  } else if (hspi->Instance == hspi2.Instance) {
    vTaskNotifyGiveFromISR(radio_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi->Instance == hspi1.Instance) {
    vTaskNotifyGiveFromISR(sd_handler, &xHigherPriorityTaskWoken);
  } else if (hspi->Instance == hspi2.Instance) {
    vTaskNotifyGiveFromISR(radio_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi->Instance == hspi2.Instance) {
    vTaskNotifyGiveFromISR(radio_handler, &xHigherPriorityTaskWoken);
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
    *gps_rx_ptr = (uint8_t)USART2->DR;

    if (gps_receive_status == GPS_RECEIVE_DATA) {
      gps_rx_cnt++;
      gps_rx_ptr++;

      if (gps_rx_cnt < 4) {
        /* Do nothing */
      } else if (gps_rx_cnt == 4) { /* Get data length */
        gps_data_length = *(gps_rx_ptr - 2);
        gps_data_length |= (uint16_t)(*(gps_rx_ptr - 1)) << 8;
      } else if (gps_rx_cnt == gps_data_length + 6) {
        /* Call GPS task after get CK_B */    
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        /* Swich to another gps buffer */
        gps_rx_buf_id = !gps_rx_buf_id;
        gps_rx_ptr = gps_buffer[gps_rx_buf_id];
        /* Initialize index and status */
        gps_rx_cnt = 0;
        gps_receive_status = GPS_RECEIVE_SYNC_1;

        vTaskNotifyGiveFromISR(gps_handler, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    } else if (gps_receive_status == GPS_RECEIVE_SYNC_1) {
      if (*gps_rx_ptr == GPS_HEADER_1)
        gps_receive_status = GPS_RECEIVE_SYNC_2;
    } else if (gps_receive_status == GPS_RECEIVE_SYNC_2) {
      gps_receive_status = (*gps_rx_ptr == GPS_HEADER_2) ?
                           GPS_RECEIVE_DATA : GPS_RECEIVE_SYNC_1;
    }
  }
  traceISR_EXIT();
}

/* Task Function */
static void radio_task(void *param)
{
  uint8_t start_flag = 1;
  uint8_t disconnect_count = 0;
  int32_t tmp;
  float yaw_input = 0.f;

  while (1) {
    if (start_flag) {
      nrf24l01p_start_rx();
      start_flag = 0;
    }
    /* If lost connected 1 sec, landing by minus 100 throttle every 500 ms */
    if (disconnect_count > 10) {
      ctrl_param.mode = NORMAL_MODE;
      tmp = (int32_t)throttle;
      tmp -= 100;
      if (tmp < 0)
        throttle = 0;
      else
        throttle = tmp;
      att.sp_roll = 0;
      att.sp_pitch = 0;
      vTaskDelay(pdMS_TO_TICKS(500));
    } else if (ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(100)) != 0) {
      vTaskDelay(pdMS_TO_TICKS(2));
      nrf24l01p_receive((uint8_t *)&pl);
      nrf24l01p_write_ack_payload((uint8_t *)&ack_pl, ACK_PAYLOAD_WIDTH);
      /* decode payload */
      if (pl.throttle < MAX_THROTTLE && pl.throttle >= 0)
        DECODE_PAYLOAD_THROTTLE(pl.throttle, throttle);
      if (throttle == 0)
        att.sp_yaw = att.yaw;
      DECODE_PAYLOAD_RADIUS(pl.sp_roll, att.sp_roll);
      DECODE_PAYLOAD_RADIUS(pl.sp_pitch, att.sp_pitch);
      /* Yaw input need to higher than 0.1 deg */
      if (pl.sp_yaw_rate > 18 || pl.sp_yaw_rate < -18) {
        DECODE_PAYLOAD_RADIUS(pl.sp_yaw_rate, yaw_input);
        att.sp_yaw += yaw_input;
      }
      DECODE_PAYLOAD_HEIGHT(pl.sp_height, pos.sp_height);
      DECODE_PAYLOAD_CTRL_PI_GAIN(pl.P, ctrl_param.P);
      DECODE_PAYLOAD_CTRL_PI_GAIN(pl.I, ctrl_param.I);
      DECODE_PAYLOAD_CTRL_D_GAIN(pl.D, ctrl_param.D);
      DECODE_PAYLOAD_MODE(pl.mode, ctrl_param.mode);
      DECODE_PAYLOAD_FAULT_RATIO(pl.fault_ratio, fault_ratio);
      disconnect_count = 0;
    } else {
      disconnect_count++;
    }
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
    vTaskSuspendAll();
    write_size = xStreamBufferSend(sd_buf_handler, data, size, 0);
    xTaskResumeAll();
    if (write_size < size) {
      rec_status &= ~REC_STATUS_PROCESS_MASK;
      rec_status |= REC_STATUS_WRITE_BUFFER_ERROR |
                    REC_STATUS_PROCESS_END;
    }
  }
}

/* x[order + 1], y[order + 1]
 * y[0] = x[0] * num[0] + x[1] * num[1] + ...
 *       -y[1] * den[1] - y[2] * den[2]
 */
void IIR_filter(int order, float *num, float *den, float *x, float *y)
{
  int i;

  for (i = order; i > 0; i--) {
    y[i] = y[i - 1];
  }
  y[0] = num[0] * x[0];
  for (i = 1; i <= order; i++) {
    y[0] += num[i] * x[i] - den[i] * y[i];
  }
  for (i = order; i > 0; i--) {
    x[i] = x[i - 1];
  }
}

float movavg(float *data, float val, uint8_t id, int size)
{
  float total = 0;

  data[id] = val;

  for (int i = 0; i < size; i++) {
    total += data[i];
  }
  return total / size;
}

static void sensor_task(void *param)
{
  TickType_t mag_tick = 0, baro_tick = 0;
  float dt = TIME_PER_TICK;
  float ax, ay, az, gx, gy, gz;
  float mx, my, mz, mag_square;
  float press, temp;
  uint8_t mag_ready = 0;
  uint8_t imu_ma_id = {0};
  uint8_t sp_ma_id = {0};
  float ma_ax[5] = {0}, ma_ay[5] = {0}, ma_az[5] = {0};
  float ma_gx[5] = {0}, ma_gy[5] = {0}, ma_gz[5] = {0};
  float ma_spr[10] = {0}, ma_spp[10] = {0}, ma_spy[10] = {0};
  struct rec_imu imu_data = {
    REC_MARK_IMU, 0, 0, 0, 0, 0, 0, 0, 0,
  };
  struct rec_mag mag_data = {
    REC_MARK_MAG, 0, 0, 0, 0
  };
  struct rec_baro baro_data = {
    REC_MARK_BARO, 0, 0, 0, 0
  };
  struct rec_att att_data = {
    REC_MARK_ATT, 0, 0, 0, 0, 0, 0
  };

  while (1) {
    current_tick = xTaskGetTickCount();
    if (!mpu9250_read_imu(&ax, &ay, &az, &gx, &gy, &gz)) {
      ENU2NED(gx, gy, gz);
      ENU2NED(ax, ay, az);
      gx *= DEG2RAD;
      gy *= DEG2RAD;
      gz *= DEG2RAD;
      ax = movavg(ma_ax, ax, imu_ma_id, 5);
      ay = movavg(ma_ay, ay, imu_ma_id, 5);
      az = movavg(ma_az, az, imu_ma_id, 5);
      sensor.ax = ax;
      sensor.ay = ay;
      sensor.az = az;
      gx = movavg(ma_gx, gx, imu_ma_id, 5);
      gy = movavg(ma_gy, gy, imu_ma_id, 5);
      gz = movavg(ma_gz, gz, imu_ma_id, 5);
      sensor.gx = gx;
      sensor.gy = gy;
      sensor.gz = gz;
      if ((++imu_ma_id) == 5)
        imu_ma_id = 0;
      world_linear_az = ahrs_world_linear_az(ax, ay, az);
#ifdef REC_IMU
      memcpy(&imu_data.ax, &sensor.ax, 24);
      imu_data.world_linear_az = world_linear_az;
      rec_data(&imu_data);
#endif
    }
    mag_ready = 0;
    if (mag_tick == pdMS_TO_TICKS(10)) {
      if (!mpu9250_read_mag(&mx, &my, &mz)) {
        mag_square = mx * mx + my * my + mz * mz;
        /* check if disturbed or not by square */
        /* 25 uT < norm(mag) < 70 uT */
        if ((mag_square > 625.f) && (mag_square < 4900.f)) {
          ENU2NED(mx, my, mz);
          sensor.mx = mx;
          sensor.my = my;
          sensor.mz = mz;
#ifdef REC_MAG
          memcpy(&mag_data.mx, &sensor.mx, 12);
          rec_data(&mag_data);
#endif
          mag_ready = 1;
        }
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
        memcpy(&baro_data.press, &sensor.pressure, 8);
        baro_data.height = pos.height;
        rec_data(&baro_data);
#endif
      }
      baro_tick = 0;
    }
    mag_tick++;
    baro_tick++;
    /* Update attitude */
    if (mag_ready)
      ahrs_update_marg(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    else
      ahrs_update_imu(gx, gy, gz, ax, ay, az, dt);
    ahrs2euler(&att.roll, &att.pitch, &att.yaw);
    ahrs2quat(att.q);
    att.sp_roll = movavg(ma_spr, att.sp_roll, sp_ma_id, 10);
    att.sp_pitch = movavg(ma_spp, att.sp_pitch, sp_ma_id, 10);
    att.sp_yaw = movavg(ma_spy, att.sp_yaw, sp_ma_id, 10);
    if ((++sp_ma_id) == 10)
      sp_ma_id = 0;
#ifdef REC_ATT
    memcpy(&att_data.roll, &att.roll, 24);
    rec_data(&att_data);
#endif
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void tof_task(void *param)
{
  uint8_t is_ready = 0;
  uint8_t range_status;
  uint16_t d;
  struct rec_tof tof_data = {
    REC_MARK_TOF, 0, 0
  };

  while (1) {
    VL53L1X_CheckForDataReady(VL53L1X_ADDR, &is_ready);

    if (is_ready) {
      is_ready = 0;
      VL53L1X_GetRangeStatus(VL53L1X_ADDR, &range_status);
      
      if (!range_status) {
        VL53L1X_GetDistance(VL53L1X_ADDR, &d);
        sensor.distance = d;
        pos.distance = (float)d * 0.001;
#ifdef REC_TOF
        tof_data.distance = (uint32_t)d;
        rec_data(&tof_data);
#endif
      }
      VL53L1X_ClearInterrupt(VL53L1X_ADDR);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void gps_task(void *param)
{
  uint8_t *idle_buf_ptr;
  uint8_t CK_A, CK_B;
  uint16_t i, length;
  uint8_t start_flag = 1;

  while (1) {
    if (start_flag) {
      gps_rx_buf_id = 0;
      gps_rx_ptr = gps_buffer[gps_rx_buf_id];
      idle_buf_ptr = gps_buffer[!gps_rx_buf_id];
      gps_nav_sol_data = (struct gps_nav_sol *)(idle_buf_ptr + 4);
      gps_rx_cnt = 0;
      gps_receive_status = GPS_RECEIVE_SYNC_1;
      gps_sv_status = 0;
      __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
      start_flag = 0;
    }
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    idle_buf_ptr = gps_buffer[!gps_rx_buf_id];
    gps_nav_sol_data = (struct gps_nav_sol *)(idle_buf_ptr + 4);
    length = *(idle_buf_ptr + 2) | ((uint16_t)(*(idle_buf_ptr + 3)) << 8);
    CK_A = 0;
    CK_B = 0;
    /* Calculate GPS checksum */
    for (i = 0; i < (length + 4); i++) {
      CK_A += *idle_buf_ptr++;
      CK_B += CK_A;
    }
    if ((*idle_buf_ptr == CK_A) && (*(idle_buf_ptr + 1) == CK_B))
      gps_sv_status = gps_nav_sol_data->numSV;
    else
      gps_sv_status = 0;
  }
}

static void sd_task(void *param)
{
  FATFS fs;
  FIL fil;
  FRESULT fres;
  char buffer[512];
  UINT size, rm;
  BaseType_t buf_reset_status = 0;
  DSTATUS init_status;
  uint8_t status;
  uint8_t process;

  while(1) {
    process = rec_status & REC_STATUS_PROCESS_MASK;
    if (process == REC_STATUS_PROCESS_UNDONE) {
      size = xStreamBufferReceive(sd_buf_handler, buffer, 512,
                                  pdMS_TO_TICKS(100));
      f_write(&fil, buffer, size, &rm);

      if (size != rm) {
        rec_status &= ~REC_STATUS_PROCESS_MASK;
        rec_status |= REC_STATUS_PROCESS_END | REC_STATUS_WRITE_SD_ERROR;
      }
    } else if (process == REC_STATUS_PROCESS_INIT) {
      /* Try to initialize SD card first */
      init_status = disk_initialize(0);

      if (init_status == 0) {
        #if defined(REC_ONE_CYCLE_MODE) 
          rec_status = REC_STATUS_PROCESS_START;
        #elif defined(REC_THROTTLE_TRIGGER_MODE)
          rec_status = REC_STATUS_PROCESS_IDLE;
        #endif
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else if (process == REC_STATUS_PROCESS_IDLE) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else if (process == REC_STATUS_PROCESS_START) {
      buf_reset_status = xStreamBufferReset(sd_buf_handler);
      status = rec_status;

      if (buf_reset_status == pdPASS) {
        f_mount(&fs, "", 0);
        fres = f_open(&fil, "uav_rec.txt", FA_CREATE_ALWAYS | FA_WRITE);

        if (fres == FR_OK) {
          status &= ~REC_STATUS_FILESYSTEM_ERROR;
          status = REC_STATUS_PROCESS_UNDONE;
        } else {
          status = REC_STATUS_PROCESS_IDLE | REC_STATUS_FILESYSTEM_ERROR;
        }
        
        status &= ~REC_STATUS_BUFFER_RESET_ERROR;
      } else {
        status |= REC_STATUS_PROCESS_IDLE | REC_STATUS_BUFFER_RESET_ERROR;
      }
      /* Delay 100 ms to wait send/receive buffer completed */
      vTaskDelay(pdMS_TO_TICKS(100));
      rec_status = status;
    } else if (process == REC_STATUS_PROCESS_END) {
      rec_status &= ~REC_STATUS_PROCESS_MASK;
      rec_status |= REC_STATUS_PROCESS_IDLE;
      f_close(&fil);
      f_mount(NULL, "", 0);
      SEGGER_SYSVIEW_Print("sd_close");
    }
  }
}

static void adc_task(void *param)
{
  uint16_t data[2];
  uint8_t start_flag = 1;
  uint8_t id = 0, i;
  float movavg_data[ADC_MA_WINDOW_SIZE] = {0};
  float total;
  struct rec_battery bat_data = {
    REC_MARK_BATTERY, 0, 0, 0
  };

  while (1) {
    if (start_flag) {
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 2);
      start_flag = 0;
    }
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    /* Moving Average (10 samples) */
    /* Voltage = adc / 4096 * 3.3 / 10k * (10k + 51k) */
    movavg_data[id] = (float)data[0] * 0.0049145507f;
    total = 0;
    for (i = 0; i < ADC_MA_WINDOW_SIZE; i++) {
      total += movavg_data[i];
    }
    if ((++id) >= ADC_MA_WINDOW_SIZE)
      id = 0;
    bat.voltage = total / ADC_MA_WINDOW_SIZE;
    /* Original current = ((adc / 4096 * 3.3) - 2.4677) / 0.0328
     * Vcc: 5V, Current direction: negative, Range: 0 ~ 2.5V
     * Current = (2.4677 - (adc / 4096 * 3.3)) / 0.0328 */
    bat.current = 2.4677f - (float)data[1] * 0.00080566406f;
    bat.current /= 0.0328f;
#ifdef REC_BATTERY
    bat_data.voltage = bat.voltage;
    bat_data.current = bat.current;
    rec_data(&bat_data);
#endif
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
  if (P >= 0 && P < 6)
    PID->P = P;
  if (I >= 0 && I < 6)
    PID->I = I;
  if (D >= 0 && D < 255)
    PID->D = D;
}

void PID_control(struct PID_param *PID, float err, float freq)
{
  float new_int_err = PID->int_err + err;
  float int_output = PID->I * new_int_err / freq;

  if (fabsf(int_output) < MAX_THRUST * 0.3f) {
    PID->int_err = new_int_err;
  }
  PID->output = (PID->P * err) + int_output + PID->D * (err - PID->last_err);
  PID->last_err = err;
}

uint16_t thrust2duty(float thrust)
{
  float duty;

  if (thrust < MIN_THRUST) /* limit min thrust */
    thrust = 0.35f;
  else if (thrust > MAX_THRUST) /* limit max thrust */
    thrust = 7.f;
  /* duty = (sqrtf(thrust * 3420) - 18) * 12.5 */
  duty = (sqrtf(thrust) * 58.480766f - 18) * 12.5f;

  return (uint16_t)duty;
}

/**
 * @brief normalize output if saturation is occured
 * 
 * @param thro_thrust input throttle
 * @param offset fault motor offset
 * @param abs_output absolute output of every err (ex: |r_out| + |p_out|)
 * @return float saturation ratio
 */
float saturated_ctrl(float thro_thrust, float offset, float abs_output)
{
  float upper_range, lower_range;
  float upper_saturation_ratio, lower_saturation_ratio;
  float saturation_ratio;

  saturation_ratio = 1;
  upper_range = MAX_THRUST - thro_thrust - offset;
  lower_range = thro_thrust - offset - MIN_THRUST;
  upper_saturation_ratio = abs_output / upper_range;
  lower_saturation_ratio = abs_output / lower_range;

  if ((upper_range > 0.f) && 
      (lower_range < (thro_thrust - MIN_THRUST) * 0.4f) &&
      (offset > 0.f)) {
    if (abs_output > upper_range)
      saturation_ratio = upper_saturation_ratio;
  } else if ((upper_range > 0.f) && (lower_range > 0.f)) {
    if ((abs_output > upper_range) && (abs_output > lower_range)) {
      if (upper_saturation_ratio > lower_saturation_ratio)
        saturation_ratio = upper_saturation_ratio;
      else
        saturation_ratio = lower_saturation_ratio;
    } else if (abs_output > upper_range) {
      saturation_ratio = upper_saturation_ratio;
    } else if (abs_output > lower_range) {
      saturation_ratio = lower_saturation_ratio;
    }
  } else {
    saturation_ratio = 1000.f; /* if range = 0, force ctrl output ~ 0 */
  }
  return saturation_ratio;
}

/**
 * @brief check which motor fault
 * 
 * @param roll_err roll error
 * @param pitch_err pitch error
 * @param yaw_err yaw error
 * @param yaw_gain yaw gain
 * @param threshold 
 * @return int 0: no failure / 1-4: motor(1-4) fault
 */
int check_fault(float roll_err, float pitch_err, float yaw_err,
                float threshold)
{
  float abs_r_err = fabsf(roll_err);
  float abs_p_err = fabsf(pitch_err);
  float abs_y_err = fabsf(yaw_err);
  float total_err;

  total_err = abs_r_err + abs_p_err + abs_y_err;
  if (total_err > threshold) {
    if ((roll_err > 0) && (pitch_err > 0))
      return 1;
    else if ((roll_err < 0) && (pitch_err > 0))
      return 2;
    else if ((roll_err < 0) && (pitch_err < 0))
      return 3;
    else if ((roll_err > 0) && (pitch_err < 0))
      return 4;
  }
  return 0;
}

static void ctrl_task(void *param)
{
  uint16_t thro_duty;
  float thro_thrust;
  float rp_output[4];
  float q[4];
  float sp_r, sp_p, sp_y;
  float err[3];
  float rp_err;
  float P1 = 0.5f;
  float I1 = 0.05f;
  float D1 = 20.f;
  struct PID_param PID_roll;
  struct PID_param PID_pitch;
  struct PID_param PID_yaw;
  struct PID_param PID2_roll;
  struct PID_param PID2_pitch;
  struct PID_param PID2_yaw;
  float last_err[3] = {0}, verr[3] = {0};
  float abs_output;
  float saturation_ratio;
  float fault_offset = 0, tmp_rp_output;
  float upper_offset_limit, lower_offset_limit, max_offset;
  uint8_t diag_motor_id[] = {2, 3, 0, 1};
  uint8_t mode = NORMAL_MODE;
  struct rec_ctrl ctrl_data = {
    REC_MARK_CTRL, 0, 0, {0}, 0, 0, 0, 0, 0
  };
  /* M1 (CCW) M2 (CW)
   * M4 (CW)  M3 (CCW) */
  while (1) {
    vTaskSuspendAll();
    thro_duty = throttle;
    memcpy(q, att.q, 4 * sizeof(float));
    sp_r = att.sp_roll;
    sp_p = att.sp_pitch;
    sp_y = att.sp_yaw;
    P1 = ctrl_param.P;
    I1 = ctrl_param.I;
    D1 = ctrl_param.D;
    mode = ctrl_param.mode;
    xTaskResumeAll();

    if (thro_duty < 400) {
#ifdef REC_THROTTLE_TRIGGER_MODE
      if ((rec_status & REC_STATUS_PROCESS_MASK) == 
           REC_STATUS_PROCESS_UNDONE) {
        rec_status &= ~REC_STATUS_PROCESS_MASK;
        rec_status |= REC_STATUS_PROCESS_END;
      }
#endif
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
      PID2_roll.last_err = 0.f;
      PID2_pitch.last_err = 0.f;
      PID2_yaw.last_err = 0.f;
      PID2_roll.int_err = 0.f;
      PID2_pitch.int_err = 0.f;
      PID2_yaw.int_err = 0.f;
      memset(last_err, 0, sizeof(last_err));
      memset(verr, 0, sizeof(verr));
      motor_fault_id = 0;
      fault_offset = 0.f;
      tmp_rp_output = 0.f;
    } else {
#ifdef REC_THROTTLE_TRIGGER_MODE
      if ((rec_status & REC_STATUS_PROCESS_MASK) == REC_STATUS_PROCESS_IDLE) {
        rec_status &= ~REC_STATUS_PROCESS_MASK;
        rec_status |= REC_STATUS_PROCESS_START;
      }
#endif
      thro_thrust = (float)thro_duty * 0.08f + 18.2f;
      thro_thrust *= thro_thrust * 0.0002924f;
      attitude_err(q, sp_r, sp_p, sp_y, err);
      /* PID loop */
      set_PID(&PID_roll, P1, I1, 0);
      set_PID(&PID_pitch, P1, I1, 0);
      set_PID(&PID_yaw, 3.f, 0, 0);
      PID_control(&PID_roll, err[0], CTRL_FREQ);
      PID_control(&PID_pitch, err[1], CTRL_FREQ);
      PID_control(&PID_yaw, err[2], CTRL_FREQ);
      verr[0] = (err[0] - last_err[0]) * CTRL_FREQ;
      last_err[0] = err[0];
      verr[1] = (err[1] - last_err[1]) * CTRL_FREQ;
      last_err[1] = err[1];
      verr[2] = (err[2] - last_err[2]) * CTRL_FREQ;
      last_err[2] = err[2];
      set_PID(&PID2_roll, 0.3f, 0.1f, 0);
      set_PID(&PID2_pitch, 0.3f, 0.1f, 0);
      set_PID(&PID2_yaw, 0.5f, 0.1f, 0);
      PID_control(&PID2_roll, PID_roll.output + verr[0], CTRL_FREQ);
      PID_control(&PID2_pitch, PID_pitch.output + verr[1], CTRL_FREQ);
      PID_control(&PID2_yaw, PID_yaw.output + verr[2], CTRL_FREQ);
      /* Prevent saturation */
      abs_output = fabsf(PID2_roll.output) + fabsf(PID2_pitch.output);
      if (motor_fault_id == 0) {
        abs_output += fabsf(PID_yaw.output);
        saturation_ratio = saturated_ctrl(thro_thrust, 0, abs_output);
        PID2_roll.output /= saturation_ratio;
        PID2_pitch.output /= saturation_ratio;
        PID2_yaw.output /= saturation_ratio;
      } else {
        saturation_ratio = saturated_ctrl(thro_thrust, fault_offset,
                                          abs_output);
        PID2_roll.output /= saturation_ratio;
        PID2_pitch.output /= saturation_ratio;
      }
      /* Compute roll/pitch output of 4 motors */
      rp_output[0] = PID2_roll.output + PID2_pitch.output;
      rp_output[1] = -PID2_roll.output + PID2_pitch.output;
      rp_output[2] = -PID2_roll.output - PID2_pitch.output;
      rp_output[3] = PID2_roll.output - PID2_pitch.output;
      
      if (mode == NORMAL_MODE) {
        motor[0] = thrust2duty(thro_thrust + rp_output[0] - PID2_yaw.output);
        motor[1] = thrust2duty(thro_thrust + rp_output[1] + PID2_yaw.output);
        motor[2] = thrust2duty(thro_thrust + rp_output[2] - PID2_yaw.output);
        motor[3] = thrust2duty(thro_thrust + rp_output[3] + PID2_yaw.output);
        
        motor_fault_id = 0;
        fault_offset = 0;
        tmp_rp_output = 0;
      } else if (mode == TEST_FAULT_MODE) {
        if (motor_fault_id == 0) {
          motor_fault_id = check_fault(err[0], err[1], err[2], 0.52);
          motor[0] = thrust2duty(thro_thrust + rp_output[0] - PID2_yaw.output);
          motor[1] = thrust2duty(thro_thrust + rp_output[1] + PID2_yaw.output);
          motor[2] = thrust2duty(thro_thrust + rp_output[2] - PID2_yaw.output);
          motor[3] = thrust2duty(thro_thrust + rp_output[3] + PID2_yaw.output);
        } else {
          rp_err = sqrtf((err[0] * err[0]) + (err[1] * err[1]));
          tmp_rp_output = -rp_output[diag_motor_id[motor_fault_id - 1]];
          /* < ~5.7 deg (0.1 rad) */
          if ((rp_err > 0.1f) && (tmp_rp_output > fault_offset)) {
            upper_offset_limit = MAX_THRUST - thro_thrust;
            lower_offset_limit = thro_thrust - MIN_THRUST;
            if (upper_offset_limit > lower_offset_limit)
              max_offset = lower_offset_limit;
            else
              max_offset = upper_offset_limit;
            if (tmp_rp_output < max_offset)
              fault_offset = tmp_rp_output;
            else
              fault_offset = max_offset;
          }
          if ((motor_fault_id == 1) || (motor_fault_id == 3)) {
            motor[0] = thrust2duty(thro_thrust - fault_offset + rp_output[0]);
            motor[1] = thrust2duty(thro_thrust + fault_offset + rp_output[1]);
            motor[2] = thrust2duty(thro_thrust - fault_offset + rp_output[2]);
            motor[3] = thrust2duty(thro_thrust + fault_offset + rp_output[3]);
          } else if ((motor_fault_id == 2) || (motor_fault_id == 4)) {
            motor[0] = thrust2duty(thro_thrust + fault_offset + rp_output[0]);
            motor[1] = thrust2duty(thro_thrust - fault_offset + rp_output[1]);
            motor[2] = thrust2duty(thro_thrust + fault_offset + rp_output[2]);
            motor[3] = thrust2duty(thro_thrust - fault_offset + rp_output[3]);
          }
        }
        if (motor[0] > thrust2duty(thro_thrust * (1 - fault_ratio)))
          motor[0] = thrust2duty(thro_thrust * (1 - fault_ratio));
      }
    }
#ifdef REC_CTRL
    ctrl_data.throttle = (uint32_t)thro_duty;
    memcpy(ctrl_data.motor, motor, sizeof(motor));
    memcpy(&ctrl_data.ex, err, sizeof(err));
    ctrl_data.offset = fault_offset;
    ctrl_data.fault_motor = (uint32_t)motor_fault_id;
    rec_data(&ctrl_data);
#endif
    /* arr range for ESC: 2000 - 3999 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_DUTY_RANGE + motor[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_DUTY_RANGE + motor[1]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_DUTY_RANGE + motor[2]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_DUTY_RANGE + motor[3]);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void msg_task(void *param)
{
  BaseType_t radio_wm = 0, sensor_wm = 0, tof_wm = 0, gps_wm = 0;
  BaseType_t sd_wm = 0, adc_wm = 0, ctrl_wm = 0, msg_wm = 0;
  BaseType_t min_remaining, remaining;
  char msg[512];
  uint16_t size;
  float r, p, y;

	while (1) {
    /* encode ack payload */
    vTaskSuspendAll();
    ENCODE_PAYLOAD_THROTTLE(throttle, ack_pl.throttle);
    ENCODE_PAYLOAD_MOTOR(motor[0], ack_pl.motor[0]);
    ENCODE_PAYLOAD_MOTOR(motor[1], ack_pl.motor[1]);
    ENCODE_PAYLOAD_MOTOR(motor[2], ack_pl.motor[2]);
    ENCODE_PAYLOAD_MOTOR(motor[3], ack_pl.motor[3]);
    ENCODE_PAYLOAD_RADIUS(att.roll, ack_pl.roll);
    ENCODE_PAYLOAD_RADIUS(att.pitch, ack_pl.pitch);
    ENCODE_PAYLOAD_RADIUS(att.yaw, ack_pl.yaw);
    ENCODE_PAYLOAD_HEIGHT(pos.height, ack_pl.height);
    ENCODE_PAYLOAD_VOLTAGE(bat.voltage, ack_pl.voltage);
    ENCODE_PAYLOAD_REC_STATUS(rec_status, ack_pl.rec_status);
    ENCODE_PAYLOAD_GPS_SV_STATUS(gps_sv_status, ack_pl.gps_sv_status);
    ENCODE_PAYLOAD_GPS_PACC(gps_nav_sol_data->pAcc, ack_pl.gps_pAcc);
    r = att.roll * RAD2DEG;
    p = att.pitch * RAD2DEG;
    y = att.yaw * RAD2DEG;
    xTaskResumeAll();

    radio_wm = uxTaskGetStackHighWaterMark(radio_handler);
    sensor_wm = uxTaskGetStackHighWaterMark(sensor_handler);
    // tof_wm = uxTaskGetStackHighWaterMark(tof_handler);
    gps_wm = uxTaskGetStackHighWaterMark(gps_handler);
    sd_wm = uxTaskGetStackHighWaterMark(sd_handler);
    adc_wm = uxTaskGetStackHighWaterMark(adc_handler);
    ctrl_wm = uxTaskGetStackHighWaterMark(ctrl_handler);
    msg_wm = uxTaskGetStackHighWaterMark(msg_handler);

    min_remaining = xPortGetMinimumEverFreeHeapSize();
    remaining = xPortGetFreeHeapSize();

		size = snprintf(msg, 512, "r: %.3f, p: %.3f, y: %.3f, "
                    "rsp: %.3f, psp: %.3f, ysp: %.3f\r\n"
                    "hsp: %.2f, h: %.2f, d: %d, V: %.2f, I: %.2f\r\n"
                    "throttle: %d, m0: %d, m1: %d, m2: %d, m3: %d\r\n"
                    "P1: %.2f, I1: %.2f, D1: %.2f\r\n"
                    "mode: %d, fault ratio: %.1f, fault id: %d\r\n"
                    "stack wm:\r\n"
                    "radio: %ld, sensor: %ld, tof: %ld, gps: %ld\r\n"
                    "sd: %ld, adc: %ld, ctrl: %ld, msg: %ld\r\n"
                    "min rm: %ld, cur rm: %ld, rec status: %d\r\n"
                    "current tick: %ld\r\n"
                    "gps:\r\n"
                    "iTOW: %ld, gpsFix: %d\r\n"
                    "ecefX: %ld, ecefY: %ld, ecefZ: %ld\r\n"
                    "pAcc: %ld, numSV: %d\r\n",
			              att.roll, att.pitch, att.yaw,
                    att.sp_roll, att.sp_pitch, att.sp_yaw,
             		    pos.sp_height, pos.height, sensor.distance,
                    bat.voltage, bat.current,
                    throttle, motor[0], motor[1], motor[2], motor[3],
                    ctrl_param.P, ctrl_param.I, ctrl_param.D,
                    ctrl_param.mode, fault_ratio, motor_fault_id,
                    radio_wm, sensor_wm, tof_wm, gps_wm,
                    sd_wm, adc_wm, ctrl_wm, msg_wm,
                    min_remaining, remaining, rec_status, current_tick,
                    gps_nav_sol_data->iTOW, gps_nav_sol_data->gpsFix,
                    gps_nav_sol_data->ecefX, gps_nav_sol_data->ecefY,
                    gps_nav_sol_data->ecefZ, gps_nav_sol_data->pAcc,
                    gps_nav_sol_data->numSV);
    // size = snprintf(msg, 512, "0.0,0.0,0.0,0.0,0.0,0.0,%.2f,%.2f,%.2f\n", r, p, y);
    CDC_Transmit_FS((uint8_t *)msg, size);
    vTaskDelay(pdMS_TO_TICKS(20));
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
