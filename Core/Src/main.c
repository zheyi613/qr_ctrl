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
#include "semphr.h"
#include "rtos_bus.h"
#include "lps22hb.h"
#include "icm20948.h"
#include "ak09916.h"
#include "nrf24l01p.h"
#include "vl53l0x.h"
#include "math.h"
#include "ahrs.h"
#include "diskio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_MAX_LEN        400

#define PAYLOAD_WIDTH      32
#define ACK_PAYLOAD_WIDTH  20

#define MOTOR_DUTY_RANGE   2000 /* esc standard: 1-2ms, pwm arr: 2000-3999 */
#define MOTOR_CTRL_RANGE   MOTOR_DUTY_RANGE * 0.2
#define MOTOR_INITIAL_DUTY 0  /* max: 1999, must more than zero to lock ESC */
#define MAX_THROTTLE       MOTOR_DUTY_RANGE * 0.9 /* max:  90% duty */

#define MIN_THRUST         0.35F
#define MAX_THRUST         7.0F

enum {
  NORMAL_MODE,
  ALTITUDE_MODE
};

#define TIME_PER_TICK      0.002F

#define RAD2DEG            57.29577F
#define DEG2RAD            0.017453292F

#define SD_BUFFER_SIZE  512
// #define ESC_CALIBRATION

/* Configure tasks to enable */
// #define RADIO_TASK
#define SENSOR_TASK
// #define TOF_TASK
#define SD_TASK
#define ADC_TASK
#define CTRL_TASK
#define MSG_TASK

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t radio_handler;
TaskHandle_t sensor_handler;
TaskHandle_t tof_handler;
TaskHandle_t sd_handler;
TaskHandle_t adc_handler;
TaskHandle_t ctrl_handler;
TaskHandle_t msg_handler;

uint8_t msg[MSG_MAX_LEN];
uint16_t len;

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
} sensor_data;

struct attitude {
  float q[4];
  float roll;
  float pitch;
  float yaw;
  float roll_target;
  float pitch_target;
  float yaw_target;
} attitude;

struct position {
  float distance;
  float height;
  float height_target;
} position;

static float battery_voltage = 0;

union {
  struct payload {
    uint16_t throttle;
    int16_t roll_target; /* +-180deg/32768 LSB */
    int16_t pitch_target;
    int16_t yaw_target;
    uint16_t height_target; /* 1mm LSB */
    uint8_t P;           /* 0.05 LSB */
    uint8_t I;
    uint8_t D;
    uint8_t mode;
    uint8_t unlock;
    uint8_t dummy[17];
  } data;
  uint8_t bytes[PAYLOAD_WIDTH];
} payload;
union {
  struct ack_payload {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t throttle;
    uint16_t motor[4];
    int16_t height;    /* 0.01 m LSB, range: +-327.68 m */
    uint8_t voltage;   /* 0.1 V LSB, range: 0 ~ 17.5 V */
    uint8_t event;
  } data;
  uint8_t bytes[ACK_PAYLOAD_WIDTH];
} ack_payload;

uint16_t throttle;
uint16_t motor[4];

struct ctrl_param {
  float P;
  float I;
  float D;
  uint8_t mode;
} ctrl_param;

/* variable of sd spi timer (ms) */
WORD Timer1, Timer2;
char *sd_idle_buf_ptr;
char sd_buffer[2][SD_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void radio_task(void *param);
static void sensor_task(void *param);
static void tof_task(void *param);
static void sd_task(void *param);
static void adc_task(void *param);
static void ctrl_task(void *param);
static void msg_task(void *param);

void enu2ned(float *x, float *y, float *z);

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
  memset(payload.bytes, 0, PAYLOAD_WIDTH);
  memset(ack_payload.bytes, 0, ACK_PAYLOAD_WIDTH);

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
  
  HAL_Delay(100);

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
  len = snprintf((char *)msg, MSG_MAX_LEN,
			             "Initialize nrf24l01p...%s", "\r\n");
  CDC_Transmit_FS(msg, len);       

  if (nrf24l01p_init(&nrf24l01p_param)) {
    while (1) {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(1000);
    }
  }
#endif
#ifdef TOF_TASK
  if (vl53l0x_init(VL53L0X_SENSE_HIGH_SPEED, 20)) {
      len = snprintf((char *)msg, MSG_MAX_LEN,
			             "Initialize VL53L0X failed!%s", "\r\n");
    while (1) {
      CDC_Transmit_FS(msg, len);
      HAL_Delay(1000);
    }
  }
#endif
#ifdef SENSOR_TASK
	struct lps22hb_cfg lps22hb = {
   .mode = LPS22HB_STREAM_MODE,
		.odr = LPS22HB_75HZ,
		.lpf = LPS22HB_BW_ODR_DIV_9,
		.ref_press = 0.0
	};
	if (lps22hb_init(lps22hb)) {
		len = snprintf((char *)msg, MSG_MAX_LEN,
			             "Initialize LPS22HB failed!%s", "\r\n");
		while (1) {
		  CDC_Transmit_FS(msg, len);
      HAL_Delay(1000);
    }
	}
	if (icm20948_init(1125, GYRO_2000_DPS, ACCEL_16G, LP_BW_119HZ)) {
		len = snprintf((char *)msg, MSG_MAX_LEN,
			             "Initialize ICM-20948 failed!%s", "\r\n");
		while (1) {
		  CDC_Transmit_FS(msg, len);
      HAL_Delay(1000);
    }
	}
  if (ak09916_init(AK09916_100HZ_MODE)) {
    len = snprintf((char *)msg, MSG_MAX_LEN,
			             "Initialize AK-09916 failed!%s", "\r\n");
    while (1) {
		  CDC_Transmit_FS(msg, len);
      HAL_Delay(1000);
    }
  }
  icm20948_read_axis6(&sensor_data.ax, &sensor_data.ay, &sensor_data.az,
                      &sensor_data.gx, &sensor_data.gy, &sensor_data.gz);
  ak09916_read_data(&sensor_data.mx, &sensor_data.my, &sensor_data.mz);
  enu2ned(&sensor_data.ax, &sensor_data.ay, &sensor_data.az);
  enu2ned(&sensor_data.mx, &sensor_data.my, &sensor_data.mz);
  ahrs_init(sensor_data.ax, sensor_data.ay, sensor_data.az,
            sensor_data.mx, sensor_data.my, sensor_data.mz);
  // ahrs_init_imu(sensor_data.ax, sensor_data.ay, sensor_data.az);
  ahrs2euler(&attitude.roll, &attitude.pitch, &attitude.yaw);
  attitude.yaw_target = attitude.yaw;
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
  status = xTaskCreate(radio_task, "radio_task", 200, NULL, 5, &radio_handler);
	configASSERT(status == pdPASS);
#endif
#ifdef SENSOR_TASK
  status = xTaskCreate(sensor_task, "sensor_task", 300, NULL, 3, &sensor_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef TOF_TASK
  status = xTaskCreate(tof_task, "tof_task", 400, NULL, 3, &tof_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef SD_TASK
  status = xTaskCreate(sd_task, "sd_task", 800, NULL, 2, &sd_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef ADC_TASK
	status = xTaskCreate(adc_task, "adc_task", 200, NULL, 3, &adc_handler);
	configASSERT(status == pdPASS);
#endif
#ifdef CTRL_TASK
	status = xTaskCreate(ctrl_task, "ctrl_task", 400, NULL, 2, &ctrl_handler);
  configASSERT(status == pdPASS);
#endif
#ifdef MSG_TASK
	status = xTaskCreate(msg_task, "msg_task", 400, NULL, 1, &msg_handler);
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

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveFromISR(adc_handler, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Task Function */
static void radio_task(void *param)
{
  struct attitude *att = &attitude;
  struct position *pos = &position;
  struct payload *pl = &payload.data;
  uint16_t *thro = &throttle;
  struct ctrl_param *ctrl = &ctrl_param;
  uint8_t start_flag = 1;

  while (1) {
    if (start_flag) {
      nrf24l01p_start_rx();
      start_flag = 0;
    }
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    nrf24l01p_rxtx(payload.bytes, ack_payload.bytes, ACK_PAYLOAD_WIDTH);
    /* decode payload */
    if (pl->throttle < MAX_THROTTLE && pl->throttle >= 0)
      *thro = pl->throttle;
    att->roll_target = (float)pl->roll_target * 9.587378e-5f;
    att->pitch_target = (float)pl->pitch_target * 9.587378e-5f;
    // if (pl->yaw_target > 1.f || pl->yaw_target < -1.f)
    //   att->yaw_target += (float)pl->yaw_target * 9.587378e-5f;

    pos->height_target = (float)pl->height_target * 0.001f;

    ctrl->P = (float)pl->P * 0.05f;
    ctrl->I = (float)pl->I * 0.05f;
    ctrl->D = (float)pl->D * 0.05f;
    ctrl->mode = pl->mode;
  }
}

void enu2ned(float *x, float *y, float *z)
{
  float tmp = *x;

  *x = *y;
  *y = tmp;
  *z = -(*z);
}

static void sensor_task(void *param)
{
  TickType_t mag_tick = 0, baro_tick = 0;
  struct sensor_data *sensor = &sensor_data;
  struct attitude *att = &attitude;
  struct position *pos = &position;
  float dt = 0.002;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0, mag_square;
  float press = 0, temp = 0;
  uint8_t mag_err = 1;

  while (1) {
    SEGGER_SYSVIEW_Print("imu");
    if (!icm20948_read_axis6(&ax, &ay, &az, &gx, &gy, &gz)) {
      gx *= DEG2RAD;
      gy *= DEG2RAD;
      gz *= DEG2RAD;
      enu2ned(&gx, &gy, &gz);
      enu2ned(&ax, &ay, &az);     
      sensor->ax = ax;
      sensor->ay = ay;
      sensor->az = az;
      sensor->gx = gx;
      sensor->gy = gy;
      sensor->gz = gz;
    }
    SEGGER_SYSVIEW_Print("mag");
    if (mag_tick == pdMS_TO_TICKS(10)) {
      if (!ak09916_read_data(&mx, &my, &mz)) {
        mag_square = mx * mx + my * my + mz * mz;
        /* check if disturbed or not by square */
        /* 25 uT < norm(mag) < 65 uT */
        if (mag_square > 625.f && mag_square < 4225.f) {
          /* transform axis to attitude form */
          enu2ned(&mx, &my, &mz);
          sensor->mx = mx;
          sensor->my = my;
          sensor->mz = mz;
          mag_err = 0;
        } else {
          mag_err = 1;
        }
      } else {
        mag_err = 1;
      }
      mag_tick = 0;
    }
    SEGGER_SYSVIEW_Print("baro");
    if (baro_tick == pdMS_TO_TICKS(20)) {
      if (!(*lps22hb_read_data)(&press, &temp)) {
        sensor->pressure = press;
        sensor->temperature = temp;
        pos->height = (powf(1013.25 / press, 1 / 5.257) - 1.0) *
                      (temp + 273.15) / 0.0065;
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
    ahrs2euler(&att->roll, &att->pitch, &att->yaw);
    ahrs2quat(att->q);
  
    vTaskDelay(1);
  }
}

static void tof_task(void *param)
{
  uint16_t *sensor_d = &sensor_data.distance;
  float *pos_d = &position.distance;
  uint16_t d;

  while (1) {
    if (vl53l0x_is_range_ready()) {
          if (!vl53l0x_get_distance(&d)) {
            *sensor_d = d;
            *pos_d = (float)d * 0.001;
          }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void sd_task(void *param)
{
  FATFS fs;
  FIL fil;
  struct sensor_data tmp;
  char buffer[256];
  UINT len, rm;
  BaseType_t start_tick;
  BaseType_t pass_tick;
  uint8_t process = 0;

  while(1) {
    if (process == 0) {
      f_mount(&fs, "", 0);
      f_open(&fil, "test1.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
      start_tick = xTaskGetTickCount();
      process = 1;
    } else if (process == 1) {
      pass_tick = xTaskGetTickCount() - start_tick;

      if (pass_tick < pdMS_TO_TICKS(10000)) {
        vTaskSuspendAll();
        memcpy(&tmp, &sensor_data, sizeof(struct sensor_data));
        xTaskResumeAll();
        len = snprintf(buffer, 256, "ax: %.2f, ay: %.2f, az: %.2f, "
                                    "gx: %.2f, gy: %.2f, gz: %.2f, "
                                    "p: %.2f, d: %.2f\n",
                                    tmp.ax, tmp.ay, tmp.az, tmp.gx, tmp.gy,
                                    tmp.gz, tmp.pressure, tmp.temperature);
        f_write(&fil, buffer, len, &rm);
      } else {
        f_close(&fil);
        f_mount(NULL, "", 0);
        process = 2;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void adc_task(void *param)
{
  float *voltage = &battery_voltage;
  uint16_t data[2];
  uint8_t start_flag = 1;

  while (1) {
    if (start_flag)
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 2);
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    /* voltage = adc / 4096 * 3.3 / 1.5 * 8.05 */
    *voltage = (float)data[0] * 0.0043237304f;
    /* current = ? */

    vTaskDelay(pdMS_TO_TICKS(20));
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
  struct attitude *att = &attitude;
  struct position *pos = &position;
  struct ctrl_param *ctrl = &ctrl_param;
  uint16_t *thro = &throttle;
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
    thro_duty = *thro;
    memcpy(q, att->q, 4 * sizeof(float));
    sp_r = att->roll_target;
    sp_p = att->pitch_target;
    sp_y = att->yaw_target;
    P = ctrl->P;
    I = ctrl->I;
    D = ctrl->D;
    mode = ctrl->mode;
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
  struct sensor_data *sensor = &sensor_data;
	struct attitude *att = &attitude;
  struct position *pos = &position;
  float *voltage = &battery_voltage;
  struct ack_payload *ack_pl = &ack_payload.data;
  uint16_t *thro = &throttle;
  struct ctrl_param *ctrl = &ctrl_param;
  BaseType_t radio_wm, sensor_wm, tof_wm, sd_wm, adc_wm, ctrl_wm, msg_wm;
  BaseType_t min_remaining, remaining;

	while (1) {
    current_tick = xTaskGetTickCount();
    pass_tick = current_tick - last_tick;
    last_tick = current_tick;
    /* encode ack payload */
    vTaskSuspendAll();
    ack_pl->roll = (int16_t)(att->roll * 10430.378f);
    ack_pl->pitch = (int16_t)(att->pitch * 10430.378f);
    ack_pl->yaw = (int16_t)(att->yaw * 10430.378f);
    ack_pl->motor[0] = motor[0];
    ack_pl->motor[1] = motor[1];
    ack_pl->motor[2] = motor[2];
    ack_pl->motor[3] = motor[3];
    ack_pl->throttle = *thro;
    ack_pl->height = (int16_t)(pos->height * 100.f);
    ack_pl->voltage = (uint8_t)(*voltage * 10.f);
    xTaskResumeAll();

    // radio_wm = uxTaskGetStackHighWaterMark(radio_handler);
    sensor_wm = uxTaskGetStackHighWaterMark(sensor_handler);
    // tof_wm = uxTaskGetStackHighWaterMark(tof_handler);
    sd_wm = uxTaskGetStackHighWaterMark(sd_handler);
    adc_wm = uxTaskGetStackHighWaterMark(adc_handler);
    ctrl_wm = uxTaskGetStackHighWaterMark(ctrl_handler);
    msg_wm = uxTaskGetStackHighWaterMark(msg_handler);

    min_remaining = xPortGetMinimumEverFreeHeapSize();
    remaining = xPortGetFreeHeapSize();

		len = snprintf((char *)msg, MSG_MAX_LEN,
			             "r: %.3f, p: %.3f, y: %.3f, "
                   "rsp: %.3f, psp: %.3f, ysp: %.3f\r\n"
                   "h: %.2f, d: %d, v: %.2f, throttle: %d\r\n"
                   "m0: %d, m1: %d, m2: %d, m3: %d\r\n"
                   "P: %.2f, I: %.2f, D: %.2f, crash: %d\r\n"
                   "stack wm:\n\r"
                   "radio: %ld, sensor: %ld, tof, %ld, sd: %ld, adc: %ld\n\r"
                   "ctrl: %ld, msg: %ld, min rm: %ld, cur rm: %ld\r\n"
                   "pass tick: %ld\r\n",
			             att->roll, att->pitch, att->yaw,
                   att->roll_target, att->pitch_target, att->yaw_target,
             		   pos->height, sensor->distance, *voltage, *thro,
                   motor[0], motor[1], motor[2], motor[3],
                   ctrl->P, ctrl->I, ctrl->D, ack_pl->event,
                   radio_wm, sensor_wm, tof_wm, sd_wm, adc_wm, ctrl_wm, msg_wm,
                   min_remaining, remaining, pass_tick);
    CDC_Transmit_FS(msg, len);
    vTaskDelay(pdMS_TO_TICKS(100));
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
    uint16_t count;

    count = Timer1;
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
