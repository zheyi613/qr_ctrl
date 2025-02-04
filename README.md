# qr_ctrl
A FreeRTOS flight controller(FC) for Quadrotor

# Hardware
- MCU: STM32F401RCT6 (FLASH: 256KB, SRAM: 64KB)
- IMU: MPU9250
- Barometer: LPS22HB
- Storage device: SDHC 16GB
- Wireless module: NRF24L01P
- Motor: 1400KV

# Threads (tasks)
- radio_task(SPI2): Receive flight command and Acknowledge drone state wirelessly form radio controller
- sensor_task(I2C2): Read sensor data and Implement complementary filter to get drone attitude
- gps_task(UART2): Read and Decode GPS data
- ctrl_task(PWM1234): Calculate PID output and Translate to motor output
- msg_task(USB): Show the drone state and OS cost
