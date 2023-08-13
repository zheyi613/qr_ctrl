/**
 * @file segger_uart_dma.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Terminal control for Flasher using USART1(PA9/PA10) in DMA mode
 * @date 2023-08-14
 */

/*
 * This source file is modified from
 * SysView_UARTSample_STM32F407/Application/HIF_UART.c
 * (https://wiki.segger.com/images/0/06/SysView_UARTSample_STM32F407.zip)
 */
#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1)
#include "SEGGER_RTT.h"
#include "stm32f4xx.h"
#include "string.h"

#define UART_BASECLK          (84000000) //note that max clock of APB1 (USART2) is 42 MHz
#define USART_RX_ERROR_FLAGS  0x0B

#define USART_REG       USART1
#define USART_TX_BIT    9
#define USART_RX_BIT    10
#define USART_IRQn      USART1_IRQn

#define _SERVER_HELLO_SIZE (4)
#define _TARGET_HELLO_SIZE (4)

#define TX_BUFFER_SIZE  128

char tx_buffer[TX_BUFFER_SIZE];
char rx_byte;
uint8_t rec_tx;

/* 
 * "Hello" message expected by SysView: [ 'S', 'V',
 * <PROTOCOL_MAJOR>, <PROTOCOL_MINOR> ]
 */
static const uint8_t _abHelloMsg[_TARGET_HELLO_SIZE] = {
	'S', 'V', (SEGGER_SYSVIEW_VERSION / 10000),
	(SEGGER_SYSVIEW_VERSION / 1000) % 10};

static struct {
  uint8_t NumBytesHelloRcvd;
  uint8_t NumBytesHelloSent;
  int           ChannelID;
} _SVInfo = {0,0,1};

/**
 * @brief Initialize USART TX/RX in DMA mode
 *    TX: normal mode
 *    RX: circular mode
 * @param baudrate (Hz)
 * @param tx_buf buffer of transmitter
 * @param rx_buf buffer of receiver
 */
void USART_DMA_init(uint32_t baudrate, char *tx_buf, char *rx_buf)
{
        uint32_t v, div;

        /* Enable PORTA and DMA2 clock */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;

        /* Enable USART1 clock */
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        /*   Initialize TX DMA (Stream 7, CH4)   */

        /* Wait DMA stream to disable if current tranfers are not finished */
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream7->CR & DMA_SxCR_EN);
        DMA2_Stream7->CR = 0;
        /* Set peripheral address */
        DMA2_Stream7->PAR = (uint32_t)&USART1->DR;

        /* Set memory address */
        DMA2_Stream7->M0AR = (uint32_t)tx_buf;

        /* Set DMA configuration */
        DMA2_Stream7->CR = 0
                | DMA_CHANNEL_4
                | DMA_MBURST_SINGLE   /* Memory burst single */
                | DMA_PBURST_SINGLE   /* Peripheral burst single */
                | DMA_PRIORITY_HIGH
                | 0                   /* Peripheral increment by PSIZE */
                | DMA_MDATAALIGN_BYTE /* Mememory data size */
                | DMA_PDATAALIGN_BYTE /* Peripheral data size */
                | DMA_MINC_ENABLE     /* Memory increment mode enable */
                | DMA_PINC_DISABLE    /* Peripheral increment mode disble */
                | DMA_NORMAL          /* Circular mode: disable */
                | DMA_MEMORY_TO_PERIPH /* Data transfer direction */
                | 0                   /* Flow controller: DMA */
                | DMA_SxCR_TCIE;      /* Transfer complete interrupt enable */

        /* Setup callbacks which are called by ISR handler and enable
         * interrupt in NVIC (priority: 6) */
        NVIC_SetPriority(DMA2_Stream7_IRQn, 6);
        NVIC_EnableIRQ(DMA2_Stream7_IRQn);

        /*   Initialize RX DMA (Stream 5, CH4)   */

        /* Wait DMA stream to disable if current tranfers are not finished */
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream5->CR & DMA_SxCR_EN);
        DMA2_Stream5->CR = 0;
        /* Set peripheral address */
        DMA2_Stream5->PAR = (uint32_t)&USART1->DR;

        /* Set memory address */
        DMA2_Stream5->M0AR = (uint32_t)rx_buf;

        /* Set the total number of data items to be transferred */
        DMA2_Stream5->NDTR = 1U;

        /* Set DMA configuration */
        DMA2_Stream5->CR = 0 
                | DMA_CHANNEL_4
                | DMA_MBURST_SINGLE   /* Memory burst single */
                | DMA_PBURST_SINGLE   /* Peripheral burst single */
                | DMA_PRIORITY_HIGH
                | 0                   /* Peripheral increment by PSIZE */
                | DMA_MDATAALIGN_BYTE /* Mememory data size */
                | DMA_PDATAALIGN_BYTE /* Peripheral data size */
                | DMA_MINC_DISABLE    /* Memory increment mode disable */
                | DMA_PINC_DISABLE    /* Peripheral increment mode disble */
                | DMA_CIRCULAR        /* Circular mode: enable */
                | DMA_PERIPH_TO_MEMORY /* Data transfer direction */
                | 0                   /* Flow controller: DMA */
                | DMA_SxCR_TCIE;      /* Transfer complete interrupt enable */

        /* Setup callbacks which are called by ISR handler and enable
         * interrupt in NVIC (priority: 6) */
        NVIC_SetPriority(DMA2_Stream5_IRQn, 6);
        NVIC_EnableIRQ(DMA2_Stream5_IRQn);

        /* Configure USART RX/TX pins for alternate function (AF7) */
        v  = GPIOA->AFR[USART_TX_BIT >> 3];
        v &= ~(15UL << ((USART_TX_BIT & 0x7) << 2));
        v |=   (7UL << ((USART_TX_BIT & 0x7) << 2));
        GPIOA->AFR[USART_TX_BIT >> 3] = v;
        v  = GPIOA->AFR[USART_RX_BIT >> 3];
        v &= ~(15UL << ((USART_RX_BIT & 0x7) << 2));
        v |=   (7UL << ((USART_RX_BIT & 0x7) << 2));
        GPIOA->AFR[USART_RX_BIT >> 3] = v;

        /* Configure USART RX/TX pins for alternate function usage */
        v  = GPIOA->MODER;
        v &= ~((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
        v |=  ((2UL << (USART_TX_BIT << 1)) | (2UL << (USART_RX_BIT << 1)));
        GPIOA->MODER = v;

        /* Configure USART RX/TX pins output speed to high speed */
        v = GPIOA->OSPEEDR;
        v &= ~((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
        v |= ((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
        GPIOA->OSPEEDR = v;

        /* Initialize USART */
        USART_REG->CR1 = 0
                | USART_CR1_OVER8     /* oversampline by 8 */
                | 0                   /* USART disable */
                | 0                   /* 1 Start, 8 Data, n Stop bits */
                | 0                   /* Parity control disable */
                | 0                   /* TX empty interrupt disable */
                | 0                   /* TX complete interrupt disable */
                | 0                   /* RX not empty interrupt disable */
                | USART_CR1_TE        /* Transmitter enable */
                | USART_CR1_RE;       /* Receiver enable */

        USART_REG->CR2 = 0;     /* STOP bits: 1 Stop bit */
        
        /* Set baudrate */
        div = (UART_BASECLK << 1) / baudrate;
        div += (div & 0x1) << 1; /* rounding */
        div = (div & 0xFFF0) | ((div >> 1) & 0x7); /* int[15:4], frac[2:0] */
        if (div > 0xFFF7) /* Limit maximum div = 255.125, if div over max */
                div = 0xFFF7;
        else if (!div)    /* Limit div = 0.125, if div is zero */
                div = 0xFFF0 & (div << 4);
        USART_REG->BRR = div;

        /* Enable USART */
        USART_REG->CR1 |= USART_CR1_UE;

        /* Enable RX DMA */
        DMA2_Stream5->CR |= DMA_SxCR_EN;

        /* Clear TC bit in the SR */
        USART_REG->SR &= ~USART_SR_TC;

        /* Select three sample bit method and Enable DMA TX/RX */
        USART_REG->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

}

void SEGGER_UART_init(unsigned long baudrate)
{
        USART_DMA_init(baudrate, tx_buffer, &rx_byte);
        rec_tx = 0;
}

void USART_DMA_transmit(uint16_t size)
{
        /* Set number of data items to transfer */
        DMA2_Stream7->NDTR = (uint32_t)size;

        /* Read SR to clear TC and Check if last transmission is not done */
        while (!(USART_REG->SR & USART_SR_TC));

        /* Enable DMA */
        DMA2_Stream7->CR |= DMA_SxCR_EN;
}

void SEGGER_UART_transmit(void)
{
        uint16_t size;

        size = SEGGER_RTT_ReadUpBufferNoLock(_SVInfo.ChannelID, tx_buffer,
                                             TX_BUFFER_SIZE);
        if (size)
                USART_DMA_transmit(size);
        else
                rec_tx = 0;
}

void SEGGER_UART_TX_ON(void)
{
        if (!rec_tx) {
                rec_tx = 1;
                SEGGER_UART_transmit();
        }
}

/* USART RX DMA handler */
void DMA2_Stream5_IRQHandler(void)
{
        DMA2->HIFCR |= DMA_HISR_TCIF5; /* Clear TCIF */

        /* Not all bytes of <Hello> message received by SysView yet? */
        if (_SVInfo.NumBytesHelloRcvd < _SERVER_HELLO_SIZE) {
                _SVInfo.NumBytesHelloRcvd++;
                /* Not all bytes of <Hello> message sent to SysView yet? */
                if (_SVInfo.NumBytesHelloSent < _TARGET_HELLO_SIZE) {
                        memcpy(tx_buffer, _abHelloMsg, _TARGET_HELLO_SIZE);
                        _SVInfo.NumBytesHelloSent += 4;
                        USART_DMA_transmit(_TARGET_HELLO_SIZE);
                }
                return;
        }
        if (!SEGGER_SYSVIEW_IsStarted())
                SEGGER_SYSVIEW_Start();

        /* Write data into corresponding RTT buffer for application 
         * to read and handle accordingly */
        SEGGER_RTT_WriteDownBuffer(_SVInfo.ChannelID, &rx_byte, 1);

        SEGGER_UART_TX_ON();
}

/* USART TX DMA handler */
void DMA2_Stream7_IRQHandler(void)
{
        DMA2->HIFCR |= DMA_HISR_TCIF7; /* Clear TCIF */

        /* Moving data to DR is completed, but transmit is not done.
         * User need to check TC flag by software. */
        if (rec_tx)
                SEGGER_UART_transmit();
}
#endif