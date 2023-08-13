/**
 * @file dwt_delay.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Implement accuracy delay us with DWT
 * @date 2022-10-20
 */

#include "dwt_delay.h"
#include "stm32f4xx.h"

void DWT_Init(void)
{
        /* Disable TRC */
        CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
        /* Enable TRC */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        /* Disable clock cycle counter */
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
        /* Enable clock cycle counter */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        #if (__CORTEX_M == 7)
                DWT->LAR = 0xC5ACCE55; /* Use in Core-M7 */
        #endif
        /* Reset the clock cycle counter value */
        DWT->CYCCNT = 0;

}

void DWT_Delay(uint32_t us)
{
        uint32_t startTick = DWT->CYCCNT,
                 delayTicks = us * (SystemCoreClock / 1000000);
        
        while(DWT->CYCCNT - startTick < delayTicks);
}

uint32_t DWT_GetTick(void)
{
        return (DWT->CYCCNT);
}