#include "delay.h"

static u8  fac_us = 0;
static u16 fac_ms = 0;

// Delay configuration. SYSCLK means current clock frequency (MHZ)
void Delay_Config(u8 SYSCLK)
{
// If define OS_CRITICAL_METHOD then use ucosII
#ifdef OS_CRITICAL_METHOD
    u32 reload;
#endif

    // System timer configuration (divided by eight)
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    // Get time value (us)
    fac_us = SYSCLK / 8;

// If define OS_CRITICAL_METHOD then use ucosII
#ifdef OS_CRITICAL_METHOD
    // Ticks per second (K)
    reload = SYSCLK / 8;
    // Set real value of reload
    reload *= 1000000 / OS_TICKS_PER_SEC;
    // Minimum unit that can be delayed in ucosII
    fac_ms = 1000 / OS_TICKS_PER_SEC;
    // Enable systick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    // Interrupt every (1 / OS_TICKS_PER_SEC) seconds
    SysTick->LOAD  = reload;
    // Enable systick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
#else
    // Get time value (ms)
    fac_ms = (u16)fac_us * 1000;
#endif
}

// If define OS_CRITICAL_METHOD then use ucosII
#ifdef OS_CRITICAL_METHOD
// Delay nus us
void delay_us(u32 nus)
{
    u32 ticks;
    u32 told, tnow, tcnt;
    
    // Set reload
    u32 reload = SysTick->LOAD;
    // Set ticks for delay
    ticks = nus * fac_us;
    tcnt = 0;
    // Lock OS schedule
    OSSchedLock();
    // Current systick value
    told = SysTick->VAL;
    
    while(1)
    {
        // Current systick value
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            // Decrement count
            if(tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                // Timer has counted for one round, compensating the reload value
                tcnt += reload - tnow + told;
            }
            told = tnow;
            // Time-out
            if(tcnt >= ticks)
            {
                break;
            }
        }
    };

    // Unlock OS schedule
    OSSchedUnlock();
}

// Delay nms ms
void delay_ms(u16 nms)
{
    // If OS is running
    if(OSRunning == OS_TRUE && OSLockNesting == 0)
    {
        // If delay time is greater than the operating system minimum unit
        if(nms >= fac_ms)
        {
            // OS time delay
            OSTimeDly(nms / fac_ms);
        }
        // Use normal method to delay
        nms %= fac_ms;
    }
    // Normal method to delay
    delay_us((u32)(nms * 1000));
}

// Not use ucosII
#else
// Delay nus us
void delay_us(u32 nus)
{
    u32 temp;
    // Set load
    SysTick->LOAD  = nus * fac_us;
    // Clear timer
    SysTick->VAL   = 0x00;
    // Enable systick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // Start timing until time-out
    do
    {
        temp = SysTick->CTRL;
    }
    while((temp & 0x01) && !(temp & (1<<16)));

    // Close systick
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    // Clear timer
    SysTick->VAL   = 0X00; 
}

// Delay nms ms
void delay_ms(u16 nms)
{
    u32 temp;

    // Set load
    SysTick->LOAD  = (u32)nms * fac_ms;
    // Clear timer
    SysTick->VAL   = 0x00;
    // Enable systick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // Start timing until time-out
    do
    {
        temp = SysTick->CTRL;
    }
    while((temp & 0x01) && !(temp & (1<<16)));

    // Close systick
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
    // Clear timer
    SysTick->VAL =0X00;
}
#endif
