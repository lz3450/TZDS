#include "sys_tick.h"

volatile uint32_t g_systickCounter;

void SysTick_Handler(void)
{
    if (g_systickCounter > 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_Init(void)
{
    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        for (;;) {}
    }
}

void delay(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U) {}
}
