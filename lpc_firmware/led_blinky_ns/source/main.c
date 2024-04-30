#include "clock_config.h"
#include "veneer_table.h"

#include "sys_tick.h"

void SystemInit(void)
{
}

int main(void)
{
    /* Set systick reload value to generate 1ms interrupt */
    assert(SysTick_Config(PLL150M_CORE_CLOCK / 1000U) == 0);

    for (;;) {
        LED_Control(LED_Blue, LED_Toggle);
        SysTick_DelayTicks(1000);
    }
}
