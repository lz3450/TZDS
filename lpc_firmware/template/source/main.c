#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "fsl_gpio.h"

#include "sys_tick.h"

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    SysTick_Init();

    for (;;) {
        GPIO_PortToggle(GPIO, BOARD_INITLEDSPINS_LED_BLUE_PORT, BOARD_INITLEDSPINS_LED_BLUE_PIN_MASK);
        SysTick_DelayTicks(1000);
    }
}
