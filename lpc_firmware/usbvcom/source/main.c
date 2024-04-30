#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "fsl_gpio.h"

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    for (;;)
    {
        USB_DeviceTasks();
        GPIO_PortToggle(GPIO, LED_BLUE_PORT, LED_BLUE_PIN_MASK);
        SDK_DelayAtLeastUs(1000, PLL150M_CORE_CLOCK);
    }
}
