#include "veneer_table.h"

#include "fsl_gpio.h"

#include "pin_mux.h"

#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif

void LED_Control(led_color_t color, led_control_t control)
{
    uint32_t port, mask;

    switch (color) {
    case LED_Red:
        port = LED_RED_PORT;
        mask = LED_RED_PIN_MASK;
        break;
    case LED_Green:
        port = LED_GREEN_PORT;
        mask = LED_GREEN_PIN_MASK;
        break;
    case LED_Blue:
        port = LED_BLUE_PORT;
        mask = LED_BLUE_PIN_MASK;
        break;
    default:
        return;
    }

    switch (control) {
    case LED_Set:
        GPIO_PortClear(GPIO, port, mask);
        break;
    case LED_Clear:
        GPIO_PortClear(GPIO, port, mask);
        break;
    case LED_Toggle:
        GPIO_PortToggle(GPIO, port, mask);
        break;
    default:
        return;
    }
}
