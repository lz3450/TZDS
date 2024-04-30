#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "resource_config.h"

#include "fsl_gpio.h"

#include "adc.h"
#include "loader.h"
#include "shell.h"
#include "sys_tick.h"
#include "tzm_api.h"

#define NORMAL_WORLD_START (0x00020000U)

volatile bool       g_LpadcConversionCompletedFlag = false;
volatile bool       g_JumpToNormalWorldFlag        = false;

void SystemInitHook(void)
{
    /* The TrustZone should be configured as early as possible after RESET.
     * Therefore it is called from SystemInit() during startup. The SystemInitHook() weak function overloading is used for this purpose.
     */
    BOARD_InitTrustZone();
}

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    /* Disable LDOGPADC power down */
    POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);
    /* Disable Temperature sensor power down. */
    POWER_DisablePD(kPDRUNCFG_PD_TEMPSENS);

    SysTick_Init();

    PRINTF("Hello from SECURE world!\r\n");

    assert(kStatus_LoaderSuccess == LOADER_Init());

    BOARD_Shell_Init();

    while (!g_JumpToNormalWorldFlag)
    {
        GPIO_PortToggle(GPIO, LED_GREEN_PORT, LED_GREEN_PIN_MASK);
        /* Delay 1000ms */
        SysTick_DelayTicks(1000);
    }
    for (int i = 3; i > 0; i--)
    {
        GPIO_PortToggle(GPIO, LED_GREEN_PORT, LED_GREEN_PIN_MASK);
        PRINTF("Jump to NORMAL world after %ds...\r\n", i);
        SysTick_DelayTicks(1000);
    }
    GPIO_PortSet(GPIO, LED_GREEN_PORT, LED_GREEN_PIN_MASK);
    TZM_JumpToNormalWorld(NORMAL_WORLD_START);

    while (true)
    {
    }
}

void UserButton(pint_pin_int_t pint, uint32_t pmatch_status)
{
    if (kPINT_PinInt0 == pint)
    {
        g_JumpToNormalWorldFlag = true;
    }
}

/* ADC0_IRQn interrupt handler */
void ADC0_IRQHandler(void)
{
    ADC_GetConvResult();
    g_LpadcConversionCompletedFlag = true;
}
