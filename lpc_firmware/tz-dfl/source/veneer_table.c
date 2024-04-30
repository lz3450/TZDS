
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif

#include "arm_cmse.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpadc.h"
#include "pin_mux.h"
#include "adc.h"
#include "veneer_table.h"

float *g_pCurrentTemperature;

void DbgConsole_Printf_NSE(char const *s)
{
    size_t string_length;
    /* Access to non-secure memory from secure world has to be properly validated */
    /* Check whether string is properly terminated */
    string_length = strnlen(s, MAX_STRING_LENGTH);
    if ((string_length == MAX_STRING_LENGTH) && (s[string_length] != '\0')) {
        PRINTF("String too long or invalid string termination!\r\n");
        for (;;) {
        }
    }

    /* Check whether string is located in non-secure memory */
    if (cmse_check_address_range((void *)s, string_length, CMSE_NONSECURE | CMSE_MPU_READ) == NULL) {
        PRINTF("String is not located in normal world!\r\n");
        for (;;) {
        }
    }
    PRINTF(s);
}

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

adc_data_t GetSensorDataNSE(void)
{
    extern adc_data_t g_ADCData;
    extern bool          g_LpadcConversionCompletedFlag;
    g_ADCData.Vbe1 = 0;
    g_ADCData.Vbe8 = 0;

    LPADC_DoSoftwareTrigger(ADC0, 1);
    while (false == g_LpadcConversionCompletedFlag) {
    }
    g_LpadcConversionCompletedFlag = false;

    return g_ADCData;
}

void SetTemperaturePtr(float *ptr)
{
    g_pCurrentTemperature = cmse_check_address_range((void *)ptr, sizeof(float), CMSE_NONSECURE | CMSE_MPU_READ);
    assert(NULL != g_pCurrentTemperature);
}
