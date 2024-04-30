#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "fsl_gpio.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

typedef struct _sensor_data
{
    uint16_t Vbe1;
    uint16_t Vbe8;
} sensor_data_t;

volatile sensor_data_t g_SensorData;
volatile bool          g_LpadcConversionCompletedFlag = false;

float MeasureTemperature(sensor_data_t data)
{
    float slope  = FSL_FEATURE_LPADC_TEMP_PARAMETER_A_CHIP_REV_1B;
    float offset = FSL_FEATURE_LPADC_TEMP_PARAMETER_B_CHIP_REV_1B;
    float alpha  = FSL_FEATURE_LPADC_TEMP_PARAMETER_ALPHA_CHIP_REV_1B;
    float temperature =
        -273.15f; /* Absolute zero degree as the incorrect return value. */

    /* Final temperature = A*[alpha*(Vbe8-Vbe1)/(Vbe8 + alpha*(Vbe8-Vbe1))] - B.
     */
    temperature = slope * (alpha * (data.Vbe8 - data.Vbe1) /
                           (data.Vbe8 + alpha * (data.Vbe8 - data.Vbe1))) -
                  offset;
    return temperature;
}

void ReadTemperature(void)
{
    GPIO_PortToggle(GPIO, LED_BLUE_PORT, LED_BLUE_PIN_MASK);

    LPADC_DoSoftwareTrigger(ADC0, 1);
    while (false == g_LpadcConversionCompletedFlag) {}
    PRINTF("Current temperature: \033[33;40m%6.2f\033[37;40m\r\n", MeasureTemperature(g_SensorData));
    g_LpadcConversionCompletedFlag = false;

    SDK_DelayAtLeastUs(1000000, PLL150M_CORE_CLOCK);
}

void read_temperature_task(void *pvParameters)
{
    for (;;)
    {
        ReadTemperature();
    }
}

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    if (pdPASS != xTaskCreate(read_temperature_task,
                              "read_temperature_task",
                              configMINIMAL_STACK_SIZE + 256,
                              NULL,
                              configMAX_PRIORITIES - 1,
                              NULL))
    {
        for (;;) {}
    }

    vTaskStartScheduler();
    for (;;) {}
}

/* ADC0_IRQn interrupt handler */
void ADC0_IRQHandler(void)
{
    lpadc_conv_result_t convResultStruct;

    /* For best temperature measure performance, the recommended LOOP Count should
     * be 4, but the first two results is useless. */
    (void)LPADC_GetConvResult(ADC0, &convResultStruct, 0);
    (void)LPADC_GetConvResult(ADC0, &convResultStruct, 0);
    /* Read the 2 temperature sensor result. */
    if (true == LPADC_GetConvResult(ADC0, &convResultStruct, 0))
    {
        g_SensorData.Vbe1 = convResultStruct.convValue;
    }
    if (true == LPADC_GetConvResult(ADC0, &convResultStruct, 0))
    {
        g_SensorData.Vbe8 = convResultStruct.convValue;
    }

    g_LpadcConversionCompletedFlag = true;
}
