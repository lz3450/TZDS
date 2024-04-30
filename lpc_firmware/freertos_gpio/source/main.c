#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"

#include "fsl_gpio.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

static void led_red_blink(void *pvParameters);
static void led_blue_blink(void *pvParameters);
static void led_green_blink(void *pvParameters);

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    if (pdPASS != xTaskCreate(led_red_blink,
                              "led_red_blink",
                              configMINIMAL_STACK_SIZE + 64,
                              NULL,
                              configMAX_PRIORITIES - 1,
                              NULL))
    {
        for (;;)
        {
        }
    }

    if (pdPASS != xTaskCreate(led_blue_blink,
                              "led_blue_blink",
                              configMINIMAL_STACK_SIZE + 64,
                              NULL,
                              configMAX_PRIORITIES - 1,
                              NULL))
    {
        for (;;)
        {
        }
    }

    if (pdPASS != xTaskCreate(led_green_blink,
                              "led_green_blink",
                              configMINIMAL_STACK_SIZE + 64,
                              NULL,
                              configMAX_PRIORITIES - 1,
                              NULL))
    {
        for (;;)
        {
        }
    }

    vTaskStartScheduler();
    for (;;)
    {
    }
}

static void led_red_blink(void *pvParameters)
{
    for (;;)
    {
        GPIO_PortToggle(GPIO, LED_RED_PORT, LED_RED_PIN_MASK);
        vTaskDelay((TickType_t)1000);
    }
}

static void led_blue_blink(void *pvParameters)
{
    for (;;)
    {
        GPIO_PortToggle(GPIO, LED_BLUE_PORT, LED_BLUE_PIN_MASK);
        vTaskDelay((TickType_t)500);
    }
}

static void led_green_blink(void *pvParameters)
{
    for (;;)
    {
        GPIO_PortToggle(GPIO, LED_GREEN_PORT, LED_GREEN_PIN_MASK);
        vTaskDelay((TickType_t)200);
    }
}
