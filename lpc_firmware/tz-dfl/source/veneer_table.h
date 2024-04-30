#ifndef _VENEER_TABLE_
#define _VENEER_TABLE_

#include "adc.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX_STRING_LENGTH 0x400

typedef enum _led_color {
    LED_Red = 0,
    LED_Green,
    LED_Blue
} led_color_t;

typedef enum _led_control {
    LED_Set = 0,
    LED_Clear,
    LED_Toggle
} led_control_t;

typedef struct _cyccnt {
    uint32_t start;
    uint32_t stop;
} cyccnt_t;

void DbgConsole_Printf_NSE(char const *s) __attribute__((cmse_nonsecure_entry));
void LED_Control(led_color_t color, led_control_t control) __attribute__((cmse_nonsecure_entry));

adc_data_t GetSensorDataNSE(void) __attribute__((cmse_nonsecure_entry));

void     SetTemperaturePtr(float *ptr) __attribute__((cmse_nonsecure_entry));
uint32_t GetTemperaturePtr(void) __attribute__((cmse_nonsecure_entry));

#endif /* _VENEER_TABLE_ */
