#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

typedef struct _adc_data {
    uint16_t Vbe1;
    uint16_t Vbe8;
} adc_data_t;

float MeasureTemperature(adc_data_t data);
void ADC_GetConvResult(void);

#endif /* _ADC_H_ */