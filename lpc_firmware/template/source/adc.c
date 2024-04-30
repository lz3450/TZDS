#include "adc.h"
#include "fsl_lpadc.h"

volatile adc_data_t g_ADCData;

float MeasureTemperature(adc_data_t data)
{
    float slope       = FSL_FEATURE_LPADC_TEMP_PARAMETER_A;
    float offset      = FSL_FEATURE_LPADC_TEMP_PARAMETER_B + 3;
    float alpha       = FSL_FEATURE_LPADC_TEMP_PARAMETER_ALPHA;
    float temperature = -273.15f; /* Absolute zero degree as the incorrect return value. */

    /* Final temperature = A*[alpha*(Vbe8-Vbe1)/(Vbe8 + alpha*(Vbe8-Vbe1))] - B. */
    temperature = slope * (alpha * (data.Vbe8 - data.Vbe1) / (data.Vbe8 + alpha * (data.Vbe8 - data.Vbe1))) - offset;
    return temperature;
}

void ADC_GetConvResult(void)
{
    lpadc_conv_result_t convResultStruct;

    /* For best temperature measure performance, the recommended LOOP Count should be 4, but the first two results is useless. */
    (void)LPADC_GetConvResult(ADC0, &convResultStruct, 0);
    (void)LPADC_GetConvResult(ADC0, &convResultStruct, 0);
    /* Read the 2 temperature sensor result. */
    if (true == LPADC_GetConvResult(ADC0, &convResultStruct, 0)) {
        g_ADCData.Vbe1 = convResultStruct.convValue;
    }
    if (true == LPADC_GetConvResult(ADC0, &convResultStruct, 0)) {
        g_ADCData.Vbe8 = convResultStruct.convValue;
    }
}
