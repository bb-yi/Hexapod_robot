#include "user_adc.h"
#include "adc.h"

float Get_Voltage(void)
{
    float Voltage;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        Voltage = ((float)HAL_ADC_GetValue(&hadc1) / 65535.0f * 3.3f * 11.0f) * 1.4f;
    }
    return Voltage;
}
float Get_Avg_Voltage(void)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 10; i++)
    {
        sum += Get_Voltage();
    }
    return sum / 10.0f;
}