#include "system.h"
#include "adc.h"

#ifdef CONFIG_ADC

void ADC_init() {
  ADC_InitTypeDef  ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_13Cycles5);

  ADC_Cmd(ADC1, ENABLE);

  // reset calibration
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));

  // recalibrate
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
}

u16_t ADC_sample() {
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (ADC_GetSoftwareStartConvStatus(ADC1));
  return ADC_GetConversionValue(ADC1);
}

#endif
