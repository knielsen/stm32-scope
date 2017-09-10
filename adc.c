#include "stm32-scope.h"


/*
  This stores 16-bit samples, but is declared uint32_t so we get 32-bit
  alignment for DMA burst transfer.
*/
volatile uint32_t adc_dma_buffers[2][ADC_BUFFER_SIZE*sizeof(uint16_t)/sizeof(uint32_t)];


void
config_adc(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_Cmd(ADC1, DISABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd(ADC1, ENABLE);
}


void
config_adc_dma(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* ADC1 on DMA2 stream 4 channel 0. */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream4);

  DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_SIZE;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(ADC1->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_dma_buffers[0];
  DMA_Init(DMA2_Stream4, &DMA_InitStructure);
}


void
adc_dma_start(void)
{
  DMA2_Stream4->M0AR = (uint32_t)&adc_dma_buffers[0];
  DMA2_Stream4->M1AR = (uint32_t)&adc_dma_buffers[1];
  DMA2_Stream4->NDTR = ADC_BUFFER_SIZE;
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  DMA_DoubleBufferModeCmd(DMA2_Stream4, ENABLE);
  DMA_Cmd(DMA2_Stream4, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);
}


uint32_t
adc_read(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);

  /* Wait for the ADC to complete. */
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    ;

  return ADC_GetConversionValue(ADC1);
}


uint32_t
adc_vrefint_read(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    ;
  return ADC_GetConversionValue(ADC1);
}


float
adc_voltage_read(void)
{
  uint32_t reading = adc_read();
  return adc_val2voltage(reading);
}
