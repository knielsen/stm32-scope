#include "stm32-scope.h"


/*
  Align the adc dma buffer on a 16-byte boundary. This way, we can do 16-byte
  dma bursts without crossing a 1024k boundary (which is not allowed according
  to the STM32F4 reference manual).
*/
volatile uint16_t adc_dma_buffers[2][ADC_BUFFER_SIZE] __attribute__ ((aligned(16)));


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
  ADC_CommonInitStructure.ADC_Prescaler = ADC_STM32_CLOCK_DIVIDER;
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
  union {
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStruct;
  } u;

  /* ADC1 on DMA2 stream 4 channel 0. */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream4);

  u.DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_SIZE;
  u.DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  u.DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  u.DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  u.DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  u.DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  u.DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  u.DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  u.DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  u.DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  u.DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  u.DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(ADC1->DR));
  u.DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  u.DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  u.DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_dma_buffers[0];
  DMA_Init(DMA2_Stream4, &u.DMA_InitStructure);

  /* Configure a transfer complete interrupt for ADC DMA. */
  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, DISABLE);
  u.NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream4_IRQn;
  u.NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
  u.NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  u.NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&u.NVIC_InitStruct);
  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
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

  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_STM32_SAMPLETIME_CONFIG);
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


volatile enum { SAMPLE_WAIT=0, SAMPLE_TRIGGER, SAMPLE_STORE } adc_state;
static uint16_t prev_sample;
static volatile uint16_t adc_trigger_level;
static volatile uint8_t trigger_rising, trigger_falling;

volatile uint16_t adc_sample_buffer[SAMPLE_BUFFER_SIZE];
static uint32_t sample_position;


void
DMA2_Stream4_IRQHandler(void)
{
  volatile uint16_t *p;
  uint32_t i;
  int state;
  uint16_t prev, level;
  uint8_t rise, fall;
  uint32_t pos;

  if (DMA2->HISR & (DMA_FLAG_TCIF4 & 0x0fffffff)) {
    /* Clear the interrupt request. */
    DMA2->HIFCR = (DMA_FLAG_TCIF4 & 0x0fffffff);

    /* Find which buffer DMA is currently using, and use the other one. */
    p = adc_dma_buffers[1 - (1 & (DMA2_Stream4->CR >> 19 /* DMA_SxCR_CT */))];
    state = adc_state;
    prev = prev_sample;
    level = adc_trigger_level;
    rise = trigger_rising;
    fall = trigger_falling;
    pos = sample_position;

    for (i = 0; i < ADC_BUFFER_SIZE; ++i) {
      uint16_t val = p[i];

      switch (state) {
      case SAMPLE_WAIT:
        break;
      case SAMPLE_TRIGGER:
        if (!( (rise && prev < level && val >= level) ||
               (fall && prev > level && val <= level) ||
               level == 0))
          break;
        state = SAMPLE_STORE;
        pos = 0;
        /* Fall through to store the triggering sample */
      case SAMPLE_STORE:
        /* ToDo: We should also store some data prior to the trigger. */
        adc_sample_buffer[pos++] = val;
        if (pos >= SAMPLE_BUFFER_SIZE)
          state = SAMPLE_WAIT;
        break;
      }
      prev = val;
    }

    prev_sample = prev;
    adc_state = state;
    sample_position = pos;
  }
}


/*
  Trigger level 0 means run mode (always trigger immediately).
*/
void
adc_start_sample_with_trigger(uint16_t level, uint8_t rising, uint8_t falling)
{
  adc_trigger_level = level;
  trigger_rising = rising;
  trigger_falling = falling;

  adc_state = SAMPLE_TRIGGER;
}


int
adc_triggered(void)
{
  return (adc_state == SAMPLE_WAIT);
}
