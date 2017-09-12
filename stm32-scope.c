#include "stm32-scope.h"


static void
config_interrupt(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}


void
EXTI0_IRQHandler(void)
{
  if (EXTI->PR & EXTI_Line0) {
    serial_putchar('.');
    /* Clear the pending interrupt event. */
    EXTI->PR = EXTI_Line0;
  }
}


int
main()
{
  setup_serial();
  serial_puts("Initialising...\r\n");
  setup_led();
  config_interrupt();
  config_adc();
  config_adc_dma();
  setup_st7787_io();

  {
    union {
      NVIC_InitTypeDef NVIC_InitStruct;
    } u;

    /* Software interrupt on EXTI0 (no GPIO triggering). */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /* Disable events on EXTI0. */
    EXTI->EMR &= ~EXTI_Line0;
    /* Disable GPIO triggers. */
    EXTI->RTSR &= ~EXTI_Line0;
    EXTI->FTSR &= ~EXTI_Line0;
    /* Enable interrupts on EXTI0. */
    EXTI->IMR |= EXTI_Line0;

    /* Clear any pending interrupt before enabling. */
    EXTI->PR = EXTI_Line0;
    u.NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    u.NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15;
    u.NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    u.NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&u.NVIC_InitStruct);

    serial_puts("Softint first trigger:\r\n");
    EXTI->PR = EXTI_Line0;
    EXTI->SWIER = EXTI_Line0;
    serial_puts("Softint second trigger:\r\n");
    EXTI->PR = EXTI_Line0;
    EXTI->SWIER = EXTI_Line0;
    serial_puts("Softint done:\r\n");
  }

  serial_puts("Initialising done, starting main loop.\r\n");

  st7787_init();
  st7787_test();
  adc_dma_start();
  for (;;) {
    if (adc_triggered()) {
      display_render_adc();
      adc_start_sample_with_trigger(2000, 1, 0);
    }
  }
}
