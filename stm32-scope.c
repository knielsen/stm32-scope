#include "stm32-scope.h"


static void
config_systick(void)
{
  SysTick->LOAD = 0xffffff;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


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


uint8_t trigger_enabled = 0;
uint16_t trigger_level = 2000;

static void
restart_adc(void)
{
  adc_start_sample_with_trigger((trigger_enabled ? trigger_level : 0), 0, 1);
}


static void
handle_serial_rx(void)
{
  int c;

  while ((c = serial_rx()) >= 0) {
    switch (c) {
    case 't':
      if (trigger_enabled) {
        trigger_enabled = 0;
        serial_puts("Trigger disabled.\r\n");
      } else {
        trigger_enabled = 1;
        serial_puts("Trigger enabled.\r\n");
      }
      restart_adc();
      break;
    case '<':
      if (trigger_level > 100)
        trigger_level -= 100;
      else
        trigger_level = 1;
      restart_adc();
      break;
    case '>':
      if (trigger_level < 4095 - 100)
        trigger_level += 100;
      else
        trigger_level = 4095;
      restart_adc();
      break;
    default:
      serial_puts("Unhandled key: '");
      serial_putchar(c);
      serial_puts("'\r\n");
      break;
    }
  }
}


int
main()
{
  setup_serial();
  serial_puts("Initialising...\r\n");
  setup_led();
  config_systick();
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

  st7787_init();
  init_fft();
  serial_puts("Initialising done, starting main loop.\r\n");

  st7787_test();
  adc_dma_start();
  for (;;) {
    handle_serial_rx();
    if (adc_triggered()) {
      display_render_adc();
      fft_sample_buf();
      display_render_fft();
      restart_adc();
    }
  }
}
