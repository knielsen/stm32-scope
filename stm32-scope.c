#include "stm32-scope.h"


int
main()
{
  setup_serial();
  serial_puts("Initialising...\r\n");
  setup_led();
  config_adc();
  config_adc_dma();
  setup_st7787_io();
  serial_puts("Initialising done, starting main loop.\r\n");

  st7787_init();
  st7787_test();
  adc_dma_start();
  for (;;) {
    float v;
    uint32_t i;
    char buf[20];

    led_on();
    serial_putchar('/');
    delay(MCU_HZ/3/2);
    led_off();
    serial_putchar('\\');
    delay(MCU_HZ/3/2);

    serial_puts("Readings:");
    for (i = 0; i < 4; ++i) {
      v = adc_val2voltage(adc_buf_val(0, i));
      serial_puts(" ");
      float_to_str(buf, v, 1, 3);
      serial_puts(buf);
    }
    serial_puts(" ..");
    for (i = 0; i < 4; ++i) {
      v = adc_val2voltage(adc_buf_val(1, (ADC_BUFFER_SIZE-4)+i));
      serial_puts(" ");
      float_to_str(buf, v, 1, 3);
      serial_puts(buf);
    }
    serial_puts("\r\n");

    /*
    serial_puts("    manual read: ");
    println_float(adc_val2voltage(ADC_GetConversionValue(ADC1)), 1, 3);

    serial_puts("HISR: 0x"); println_uint32_hex(DMA2->HISR);
    serial_puts("  CR: 0x"); println_uint32_hex(DMA2_Stream4->CR);
    serial_puts("NDTR: 0x"); println_uint32_hex(DMA2_Stream4->NDTR);
    serial_puts(" PAR: 0x"); println_uint32_hex(DMA2_Stream4->PAR);
    serial_puts("M0AR: 0x"); println_uint32_hex(DMA2_Stream4->M0AR);
    serial_puts("M1AR: 0x"); println_uint32_hex(DMA2_Stream4->M1AR);
    serial_puts(" FCR: 0x"); println_uint32_hex(DMA2_Stream4->FCR);

    serial_puts(" ADC_SR: 0x"); println_uint32_hex(ADC1->SR);
    serial_puts("ADC_CR1: 0x"); println_uint32_hex(ADC1->CR1);
    serial_puts("ADC_CR2: 0x"); println_uint32_hex(ADC1->CR2);
    */
  }
}
