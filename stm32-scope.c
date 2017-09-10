#include "stm32-scope.h"


int
main()
{
  setup_serial();
  serial_puts("Initialising...\r\n");
  setup_led();
  config_adc();
  serial_puts("Initialising done, starting main loop.\r\n");

  for (;;) {
    float v;

    led_on();
    serial_putchar('/');
    delay(MCU_HZ/3/2);
    led_off();
    serial_putchar('\\');
    delay(MCU_HZ/3/2);

    v = adc_voltage_read();
    serial_puts("Voltage: ");
    println_float(v, 1, 3);

    serial_puts("    Internal reference: ");
    println_uint32(adc_vrefint_read());
  }
}
