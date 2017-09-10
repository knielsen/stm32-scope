#include "stm32-scope.h"


int
main()
{
  setup_serial();
  serial_puts("Initialising...\r\n");
  setup_led();

  for (;;) {
    led_on();
    serial_putchar('/');
    delay(MCU_HZ/3/2);
    led_off();
    serial_putchar('\\');
    delay(MCU_HZ/3/2);
  }
}
