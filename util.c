#include "stm32-scope.h"

void
delay(uint32_t nCount)
{
  /* This should be 3 cycles per iteration. nCount must be > 0. */
  __asm volatile
    ("\n"
     "0:\n\t"
     "subs %1, #1\n\t"
     "bne.n 0b"
     : "=r" (nCount)
     : "0" (nCount)
     : "cc");
}
