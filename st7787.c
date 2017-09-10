#include "stm32-scope.h"
#include "st7787.h"


uint8_t frame_buffer[320*200*12/8];


static inline void
delay_ns(uint32_t ns)
{
  delay((MCU_HZ/(3*1000000)*ns+999)/1000);
}


static inline void
delay_ms(uint32_t ms)
{
  delay(MCU_HZ/(3*1000)*ms);
}


void
setup_st7787_io(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* CS/DC/WR/RD on PB12-15 as output. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Initialise CS/WR/RD all de-asserted, DC as command. */
  GPIO_SetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15);
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);

  /* IM2/IM1/IM0/RESET on PC12-15 as output. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Assert RESET. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  /* Select 16-bit parallel interface on IM0-2. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);     /* IM1 <- 0   8/16 over 9/18 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_14);       /* IM0 <- 1   16 over 8 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_12);       /* IM2 <- 1   parallel interface */

  /* TE on PC11 as input. PB16-17 on PC9-10 as input (for now). */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* PB0-15 on PD0-15 as input (for now). */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
    GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|
    GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Also set output speed etc., for quick change between input and output. */
  GPIOD->OTYPER = 0x00000000;                    /* 0 means push-pull */
  GPIOD->OSPEEDR = 0xaaaaaaaa;                  /* 0b10 means 50 MHz */
  GPIOD->PUPDR = 0x00000000;                    /* 0 means no pull */
  /*
    ToDo: DB16-17 (or DB1-17 in 1-bit serial mode) can be tied to GND or VCC,
    according to datasheet.
    But for now, let's leave them floating, to avoid risk of contention between
    MCU and display pins both in output mode.
  */

  /*
    After power-up, should hold RESET asserted for min 120 msec.
    Let do a 120 ms reset pulse, followed by a 20 us reset pulse.
    (The datasheet was a bit unclear, not sure if this wait is needed, or
    if a 20 us reset pulse is enough).
  */
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  delay_ms(1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  delay_ms(120);
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  delay_ms(1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  delay_ns(20*1000);
  /* Now release reset, taking the device into operational mode. */
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  /* Wait anoter 120 msec for RESET to complete. */
  delay_ms(120);
}


static inline void
assert_rd(void)
{
  my_gpio_reset(GPIOB, GPIO_Pin_15);
}


static inline void
deassert_rd(void)
{
  my_gpio_set(GPIOB, GPIO_Pin_15);
}


static inline void
assert_wr(void)
{
  my_gpio_reset(GPIOB, GPIO_Pin_14);
}


static inline void
deassert_wr(void)
{
  my_gpio_set(GPIOB, GPIO_Pin_14);
}


static inline void
assert_cs(void)
{
  my_gpio_reset(GPIOB, GPIO_Pin_12);
}


static inline void
deassert_cs(void)
{
  my_gpio_set(GPIOB, GPIO_Pin_12);
}


static inline void
dc_select_command(void)
{
  my_gpio_reset(GPIOB, GPIO_Pin_13);
}


static inline void
dc_select_data(void)
{
  my_gpio_set(GPIOB, GPIO_Pin_13);
}


static void
db_select_input_16(void)
{
  GPIOD->MODER = 0x00000000;                    /* 0b00 is input mode */
}


static void
db_select_output_16(void)
{
  GPIOD->MODER = 0x55555555;                    /* 0b01 is output mode */
}


static inline void
db_write16(uint32_t value)
{
  GPIOD->ODR = value & 0xffff;
}


static inline uint32_t
db_read16(void)
{
  return GPIOD->IDR & 0xffff;
}


static void
display_command(uint8_t cmd, uint16_t *in, uint32_t in_len, uint16_t *out, uint32_t out_len)
{
  assert_cs();

  /* Write the command byte. */
  dc_select_command();
  delay_ns(T_AST);
  db_select_output_16();
  db_write16(cmd);
  assert_wr();
  delay_ns(T_DST);
  deassert_wr();
  delay_ns(T_DHT);

  /* Write any command data. */
  if (in_len) {
    dc_select_data();
    delay_ns(T_AST);
    do {
      db_write16(*in++);
      assert_wr();
      delay_ns(T_DST);
      deassert_wr();
      delay_ns(T_DHT);
    } while (--in_len > 0);
  }

  db_select_input_16();

  /* Read reply. */
  if (out_len) {
    dc_select_data();
    delay_ns(T_AST);
    do
    {
      assert_rd();
      delay_ns(T_RAT);
      *out++ = db_read16();
      delay_ns(T_RC);   /* - T_RAT */
      deassert_rd();
      delay_ns(T_RDH);
    } while (--out_len > 0);
  }

  deassert_cs();
}


static void
display_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t *pixels)
{
  uint16_t buf[4];

  buf[0] = x >> 8;
  buf[1] = x & 0xff;
  buf[2] = (x+w-1) >> 8;
  buf[3] = (x+w-1) & 0xff;
  display_command(C_CASET, buf, 4, NULL, 0);
  buf[0] = y >> 8;
  buf[1] = y & 0xff;
  buf[2] = (y+h-1) >> 8;
  buf[3] = (y+h-1) & 0xff;
  display_command(C_RASET, buf, 4, NULL, 0);
  display_command(C_RAMWR, pixels, w*h, NULL, 0);
}


static void
display_cls(void)
{
  uint16_t buf[240];
  uint32_t i;

  memset(buf, 0, sizeof(buf));
  for (i = 0; i < 320; ++i) {
    display_blit(0, i, 120, 1, buf);
    display_blit(120, i, 120, 1, buf);
  }
}


void
st7787_init(void)
{
  uint16_t buf[1];

  /* Take the display out of sleep mode. */
  display_command(C_SLPOUT, NULL, 0, NULL, 0);
  /*
    Wait for sleep-out command to complete.
    Datasheet says 5 msec is enough before next command, but 120 msec is
    needed before the display is fully out of sleep mode.
  */
  delay_ms(120);
  /*
    Select 16-bit 565 RGB pixel format (mode 5).
    Same for RGB mode (but we don't use it).
  */
  buf[0] = (13 << 4) | 5;
  display_command(C_COLMOD, buf, 1, NULL, 0);
  /* Clear the screen. */
  display_cls();
  /* Disable external vsync. */
  display_command(C_VSYNCOUT, NULL, 0, NULL, 0);
  /* Turn on the display */
  display_command(C_DISPON, NULL, 0, NULL, 0);
}


void
st7787_test(void)
{
  uint32_t i, j;

  display_cls();
  for (i = 0; i < 100; ++i) {
    uint16_t pixels[100];
    for (j = 0; j < 100; ++j)
      pixels[j] = (i % 32) << 11 | ((17*i)%64) << 5 | ((57*i)%32);
    display_blit((i*119)%200, ((i*73)%300), 10, 10, pixels);
  }
}