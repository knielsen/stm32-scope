#include "stm32-scope.h"
#include "st7787.h"


/*
  ST7787 display controller.

  Pinout:
    CS       PD7 (FSMC)      VDDI --
    DC       PE6 (FSMC)     RESET -- PC15
    WR       PD5 (FSMC)       IM0 -- PC14
    RD       PD4 (FSCM)       IM1 -- PC13
    DB0      PD14 (FSMC)      IM2 -- PC12
    DB1      PD15 (FSMC)       TE -- PC11
    DB2      PD0 (FSMC)      DB17 --
    DB3      PD1 (FSMC)      DB16 --
    DB4      PE7 (FSMC)      DB15 --
    DB5      PE8 (FSMC)      DB14 --
    DB6      PE9 (FSMC)      DB13 --
    DB7      PE10 (FSMC)     DB12 --
    IM0      PC14            DB11 --
    IM1      PC13            DB10 --
    IM2      PC12             DB9 --
    TE       PC11             DB8 --
    RESET    PC15             DB7 -- PE10
                              DB6 -- PE9
                              DB5 -- PE8
                              DB4 -- PE7
                              DB3 -- PD1
                              DB2 -- PD0
                              DB1 -- PD15
                              DB0 -- PD14
                               RD -- PD4
                               WR -- PD5
                               DC -- PE6
                               CS -- PG9 (PD7 for 100 pin)
                              GND --
                              VDD --
                        (LED-)GND --
                             LED+ --
*/


#ifdef LILLE_VIDUNDER
/* On "det lille vidunder", there is an SRAM on bank 1, use bank 2. */
#define FSMC_BANK FSMC_Bank1_NORSRAM2
#define FSMC_BASE 0x64000000
#endif

#ifdef STM32F4_DISCOVERY
#define FSMC_BANK FSMC_Bank1_NORSRAM1
#define FSMC_BASE 0x60000000
#endif

#define ST7787_CS_ADR_BIT 22
#define ST7787_CMD_BYTE (*(volatile uint8_t *)FSMC_BASE)
#define ST7787_DATA_BYTE (*(volatile uint8_t *)(FSMC_BASE | (1<<ST7787_CS_ADR_BIT)))


uint8_t frame_buffer[320*240*3/2];


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


static void
fsmc_manual_init(void)
{
  FSMC_NORSRAMInitTypeDef fsmc_init;
  FSMC_NORSRAMTimingInitTypeDef timing, alttiming;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOD and E clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|
    GPIO_Pin_7|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|
    GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);

  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
  FSMC_NORSRAMDeInit(FSMC_BANK);
  FSMC_NORSRAMCmd(FSMC_BANK, DISABLE);

  fsmc_init.FSMC_Bank = FSMC_BANK;
  fsmc_init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  fsmc_init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  fsmc_init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
  fsmc_init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  fsmc_init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  fsmc_init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  fsmc_init.FSMC_WrapMode = FSMC_WrapMode_Disable;
  fsmc_init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  fsmc_init.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  fsmc_init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  fsmc_init.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
  fsmc_init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  fsmc_init.FSMC_ReadWriteTimingStruct = &timing;
  fsmc_init.FSMC_WriteTimingStruct = &alttiming;

  /* Timings. At MCU_HZ=168e6, one cycle is 5.95ns. */

  /* Read timing. */
  timing.FSMC_AddressSetupTime = 2;             /* Min 10 ns */
  timing.FSMC_AddressHoldTime = 0xf;
  timing.FSMC_DataSetupTime = 60;     /* Min 355 ns (45 ns for status regs) */
  /*
    Min NOE high time is 90 ns, 16 cycles. We take 2 cycles on address setup,
    so put the remaining 14 cycles as BUSTURN.
  */
  timing.FSMC_BusTurnAroundDuration = 14;
  timing.FSMC_CLKDivision = 0xf;
  timing.FSMC_DataLatency = 0xf;
  timing.FSMC_AccessMode = FSMC_AccessMode_A;

  /* Write timing. */
  alttiming.FSMC_AddressSetupTime = 3;          /* Min 10 ns */
  alttiming.FSMC_AddressHoldTime = 0xf;
  /* Data setup is spec'ed at 20 ns - but did not run stable below 7 cycles? */
  alttiming.FSMC_DataSetupTime = 7;
  /*
    Min write cycle time is spec'ed at 66 ns (15 MHz), which is ~11 cycles.
    We spend 3+7+1=10 cycles in ADDSET/DATAST, so no BUSTURN cycles.
  */
  alttiming.FSMC_BusTurnAroundDuration = 0;
  alttiming.FSMC_CLKDivision = 0xf;
  alttiming.FSMC_DataLatency = 0xf;
  alttiming.FSMC_AccessMode = FSMC_AccessMode_A;

  FSMC_NORSRAMInit(&fsmc_init);

  FSMC_NORSRAMCmd(FSMC_BANK, ENABLE);

#ifdef LILLE_VIDUNDER
  /* Take chip select permanently low for the SRAM chip, to avoid conflict. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_SetBits(GPIOD, GPIO_Pin_7);
#endif
}


void
setup_st7787_io(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* IM2/IM1/IM0/RESET on PC12-15 as output. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Assert RESET. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  /* Select 16-bit parallel interface on IM0-2. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);     /* IM1 <- 0   8/16 over 9/18 bits */
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);     /* IM0 <- 0   8 over 16 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_12);       /* IM2 <- 1   parallel interface */

  /* TE on PC11 as input. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Setup the data and control signals on FSMC. */
  fsmc_manual_init();

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


static inline uint8_t
st7787_read_cmd(void)
{
  return ST7787_CMD_BYTE;
}


static inline uint8_t
st7787_read_data(void)
{
  return ST7787_DATA_BYTE;
}


static inline void
st7787_write_cmd(uint8_t val)
{
  ST7787_CMD_BYTE = val;
}


static inline void
st7787_write_data(uint8_t val)
{
  ST7787_DATA_BYTE = val;
}


static void
display_command(uint8_t cmd, uint8_t *in, uint32_t in_len, uint8_t *out, uint32_t out_len)
{
  st7787_write_cmd(cmd);
  if (in_len) {
    do {
      st7787_write_data(*in++);
    } while (--in_len > 0);
  }
  if (out_len) {
    do
    {
      *out++ = st7787_read_data();
    } while (--out_len > 0);
  }
}


static void
display_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t *pixels)
{
  uint8_t buf[4];

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
  display_command(C_RAMWR, pixels, (3*w*h+1)/2, NULL, 0);
}


static void
display_cls(void)
{
  uint8_t buf[240];
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
  uint32_t i;
  uint8_t buf[128];

  serial_puts("ST7787 RDDID: ");
  display_command(C_RDDID, NULL, 0, buf, 4);
  serial_output_hexbyte(buf[0]);
  serial_puts(" ");
  serial_output_hexbyte(buf[1]);
  serial_puts(" ");
  serial_output_hexbyte(buf[2]);
  serial_puts(" ");
  serial_output_hexbyte(buf[3]);
  serial_puts("\r\n");

  /* Take the display out of sleep mode. */
  display_command(C_SLPOUT, NULL, 0, NULL, 0);
  /*
    Wait for sleep-out command to complete.
    Datasheet says 5 msec is enough before next command, but 120 msec is
    needed before the display is fully out of sleep mode.
  */
  delay_ms(120);
  /*
    Select 12-bit 444 RGB pixel format (mode 3).
    Select 16-bit mode for RGB mode (but we don't use it).
  */
  buf[0] = (13 << 4) | 3;
  display_command(C_COLMOD, buf, 1, NULL, 0);
  /* Clear the screen. */
  display_cls();
  /* Disable external vsync. */
  display_command(C_VSYNCOUT, NULL, 0, NULL, 0);
  /* Turn on the display */
  display_command(C_DISPON, NULL, 0, NULL, 0);

  /*
    Reprogram the lookup table to work for 12-bit colour mode.

    The lookup table is used when writing less than 18-bit colour into the
    dislay's framebuffer. The default table is set up for 16-bit colour, so
    in 12-bit mode the colours have only 1/2 intensity for red/blue and 1/4
    for green unless the table is initialised correctly.

    (Interestingly, the RGBSET command is not documented in ST7787 datasheet,
    except for a single small reference in a table heading. I found it though
    in the ILI9341 datasheet, and it seems to work fine on ST7787).
  */
  for (i = 0; i < 128; ++i) {
    if (i < 16)
      buf[i] = (i << 2) | (i >> 2);
    else if (i >= 32 && i < 32+16)
      buf[i] = ((i - 32) << 2) | ((i - 32) >> 2);
    else if (i >= 96 && i < 96+16)
      buf[i] = ((i - 96) << 2) | ((i - 96) >> 2);
    else
      buf[i] = 0;
  }
  display_command(C_RGBSET, buf, 128, NULL, 0);
}


void
st7787_test(void)
{
  uint32_t i, j;
  uint8_t buf[20*20*2];

  display_cls();

  serial_puts("Simple test,,,\r\n");

  memset(buf, 0xff, sizeof(buf));
  display_blit(0, 0, 20, 20, buf);
  display_command(C_RAMRD, NULL, 0, buf, 30);
  serial_puts("Display readback: ");
  for (i = 0; i < 30; ++i)
    serial_output_hexbyte(buf[i]);
  serial_puts("\r\n");
  delay_ms(1000);

  for (i = 0; i < 100; ++i) {
    uint8_t pixels[100*3/2];
    for (j = 0; j < 100*3/2; j+= 3) {
      uint8_t col_r = (i % 32) >> 1;
      uint8_t col_g = ((17*i)%64) >> 2;
      uint8_t col_b = ((57*i)%32) >> 1;
      pixels[j] = (col_r << 4) | col_g;
      pixels[j+1] = (col_b << 4) | col_r;
      pixels[j+2] = (col_g << 4) | col_b;
    }
    display_blit((i*119)%200, ((i*73)%300), 10, 10, pixels);
  }

  serial_puts("RDDST: ");
  display_command(C_RDDST, NULL, 0, buf, 5);
  serial_output_hexbyte(buf[0]);
  serial_puts(" ");
  serial_output_hexbyte(buf[1]);
  serial_puts(" ");
  serial_output_hexbyte(buf[2]);
  serial_puts(" ");
  serial_output_hexbyte(buf[3]);
  serial_puts(" ");
  serial_output_hexbyte(buf[4]);
  serial_puts("\r\n");
  delay_ms(1000);
}


/* Code for 12-bit local framebuffer. */
static void
frame_cls(void)
{
  memset(frame_buffer, 0, sizeof(frame_buffer));
}


static void
frame_transfer(void)
{
  /* ToDo: Wait for vertical blanking */
  uint8_t buf[4];
  buf[0] = 0 >> 8;
  buf[1] = 0 & 0xff;
  buf[2] = 239 >> 8;
  buf[3] = 239 & 0xff;
  display_command(C_CASET, buf, 4, NULL, 0);
  buf[0] = 0 >> 8;
  buf[1] = 0 & 0xff;
  buf[2] = 319 >> 8;
  buf[3] = 319 & 0xff;
  display_command(C_RASET, buf, 4, NULL, 0);
  display_command(C_RAMWR, frame_buffer, 240*320*3/2, NULL, 0);
}


/*
  The 8-bit interface of the st7787 packs 2 12-bit pixels into 3 consecutive
  bytes. The format is rrrrggggbbbb but big-endian, while cortex-M is little
  endian. So for pixels p,P we have: rrrrgggg bbbbRRRR GGGGBBBB which ends
  up in (unaligned) 16-bit as bbbb....rrrrgggg GGGGBBBB....RRRR. That's rather
  off, so probably better to just do two bytewise memory operations.
*/
static void
put_pixel(uint32_t x, uint32_t y, uint16_t col)
{
  uint32_t nibble_idx = (240*3)*y + 3*x;
  uint8_t *p = frame_buffer + nibble_idx/2;
  if (nibble_idx & 1) {
    p[0] = (p[0] & 0xf0) | (col >> 8);
    p[1] = col;
  } else {
    p[0] = col >> 4;
    p[1] = (p[1] & 0x0f) | (col << 4);
  }
}


void
display_render_adc(void)
{
  uint32_t i, j;
  uint16_t val;
  uint32_t height;
  uint32_t last_height, h1, h2;

  frame_cls();

  val = adc_sample_buffer[0];
  height = val*240/4096;
  put_pixel(height, 0, 0x026);
  last_height = height;

  for (i = 1; i < 320; ++i) {
    val = adc_sample_buffer[i];
    height = val*240/4096;
    if (height < last_height) {
      h1 = height; h2 = last_height;
    } else {
      h1 = last_height; h2 = height;
    }
    for (j = h1; j <= h2; ++j)
      put_pixel(j, i, 0x026);
    last_height = height;
  }
}


void
display_render_fft(void)
{
  uint32_t i, j;

  /* ToDo: For now hardcoded and assuming FFT size is 2048... */

  /* Frequency markers */
  for (i = 1; i < 15; ++i) {
    for (j = 0; j < 240; ++j) {
      if ((i % 5) == 0 || (j % 3) == 0)
        put_pixel(j, (i*10000*FFT_SIZE+SAMPLE_RATE)/(SAMPLE_RATE*3), 0xa33);
    }
  }

  for (i = 0; i < 320; ++i) {
    float v_min = fft_data[1+3*i];
    float v_max = v_min;
    uint32_t h1, h2;

    for (j = 1; j < 3; ++j) {
      float v = fft_data[1+3*i+j];
      if (v < v_min)
        v_min = v;
      else if (v > v_max)
        v_max = v;
    }
    h1 = 200*v_min;
    h2 = 200*v_max;
    if (h1 >= 240)
      h1 = 239;
    if(h2 >= 240)
      h2 = 239;
    for (j = h1; j <= h2; ++j)
      put_pixel(j, i, 0xff0);
  }
  frame_transfer();
}
