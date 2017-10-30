#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx.h>


#define MCU_HZ 168000000

#define LILLE_VIDUNDER
#define DBG_RX

#define ADC_BUFFER_SIZE 64                      /* In 16-bit entries */

#define SAMPLE_BUFFER_SIZE 4096

#define ST7787_W 320
#define ST7787_H 240


#ifdef STM32F4_DISCOVERY
#define LED_PERIPH RCC_AHB1Periph_GPIOD
#define LED_GPIO GPIOD
#define LED_PIN GPIO_Pin_12
#endif

#ifdef LILLE_VIDUNDER
#define LED_PERIPH RCC_AHB1Periph_GPIOG
#define LED_GPIO GPIOG
#define LED_PIN GPIO_Pin_15
#endif


/*
  Place data in core-coupled memory.
  This data will not be initialised by startup code.
  And it is not available to peripherals (such as DMA).
*/
#define PLACE_IN_CCM __attribute__ ((section(".uninit_ccm_data")))


/* Fast access to GPIO. */
static inline void
my_gpio_set(GPIO_TypeDef *gpio, uint32_t bits)
{
  gpio->BSRRL = bits;
}


static inline void
my_gpio_reset(GPIO_TypeDef *gpio, uint32_t bits)
{
  gpio->BSRRH = bits;
}


/*
  Writing a single bit in a GPIO, using bit-banding.
  BIT_NUMBER is the bit (0..15). VAL is 0 or 1.
*/
static inline void
my_gpio_write_one_bit(GPIO_TypeDef *gpio, uint32_t bit_number, uint32_t val)
{
  static const uint32_t bit_band_base = PERIPH_BB_BASE;  /* 0x42000000 */
  static const uint32_t periph_base = PERIPH_BASE;       /* 0x40000000 */
  uint32_t byte_offset = (uint32_t)gpio - periph_base + offsetof(GPIO_TypeDef, ODR);
  uint32_t word_addr = bit_band_base | (byte_offset<<5) | (bit_number<<2);
  *(volatile uint32_t *)word_addr = val;
}


/*
  Read a single bit in a GPIO, using bit-banding.
  BIT_NUMBER is the bit (0..15). Returns 0 or 1.
*/
static inline uint32_t
my_gpio_read_one_bit(GPIO_TypeDef *gpio, uint32_t bit_number)
{
  static const uint32_t bit_band_base = PERIPH_BB_BASE;  /* 0x42000000 */
  static const uint32_t periph_base = PERIPH_BASE;       /* 0x40000000 */
  uint32_t byte_offset = (uint32_t)gpio - periph_base + offsetof(GPIO_TypeDef, ODR);
  uint32_t word_addr = bit_band_base | (byte_offset<<5) | (bit_number<<2);
  return *(volatile uint32_t *)word_addr;
}


static inline uint32_t
get_time(void)
{
  return SysTick->VAL;
}
static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


/* util.c */
extern void delay(uint32_t nCount);

/* dbg.c */
extern void setup_serial(void);
extern void serial_putchar(uint32_t c);
extern void serial_puts(const char *s);
extern void serial_output_hexbyte(uint8_t byte);
extern char *tostr_uint32(char *buf, uint32_t val);
extern void println_uint32(uint32_t val);
extern void println_int32(int32_t val);
extern void print_uint32_hex(uint32_t val);
extern void println_uint32_hex(uint32_t val);
extern char *float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void serial_dump_buf(uint8_t *buf, uint32_t len);
#ifdef DBG_RX
extern int serial_rx(void);
#endif

/* led.c */
extern void setup_led(void);
extern void led_on(void);
extern void led_off(void);

/* adc.c */

/*
  Note: Only a few sample rates are available in STM32F4:
    3, 15, 28, 56, 84, 112, 144, 480
*/
#define ADC_STM32_SAMPLE_CYCLES 56
#define ADC_STM32_SAMPLETIME_CONFIG_MK2(cycles) ADC_SampleTime_ ## cycles ## Cycles
#define ADC_STM32_SAMPLETIME_CONFIG_MK(cycles) ADC_STM32_SAMPLETIME_CONFIG_MK2(cycles)
#define ADC_STM32_SAMPLETIME_CONFIG ADC_STM32_SAMPLETIME_CONFIG_MK(ADC_STM32_SAMPLE_CYCLES)

/*
  ADC clock depends on APB2 frequency (84 MHz max) and ADC clock divider.
  Max ADC clock is spec'ed at 30 MHz, so from 84 MHz a divisor of 4 gives the
  maximum valid clock of 21 MHz.
*/
#define ADC_STM32_CLOCK_DIVIDER ADC_Prescaler_Div4
#define ADC_STM32_CLOCK 21000000
/*
  Sample rate comes from sample time plus 12 cycles for conversion
  (at the maximum resolution of 12 bit).
  The SAMPLE_RATE value is in units of samples per second.
*/
#define SAMPLE_RATE (ADC_STM32_CLOCK / (12 + ADC_STM32_SAMPLE_CYCLES))

extern volatile uint16_t adc_dma_buffers[2][ADC_BUFFER_SIZE];
extern volatile uint16_t adc_sample_buffer[SAMPLE_BUFFER_SIZE];
static inline uint16_t adc_buf_val(uint32_t buf, uint32_t idx) {
  return adc_dma_buffers[buf][idx];
}
static inline float adc_val2voltage(uint32_t val) { return (float)val*(3.3f/4095.0f); }
extern void config_adc(void);
extern void config_adc_dma(void);
extern void adc_dma_start(void);
extern uint32_t adc_read(void);
extern uint32_t adc_vrefint_read(void);
extern float adc_voltage_read(void);
extern void adc_start_sample_with_trigger(uint16_t level, uint8_t rising, uint8_t falling);
extern int adc_triggered(void);

/* st7787.c */
extern void setup_st7787_io(void);
extern void st7787_init(void);
extern void st7787_test(void);
extern void display_render_adc(void);
extern void display_render_fft(void);
extern void display_render_3d();

/* fft.c */
#define FFT_SIZE 2048
extern float fft_data[FFT_SIZE];
extern void init_fft();
extern void fft_sample_buf(void);

/* stm32-scope.c */
extern uint8_t trigger_enabled;
extern uint8_t mode_3d;
extern uint16_t trigger_level;
