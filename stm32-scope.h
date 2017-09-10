#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx.h>


#define MCU_HZ 168000000

#define LILLE_VIDUNDER

#define ADC_BUFFER_SIZE 64                      /* In 16-bit entries */


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


/* util.c */
extern void delay(uint32_t nCount);

/* dbg.c */
extern void setup_serial(void);
extern void serial_putchar(uint32_t c);
extern void serial_puts(const char *s);
extern void serial_output_hexbyte(uint8_t byte);
extern void println_uint32(uint32_t val);
extern void println_int32(int32_t val);
extern void print_uint32_hex(uint32_t val);
extern void println_uint32_hex(uint32_t val);
extern char *float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void serial_dump_buf(uint8_t *buf, uint32_t len);

/* led.c */
extern void setup_led(void);
extern void led_on(void);
extern void led_off(void);

/* adc.c */
extern volatile uint32_t adc_dma_buffers[2][ADC_BUFFER_SIZE*sizeof(uint16_t)/sizeof(uint32_t)];
static inline uint16_t adc_buf_val(uint32_t buf, uint32_t idx) {
  return ((volatile uint16_t *)&adc_dma_buffers[buf])[idx];
}
static inline float adc_val2voltage(uint32_t val) { return (float)val*(3.3f/4095.0f); }
extern void config_adc(void);
extern void config_adc_dma(void);
extern void adc_dma_start(void);
extern uint32_t adc_read(void);
extern uint32_t adc_vrefint_read(void);
extern float adc_voltage_read(void);
