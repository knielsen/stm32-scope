#include "stm32-scope.h"

#include <arm_math.h>

static arm_rfft_fast_instance_f32 arm_fft_cfg PLACE_IN_CCM;
/*
  This holds the input data for the FFT.
  We might place this shared with the frame-buffer, if we are really
  tight for space, and are going to re-draw from scratch after the FFT.
*/
static float fft_buf[FFT_SIZE] PLACE_IN_CCM;
/*
  This holds the magnitude output from the FFT. Only the first half 0<=i<=N/2
  is stored, as the upper half is symmetric and above the Nykvist frequency, so
  of little use.
*/
float fft_data[FFT_SIZE] PLACE_IN_CCM;


void init_fft() {
  arm_rfft_fast_init_f32(&arm_fft_cfg, FFT_SIZE);
}


//#define TIME_FFT

void fft_sample_buf(void)
{
  uint32_t i;
  float middle;
#ifdef TIME_FFT
  uint32_t t0, t1, t2, t3;
#endif

#ifdef TIME_FFT
  t0 = get_time();
#endif
  for (i = 0; i < FFT_SIZE; ++i)
    fft_buf[i] = adc_val2voltage(adc_sample_buffer[i]);

#ifdef TIME_FFT
  t1 = get_time();
#endif
  arm_rfft_fast_f32(&arm_fft_cfg, fft_buf, fft_data, 0);
#ifdef TIME_FFT
  t2 = get_time();
#endif

  // fft_data[0] is real. fft_data[1] is real also and stores the [N/2] bin.
  middle = fft_data[1];
  for (i = 1; i < FFT_SIZE/2; ++i) {
    float re = fft_data[2*i];
    float im = fft_data[2*i+1];
    float val = sqrtf(re*re+im*im)/(float)FFT_SIZE;
    fft_data[i] = val;
  }
  /*
    Could do this if we really want, but it is redundant due to symmetry, and
    of limited use since it is all above the Nykvist frequency.

  for (i = 1 ; i < FFT_SIZE/2; ++i)
    fft_data[FFT_SIZE-i] = fft_data[i];
  */

  fft_data[FFT_SIZE/2] = middle;
#ifdef TIME_FFT
  t3 = get_time();
#endif

#ifdef TIME_FFT
  serial_puts("Total FFT cycles: "); println_uint32(calc_time_from_val(t0, t3));
  serial_puts("      FFT cycles: "); println_uint32(calc_time_from_val(t1, t2));
  serial_puts("      pre cycles: "); println_uint32(calc_time_from_val(t0, t1));
  serial_puts("     post cycles: "); println_uint32(calc_time_from_val(t2, t3));
#endif
}
