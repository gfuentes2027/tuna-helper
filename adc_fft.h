#ifndef ADC_FFT_H
#define ADC_FFT_H

#include <stdint.h>
#include "DSPLib.h"

#define NUM_SAMPLES       512U
#define SAMPLE_RATE_HZ    4000UL

extern _q15 voltages[NUM_SAMPLES];

void ADC_Volt_Init(void);
void Read_Volts(void);
void perform_fft(_q15 samples[]);
uint16_t get_fund_freq(void);

#endif
