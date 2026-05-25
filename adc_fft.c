#include <msp430.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "DSPLib.h"

#define HAMMING_ALPHA       0.53836
#define HAMMING_BETA        0.46164
#define PI                  3.1415926536

void ADC_Volt_Init(void);
int Read_Volts(void);
void perform_fft(void);
extern void initHamming(void);
int voltages[256];
const int num_samples = sizeof(voltages)/sizeof(voltages[0]);

DSPLIB_DATA(window,4);
_q15 window[num_samples];

//input FFT signal
DSPLIB_DATA(dsp_voltages, MSP_ALIGN_FFT_Q15(num_samples));
_q15 dsp_voltages[num_samples * 2];

/* Temporary data array for processing */
DSPLIB_DATA(temp,4);
_q15 temp[num_samples * 2];

void perform_fft(void)
{
    msp_fft_q15_params fftParams;
    uint16_t shift;
    msp_status status;
    msp_mpy_q15_params mpyParams;

    fftParams.length = num_samples;
    fftParams.bitReverse = 1;
    fftParams.twiddleTable = MAP_msp_cmplx_twiddle_table_2048_q15;

    msp_fft_auto_q15(&fftParams, voltages, &shift);

    for (i = 0; i < num_samples / 2; i++) {
        int16_t real = (int16_t)voltages[2 * i];
        int16_t imag = (int16_t)voltages[2 * i + 1];
    }

}

void ADC_Volt_Init(void)
{
    REFCTL0 |= REFVSEL_2 | REFON;
    while ((REFCTL0 & REFGENRDY) == 0);

    ADC12CTL0 = ADC12SHT0_8 | ADC12ON;
    ADC12CTL1 = ADC12SHP;
    ADC12CTL2 = ADC12RES_2;
    ADC12CTL3 = ADC12TCMAP;

    ADC12MCTL0 = ADC12INCH_2 | ADC12VRSEL_1;
    ADC12CTL0 |= ADC12ENC;
}

int Read_Volts(void)
{
    for (i = 0; i < 256; i++) {
         ADC12CTL0 |= ADC12SC;
        while (ADC12CTL1 & ADC12BUSY);
        unsigned int raw = ADC12MEM0;
        unsigned int mV = (unsigned long)raw * 2500UL / 4095UL;
        voltages[i] = mV;
    }

    return voltages;
}

void initHamming(void)
{
    msp_status status;
    msp_sub_q15_params subParams;
    msp_copy_q15_params copyParams;
    msp_fill_q15_params fillParams;
    msp_sinusoid_q15_params sinParams;

    sinParams.length = 3*num_samples/2;
    sinParams.amplitude = _Q15(HAMMING_BETA);
    sinParams.cosOmega = _Q15(cosf(2*PI/(num_samples-1)));
    sinParams.sinOmega = _Q15(sinf(2*PI/(num_samples-1)));
    status = msp_sinusoid_q15(&sinParams, temp);
    msp_checkStatus(status);

    copyParams.length = num_samples;
    status = msp_copy_q15(&copyParams, &temp[num_samples/4], &temp[0]);
    msp_checkStatus(status);

    fillParams.length = num_samples;
    fillParams.value = _Q15(HAMMING_ALPHA);
    status = msp_fill_q15(&fillParams, window);
    msp_checkStatus(status);

    subParams.length = num_samples;
    status = msp_sub_q15(&subParams, window, temp, window);
    msp_checkStatus(status);
}