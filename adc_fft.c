#include <msp430.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "DSPLib.h"
#include "adc_fft.h"

#define HAMMING_ALPHA  0.53836f
#define HAMMING_BETA   0.46164f
#define PI             3.1415926536f
#define MIN_FREQ_HZ    70UL
#define MAX_FREQ_HZ    400UL

DSPLIB_DATA(window, 4);
static _q15 window[NUM_SAMPLES];

DSPLIB_DATA(voltages, MSP_ALIGN_FFT_Q15(NUM_SAMPLES));
_q15 voltages[NUM_SAMPLES];

/* The sinusoid helper needs 3N/2 temporary samples. */
DSPLIB_DATA(temp, 4);
static _q15 temp[3U * NUM_SAMPLES / 2U];

static uint32_t fft_power[NUM_SAMPLES / 2U];
static bool window_initialized = false;

static void initHamming(void)
{
    msp_status status;
    msp_sub_q15_params subParams;
    msp_copy_q15_params copyParams;
    msp_fill_q15_params fillParams;
    msp_sinusoid_q15_params sinParams;

    sinParams.length = 3U * NUM_SAMPLES / 2U;
    sinParams.amplitude = _Q15(HAMMING_BETA);
    sinParams.cosOmega = _Q15(cosf(2.0f * PI / (NUM_SAMPLES - 1U)));
    sinParams.sinOmega = _Q15(sinf(2.0f * PI / (NUM_SAMPLES - 1U)));
    status = msp_sinusoid_q15(&sinParams, temp);
    msp_checkStatus(status);

    copyParams.length = NUM_SAMPLES;
    status = msp_copy_q15(&copyParams, &temp[NUM_SAMPLES / 4U], temp);
    msp_checkStatus(status);

    fillParams.length = NUM_SAMPLES;
    fillParams.value = _Q15(HAMMING_ALPHA);
    status = msp_fill_q15(&fillParams, window);
    msp_checkStatus(status);

    subParams.length = NUM_SAMPLES;
    status = msp_sub_q15(&subParams, window, temp, window);
    msp_checkStatus(status);
    window_initialized = true;
}

void ADC_Volt_Init(void)
{
    REFCTL0 |= REFVSEL_2 | REFON;
    while ((REFCTL0 & REFGENRDY) == 0U) {
    }

    /* Timer A0.1 triggers one conversion every 250 SMCLK ticks (4 kHz). */
    TA0CCR0 = (uint16_t)(1000000UL / SAMPLE_RATE_HZ - 1UL);
    TA0CCR1 = TA0CCR0 / 2U;
    TA0CCTL1 = OUTMOD_3;
    TA0CTL = TASSEL__SMCLK | MC__STOP | TACLR;

    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
    ADC12CTL1 = ADC12SHP | ADC12SHS_1 | ADC12CONSEQ_2;
    ADC12CTL2 = ADC12RES_2;
    ADC12MCTL0 = ADC12INCH_2 | ADC12VRSEL_1;
    ADC12CTL0 |= ADC12ENC;
}

void Read_Volts(void)
{
    uint16_t i;
    int32_t sum = 0;

    ADC12IFGR0 &= ~ADC12IFG0;
    ADC12CTL0 |= ADC12SC;
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;

    for (i = 0U; i < NUM_SAMPLES; ++i) {
        while ((ADC12IFGR0 & ADC12IFG0) == 0U) {
        }
        voltages[i] = (_q15)ADC12MEM0;
        sum += voltages[i];
        ADC12IFGR0 &= ~ADC12IFG0;
    }

    TA0CTL = TASSEL__SMCLK | MC__STOP | TACLR;

    {
        const int16_t average = (int16_t)(sum / (int32_t)NUM_SAMPLES);
        for (i = 0U; i < NUM_SAMPLES; ++i) {
            voltages[i] = (_q15)((int32_t)voltages[i] - average);
        }
    }
}

void perform_fft(_q15 samples[])
{
    uint16_t i;
    uint16_t shift;
    msp_status status;
    msp_fft_q15_params fftParams;
    msp_mpy_q15_params mpyParams;

    if (!window_initialized) {
        initHamming();
    }

    mpyParams.length = NUM_SAMPLES;
    status = msp_mpy_q15(&mpyParams, samples, window, samples);
    msp_checkStatus(status);

    fftParams.length = NUM_SAMPLES;
    fftParams.bitReverse = true;
    fftParams.twiddleTable = MAP_msp_cmplx_twiddle_table_2048_q15;
    status = msp_fft_auto_q15(&fftParams, samples, &shift);
    msp_checkStatus(status);

    for (i = 0U; i < NUM_SAMPLES / 2U; ++i) {
        const int32_t real = samples[2U * i];
        const int32_t imag = samples[2U * i + 1U];
        fft_power[i] = (uint32_t)(real * real + imag * imag);
    }
}

uint16_t get_fund_freq(void)
{
    uint16_t i;
    const uint16_t min_bin = (uint16_t)((MIN_FREQ_HZ * NUM_SAMPLES +
                                         SAMPLE_RATE_HZ - 1UL) / SAMPLE_RATE_HZ);
    const uint16_t max_bin = (uint16_t)((MAX_FREQ_HZ * NUM_SAMPLES) /
                                         SAMPLE_RATE_HZ);
    uint16_t peak_bin = min_bin;
    uint32_t peak_power = fft_power[min_bin];
    int16_t offset_q8 = 0;

    for (i = min_bin + 1U; i <= max_bin; ++i) {
        if (fft_power[i] > peak_power) {
            peak_power = fft_power[i];
            peak_bin = i;
        }
    }

    /* Parabolic interpolation gives a fractional-bin estimate in Q8. */
    if ((peak_bin > min_bin) && (peak_bin < max_bin)) {
        const int32_t left = (int32_t)fft_power[peak_bin - 1U];
        const int32_t center = (int32_t)fft_power[peak_bin];
        const int32_t right = (int32_t)fft_power[peak_bin + 1U];
        const int32_t denominator = left - 2L * center + right;

        if (denominator != 0L) {
            offset_q8 = (int16_t)(((left - right) * 128L) / denominator);
        }
    }

    return (uint16_t)(((int32_t)peak_bin * 256L + offset_q8) *
                       SAMPLE_RATE_HZ / ((int32_t)NUM_SAMPLES * 256L));
}
