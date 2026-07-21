#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include "adc_fft.h"

static void Clock_Init(void);
static void UART_Init(void);
static void UCA0_UART_TX_data(unsigned char data);
static void UCA0_UART_TX_string(const char *text);

int main(void)
{
    char buffer[48];

    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    Clock_Init();

    P1SEL0 |= BIT2;
    P1SEL1 |= BIT2;
    P1DIR &= ~BIT2;

    UART_Init();
    ADC_Volt_Init();
    UCA0_UART_TX_string("Guitar tuner ready\r\n");

    for (;;) {
        uint16_t frequency;

        Read_Volts();
        perform_fft(voltages);
        frequency = get_fund_freq();

        sprintf(buffer, "Fundamental Frequency: %d Hz\r\n", (int)frequency);
        UCA0_UART_TX_string(buffer);
    }
}

static void Clock_Init(void)
{
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__VLOCLK;
    CSCTL3 = DIVM__1 | DIVS__1 | DIVA__1;
    CSCTL0_H = 0U;
}

static void UART_Init(void)
{
    P2SEL1 |= BIT0 | BIT1;
    P2SEL0 &= ~(BIT0 | BIT1);

    UCA0CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA0BRW = 6U;
    UCA0MCTLW = UCOS16 | UCBRF_8 | 0x2000U;
    UCA0CTLW0 &= ~UCSWRST;
}

static void UCA0_UART_TX_data(unsigned char data)
{
    while ((UCA0IFG & UCTXIFG) == 0U) {
    }
    UCA0TXBUF = data;
}

static void UCA0_UART_TX_string(const char *text)
{
    while (*text != '\0') {
        UCA0_UART_TX_data((unsigned char)*text++);
    }
}
