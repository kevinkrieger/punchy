#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>

/* Initialize delay timer */
//void init_delay_timer(void) {
// Timer A div 8

//}

/* Delay a number of milliseconds using timer a, up to 256 ms */
void delay_ms(uint8_t milliseconds) {
// 1ms will be CLOCK/1000 edges of clock.
// Clock is one of ACLK, SMCLK or external TACLK or INCLK using TASSELx bits
// It can be divided by 1,2,4 or 8 using IDx bits.
// When TAR (16 bits) reaches the value in TACCR2 an interrupt can be generated.

// Timer A compare 0 mode (CAP = 0), capture rising edges
TACCTL0 = CM_1;
TACCR0 = (milliseconds*248); //Value to count to for milliseconds delay! 248 is tweaked to account for instructions in this routine
// Timer A smclk, Timer A div 8 (DIVx = 3), up mode to TACCR0!
//Use SMCLK but change divider to it to 8 for this routine. Also change div to TAR to 8,
// then we get FOSC/64 clock to TA. 1 ms would be FOSC/64000 (@ 16MHz - 250 edges!) lets approximate with 256
BCSCTL2 |= DIVS_3;
TACTL = TASSEL_2 + ID_3 + MC_1;

// Wait for the interrupt...
while(!(TACCTL0&CCIFG));
BCSCTL2 &= ~DIVS_3; // Set SMCLK back to the way it was

TACTL |= TACLR; // reset timer (TAR, clock div, count direction)
//TACTL &= ~(MC0 | MC1); // Stop mode, halt the timer
//TACTL = 0; // Reset timer
//TAR = 0;
//TACCTL0 = 0;
//TACCR0 = 0;

}

/* Delay a number of seconds using timer a */
void delay_s(uint8_t seconds) {

}
