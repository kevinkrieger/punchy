/* legacymsp430.h includes msp430.h and iomacros.h, it also defines
   other nice syntactic sugar, like interrupt(). */

#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>
#include "string.h"
#include "hc05.h"
#include "mpu6050.h"

/* Defines */
#define FOSC        1000000 // Oscillator speed in Hz

//extern inline void sendAck();

/* Data */
volatile uint8_t data_received;
char *hello_message = "\r\nHello world! \r\n";
uint8_t TXByteCounter = 1;
uint8_t TXByte = 0x75;

uint8_t i = 0;


/* Routines */



/* Initialize Clocks */
void initClocks(void) {
	/* Set 1mhz calibrated for clock
	 * The if statement will halt the processor if there is no
	 * calibration data (if it was erased, for example)*/
	if (CALBC1_1MHZ == 0xFF) {
		while(1);
	}
	DCOCTL = 0;
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;
	/* Set the ACLK to use the Very Low Power Low Frequency Oscillator (VLO)
	 * It is typically 12kHz. Selected by setting LFXT1Sx = 10 when XTS = 0.
	 * Divide ACLK with the DIVA0 and DIVA1 bits:
	 * 00 - /1 ** PUC value
	 * 01 - /2
	 * 10 - /4
	 * 11 - /8 -> Set this so we get 12kHz/8 = 1.5kHz*/
}


int main(void) {
    /* Configure Watch dog timer as an interval timer. WDTIFG is
     * set upon expiration of selected time interval, and PUC is not
     * generated, so there is no reset of the device. Also, WDTIE bit
     * remains unchanged, so you don't have to reset the WDT interrupt.
     *
     * WDTCTL is 16 bits and always needs to be accessed with
     * the upper 8 bits as the WDT password, WDTPW (0x5A).
     * Use ACLK for WDTCNT - selected with the WDTSSEL bit
     * Set WDTTMSEL bit to 1 for interval timer mode.
     * WDTIS0 and WDTIS1 set the interval.
     * 00 = WDT clock source/32768 **This is the PUC value
     * 01 = WDT clock source/8192
     * 10 = WDT clock source/512
     * 11 = WDT clock source/64
     * With ACLK = 1.5kHz and dividing it by 32768, we get ~21.8 seconds
     * With ACLK = 1.5kHz and dividing it by 64, we get 42.6mS
     * between WDT interrupts. */
	WDTCTL = WDTPW + WDTHOLD;
	initClocks();

	/* Configure Bluetooth module */
	hc05_init();

    /* Initialize mpu6050 */
	mpu6050_init();
    hc05_transmit("mpu6050 initialized\r\n",17);
	_BIS_SR(GIE);

	while(1) {

        switch(data_received) {
			case 'T':
				mpu6050_temp();
				break;
			case 'A':
				mpu6050_accel();
				break;
			case 'G':
				mpu6050_gyro();
				break;
            case 'M':
				mpu6050_getAddress();
				break;
			case 'W':
				mpu6050_wakeup();
				break;
			case 'S':
				mpu6050_sleep();
				break;
			case 'R':
				mpu6050_reset();
				break;
			default:
				sendAck();
				break;
		}

		  __bis_SR_register(LPM1_bits + GIE);
	}

}

interrupt (USCIAB0RX_VECTOR) receive_uart_I2C_state_isr(void) {
	// Interrupt for both i2c state and UART receive
	// UART receive
	if(IFG2 & UCA0RXIFG) {
		data_received = UCA0RXBUF;
		hc05_transmit((char*)&data_received, 1);
		__bic_SR_register_on_exit(LPM1_bits);
	} else {
        i2c_handle_state_interrupt();
        __bic_SR_register_on_exit(LPM0_bits);
	}
}

interrupt (USCIAB0TX_VECTOR) transmit_uart_I2C_data_isr(void) {
	// Interrupt for both i2c data and UART transmit
	// UART transmit
	if(IFG2 & UCA0TXIFG) {
		;//IFG2 &= ~UCA0TXIFG;
	}
	// I2C TXRX
	i2c_handle_TXRX_interrupt();
	__bic_SR_register_on_exit(LPM0_bits);
}
