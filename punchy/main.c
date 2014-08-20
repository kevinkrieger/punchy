/* legacymsp430.h includes msp430.h and iomacros.h, it also defines
   other nice syntactic sugar, like interrupt(). */

#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>
#include "string.h"
#include "hc05.h"
#include "mpu6050.h"
#include "stdio.h"

/* Defines */
#define FOSC        1000000 // Oscillator speed in Hz

//extern inline void sendAck();

/* Data */
volatile uint8_t data_received;
//char *hello_message = "\r\nHello world! \r\n";
uint8_t TXByteCounter = 1;
uint8_t TXByte = 0x75;
uint8_t mpu6050_interrupt;
uint8_t i = 0;
uint16_t fifoCount = 0;
//uint8_t fifoBuffer[64];
uint8_t fifoBuffer[16];
uint16_t packetSize;
//uint8_t teapotPacket[14] = {'$',0x02,0,0,0,0,0,0,0,0,0x00,0x00,'\r','\n'};
uint8_t mpuIntStatus;
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
	mpu6050_interrupt = 0;
	mpu6050_init();
    hc05_transmit("mpu6050 initialized\r\n",21);
	_BIS_SR(GIE);
	while(1) {

        if(data_received != 0) {
            switch(data_received) {
                case 'T':
                    mpu6050_temp();
                    data_received = 0;
                    break;
                case 'A':
                    mpu6050_accel();
                    data_received = 0;
                    break;
                case 'G':
                    mpu6050_gyro();
                    data_received = 0;
                    break;
                case 'g':
                    mpu6050_calibrate_gyros();
                    data_received = 0;
                    break;
                case 'M':
                    mpu6050_getAddress();
                    data_received = 0;
                    break;
                case 'W':
                    mpu6050_wakeup();
                    data_received = 0;
                    break;
                case 'S':
                    mpu6050_sleep();
                    data_received = 0;
                    break;
                case 'R':
                    mpu6050_reset();
                    data_received = 0;
                    break;
                case 'd':
                    mpu6050_dmpinit();
                    data_received = 0;
                    break;
                case 'E':
                    mpu6050_setDMPEnabled(true);
                    P2DIR &= ~MPU6050_INT;  // Input
                    P2SEL &= ~MPU6050_INT;  // Digital IO Psel and psel2 are 0
                    P2SEL2 &= ~MPU6050_INT;
                    P2IES &= ~MPU6050_INT;  // Edge select 0 = low to high
                    P2IFG &= ~MPU6050_INT;  // Clear the interrupt flag before enabling interrupt
                    P2IE |= MPU6050_INT;    // Interrupt enable
                    data_received = 0;
                    break;
                case 'e':
                    mpu6050_setDMPEnabled(false);
                    data_received = 0;
                    break;
                default:
                    sendAck();
                    data_received = 0;
                    break;
            }
		}
		if(mpu6050_interrupt) {
 /*           fifoCount = mpu6050_getFIFOCount();
            sprintf(tempbuf,"F: %X\r\n",fifoCount);
            hc05_transmit(tempbuf,strlen(tempbuf));
            mpuIntStatus = mpu6050_getIntStatus();
            if(fifoCount > 1024 || mpuIntStatus & 0x10) {
                mpu6050_resetFIFO();
                hc05_transmit("FO\r\n",4);
            } else if(mpuIntStatus & 0x02) {
                while(fifoCount < packetSize) fifoCount = mpu6050_getFIFOCount();

                mpu6050_getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;
                                // display quaternion values in InvenSense Teapot demo format:
                teapotPacket[2] = fifoBuffer[0];
                teapotPacket[3] = fifoBuffer[1];
                teapotPacket[4] = fifoBuffer[4];
                teapotPacket[5] = fifoBuffer[5];
                teapotPacket[6] = fifoBuffer[8];
                teapotPacket[7] = fifoBuffer[9];
                teapotPacket[8] = fifoBuffer[12];
                teapotPacket[9] = fifoBuffer[13];
                //Serial.write(teapotPacket, 14);
                hc05_transmit(teapotPacket,14);
                teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
            }
   */
            fifoCount = mpu6050_getFIFOCount();
            sprintf(tempbuf,"FC: %X\r\n",fifoCount);
            hc05_transmit(tempbuf,strlen(tempbuf));
            if(fifoCount > 16) {
                fifoCount =16;
            }
            mpu6050_getFIFOBytes(mpu6050_buffer,fifoCount);
           /* This seems to keep the fifo operating. I probably need to read the fifo faster so it doesn't 'die' on me */
            mpu6050_resetFIFO();
            for(int i = 0; i<fifoCount;i++) {
                sprintf(tempbuf,"%X ",mpu6050_buffer[i]);
                hc05_transmit(tempbuf,3);
            }
            mpu6050_interrupt = 0;
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

interrupt (PORT2_VECTOR) port2_isr(void) {
    if(P2IFG & MPU6050_INT) {
        /* We got an edge interrupt on the INT pin */
        mpu6050_interrupt = 1;
        P2IFG &= ~MPU6050_INT;
        __bic_SR_register_on_exit(LPM0_bits);
    }
}
