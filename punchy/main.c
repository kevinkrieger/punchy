/* legacymsp430.h includes msp430.h and iomacros.h, it also defines
   other nice syntactic sugar, like interrupt(). */

#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>
#include "string.h"
#include "hc05.h"
#include "mpu6050.h"
#include "stdio.h"
#include "utility.h"

/* Data */
volatile uint8_t data_received;
uint8_t TXByteCounter = 1;
uint8_t TXByte = 0x75;
uint8_t mpu6050_interrupt = 0;
uint8_t i = 0;
uint16_t fifoCount = 0;
uint16_t packetSize;
char teapotPacket[14] = {'$',0x02,0,0,0,0,0,0,0,0,0x00,0x00,'\r','\n'};
char mpuIntStatus;
uint8_t motion_detect_mode = 0;
uint8_t dmp_mode = 0;


/* Routines */

/* Initialize Clocks */
void initClocks(void) {
	/* Set calibrated clock
	 * The if statement will halt the processor if there is no
	 * calibration data (if it was erased, for example)*/
	if (CALBC1_16MHZ == 0xFF) {
		while(1);
	}
	DCOCTL = 0;
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	/* Set the ACLK to use the Very Low Power Low Frequency Oscillator (VLO)
	 * It is typically 12kHz. Selected by setting LFXT1Sx = 10 when XTS = 0.
	 * Divide ACLK with the DIVA0 and DIVA1 bits:
	 * 00 - /1 ** PUC value
	 * 01 - /2
	 * 10 - /4
	 * 11 - /8 -> Set this so we get 12kHz/8 = 1.5kHz*/
	 BCSCTL3 &= ~(LFXT1S0 + LFXT1S1); // disable VLO (12kHz very low power internal oscillator for ACLK)
	 // enable it when you need it by setting LFXT1S1
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
	hc05_init(__baud_to_uca0br(9600));
    hc05_transmit("HC05 init\r\n",11);

    /* Initialize mpu6050 */
	mpu6050_init();
    hc05_transmit("mpu6050 init\r\n",14);

    /* Now we are ready for application code to run. Enable interrupts */
	_BIS_SR(GIE);

	while(1) {

        if(data_received != 0) {
            switch(data_received) {
                case 'T':
                data_received = 0;
                    mpu6050_temp();

                    break;
                case 'A':
                    data_received = 0;

                    mpu6050_accel();
                    break;
                case 'G':
                    data_received = 0;

                    mpu6050_gyro();
                    break;
                case 'g':
                    data_received = 0;
                    mpu6050_calibrate_gyros();
                    break;
                case 'M':
                    data_received = 0;
                    mpu6050_getAddress();
                    break;
                case 'W':
                    data_received = 0;
                    mpu6050_wakeup();
                    break;
                case 'S':
                    data_received = 0;
                    mpu6050_sleep();
                    break;
                case 'R':
                    data_received = 0;
                    dmp_mode = 0;
                    motion_detect_mode = 0;
                    mpu6050_reset();
                    break;
                case 'd':
                    data_received = 0;
                    mpu6050_dmpinit();
                    break;
                case 'E':
                    data_received = 0;
                    motion_detect_mode = 0;
                    dmp_mode = 1;
                    mpu6050_setDMPEnabled(true);
                    P2DIR &= ~MPU6050_INT;  // Input
                    P2SEL &= ~MPU6050_INT;  // Digital IO Psel and psel2 are 0
                    P2SEL2 &= ~MPU6050_INT;
                    P2IES &= ~MPU6050_INT;  // Edge select 0 = low to high
                    P2IFG &= ~MPU6050_INT;  // Clear the interrupt flag before enabling interrupt
                    P2IE |= MPU6050_INT;    // Interrupt enable
                    //mpu6050_resetFIFO();
                    break;
                case 'e':
                    dmp_mode = 0;
                    data_received = 0;
                    mpu6050_setDMPEnabled(false);
                    break;
                case 'm': /* Itseems that I can have motion detect interrupts if I first call the dmpinit() function, then this code is run. I can probably narrow it down
                to a certain function call in the dmpinit() it will just take time */
                    sendAck();
                    motion_detect_mode = 1;
                    dmp_mode = 0;
                    mpu6050_setDMPEnabled(false);
                    i2c_write_reg(MPU6050_RA_INT_PIN_CFG,0x10);//interrupt status cleared on any read
                    //i2c_write_reg(MPU6050_RA_MOT_DETECT_CTRL,0x30); // add the 3 ms delay to accel put
                    mpu6050_setMotionDetectionThreshold(0x01);//not sure... but I'm told it's 2mg per LSB so 0xFF would only be about 0.512g
                    mpu6050_setMotionDetectionDuration(40);
                    mpu6050_setIntEnabled(0x40);//motion detect... based on the product specification document, I don't think motion detect can generate an interrupt on INT pin,
                    // so we also set the data ready interrupt.
                   mpu6050_configAccel(MPU6050_ACCEL_FS_16<<(MPU6050_ACONFIG_AFS_SEL_BIT-1));
                    data_received = 0;
                    P2DIR &= ~MPU6050_INT;  // Input
                    P2SEL &= ~MPU6050_INT;  // Digital IO Psel and psel2 are 0
                    P2SEL2 &= ~MPU6050_INT;
                    P2IES &= ~MPU6050_INT;  // Edge select 0 = low to high
                    P2IFG &= ~MPU6050_INT;  // Clear the interrupt flag before enabling interrupt
                    P2IE |= MPU6050_INT;    // Interrupt enable
                    /*while(1) {

                        if(mpu6050_getIntStatus() & 0x40) {
                            //motion interrupt
                            hc05_transmit("Motion\r\n",8);
                        }
                    }*/

                    break;
              //  case 'h':
                //    hc05_setspeed(115200);
                //    data_received = 0;
                //    break;
                //case 'k':
                //    hc05_key();
                //    data_received = 0;
               //     break;
                default:
                    sendAck();
                    data_received = 0;
                    break;
            }
		}
		if(mpu6050_interrupt) {
            if(dmp_mode) {
                mpuIntStatus = mpu6050_getIntStatus();
                fifoCount = mpu6050_getFIFOCount();
                if(fifoCount > 16) {
                    fifoCount =16;
                }
                mpu6050_getFIFOBytes(mpu6050_buffer,fifoCount);
               /* This seems to keep the fifo operating. I probably need to read the fifo faster so it doesn't 'die' on me */
                mpu6050_resetFIFO();

                /* From J.Rowberg's library, the dmp packet output is:
                bytes 0-15 quaternion (32 bits)  (w,x,y,z) but just use the first two bytes as 16 bit number
                bytes 16-27 gyro (32 bits) (gx,gy,gz) but just use the first two bytes as 16 bit number
                bytes 28-39 acceleration (32 bits) (ax,ay,az) but just use the first two bytes as 16 bit number
                */

                teapotPacket[2] = mpu6050_buffer[0];
                teapotPacket[3] = mpu6050_buffer[1];
                teapotPacket[4] = mpu6050_buffer[4];
                teapotPacket[5] = mpu6050_buffer[5];
                teapotPacket[6] = mpu6050_buffer[8];
                teapotPacket[7] = mpu6050_buffer[9];
                teapotPacket[8] = mpu6050_buffer[12];
                teapotPacket[9] = mpu6050_buffer[13];
                teapotPacket[10] = mpuIntStatus; // I modified the packet to sent the interrupt status in this byte
                hc05_transmit(teapotPacket,14);
                 teapotPacket[11]++;
                mpu6050_interrupt = 0;

            } else if (motion_detect_mode) {
                if(mpu6050_getIntStatus() & 0x40) {
                    // Disable motion interrupts, get accel and gyro values until they aren't interesting
                    // anymore, then quit and enable interrupt.
                    mpu6050_setIntEnabled(0x00);
                    //motion interrupt
                  //  hc05_transmit("Motion\r\n",8);
                    for(uint8_t j = 0; j<200; j++) {
                        mpu6050_accel();

                        //__delay_us(1000);
                        delay_ms(1); // For some reason, using this delay_ms(5) or 1 ms, the output in mpu6050_accel() is not right...
                        // mostly the \r\n are cut off. I have no idea why, perhaps due to modifying the smclk? a test will be to wait at
                        // the end of the transmit function!

                    }

                    mpu6050_setIntEnabled(0x40);
                }
            }
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
