#ifndef I2C_H
#define I2C_H
#include <stdint.h>
#include <legacymsp430.h>
#include <msp430g2553.h>

/* Defines */
typedef int bool;
#define true 1
#define false 0

#define SDA			BIT7
#define SCL			BIT6

#define I2C_SPEED_HZ 400000

/* Data */
uint8_t tx_to_rc;
uint8_t i2c_rx_buffer[32];
uint8_t i2c_rx_buffer_pointer;
uint8_t i2c_rx_buffer_length;
uint8_t i2c_tx_buffer[32];
uint8_t i2c_tx_buffer_counter;


/* I2C routines */
void i2c_init(void);

void i2c_transmit(void);

void i2c_receive(void);

void i2c_transmit_to_receive(void);

void i2c_multireceive(uint8_t amount);

void i2c_read_reg(uint8_t reg);

void i2c_write_reg(uint8_t reg, uint8_t value);

void i2c_read_bits(uint8_t reg,uint8_t bitStart, uint8_t length);

bool i2c_write_bit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
void i2c_write_bytes(uint8_t regAddr, uint8_t length, uint8_t* data);
void i2c_read_bytes(uint8_t regAddr, uint8_t length, uint8_t *data);
bool i2c_write_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool i2c_write_word(uint8_t regAddr, uint16_t data);
static inline void i2c_handle_state_interrupt(void) {
	if(UCB0STAT & UCNACKIFG) {
        //Need to send stop
		UCB0CTL1 |= UCTXSTP;
		UCB0STAT &= ~UCNACKIFG;
        // I2C state machine
        //	i2c_state_machine_interrupt();
	}
}

static inline void i2c_handle_TXRX_interrupt(void) {
  if(UCB0RXIFG & IFG2) { // Received full character on I2C
/*		if ( byteCtr == 0 ){
      UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
      *TI_receive_field = UCB0RXBUF;
      TI_receive_field++;
    }
    else {
      *TI_receive_field = UCB0RXBUF;
      TI_receive_field++;
      byteCtr--;
    }
	*///	mpu6050_buffer[mpu6050_buffer_pointer++] = UCB0RXBUF;
		//if(mpu6050_buffer_pointer == 16) mpu6050_buffer_pointer = 0;
		/*i2c_rx_buffer_length--;
		if(i2c_rx_buffer_length) {
            if(i2c_rx_buffer_length==1) {
                // send stop and wait for it to finish
                UCB0CTL1 |= UCTXSTP;
                while(UCB0CTL1 & UCTXSTP);
            } else {
                // read byte
                i2c_rx_buffer[i2c_rx_buffer_pointer++] = UCB0RXBUF;
                if(i2c_rx_buffer_pointer == 32) i2c_rx_buffer_pointer = 0;
            }
		} else {
            //read byte
            i2c_rx_buffer[i2c_rx_buffer_pointer++] = UCB0RXBUF;
            if(i2c_rx_buffer_pointer == 32) i2c_rx_buffer_pointer = 0;
		}*/
            i2c_rx_buffer[i2c_rx_buffer_pointer++] = UCB0RXBUF;
            if(i2c_rx_buffer_pointer == 32) i2c_rx_buffer_pointer = 0;

//		__bic_SR_register_on_exit(LPM0_bits);
	// I2c state machine
//		mpu6050_buffer[mpu6050_buffer_pointer] = UCB0RXBUF;
	//	mpu6050_buffer_pointer++;
	//	if(counterSize == 1) {
			// last byte read send stop
	//		UCB0CTL1 |= UCTXSTP;
		//	while(UCB0CTL1 &UCTXSTP);
		//}
	//	counterSize--;
		//i2c_state_machine_interrupt();
	} else if (UCB0TXIFG & IFG2) {	//Transmit buffer ready onn I2C
	/*if (byteCtr == 0){
      UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
      IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
    }
    else {
      UCB0TXBUF = *TI_transmit_field;
      TI_transmit_field++;
      byteCtr--;
    }*/
		if(i2c_tx_buffer_counter>0) {
			//UCB0TXBUF = mpu6050_tx_buffer[TXByteCounter-1];
			UCB0TXBUF = i2c_tx_buffer[i2c_tx_buffer_counter-1];
			//TXByteCounter--;
			i2c_tx_buffer_counter--;
//			__bic_SR_register_on_exit(LPM0_bits);
		/*} else if (i2c_tx_buffer_counter == 1){
			//UCB0TXBUF = mpu6050_tx_buffer[0];
			UCB0TXBUF = i2c_tx_buffer[0];
			UCB0CTL1 |= UCTXSTP;
			IFG2 &= ~UCB0TXIFG;
//			__bic_SR_register_on_exit(LPM0_bits);*/
		} else {// don't stop if we are receiving next
			if(tx_to_rc) {
				IFG2 &= ~UCB0TXIFG;
//				__bic_SR_register_on_exit(LPM0_bits);
			} else {
				UCB0CTL1 |= UCTXSTP;
				IFG2 &= ~UCB0TXIFG;
//				__bic_SR_register_on_exit(LPM0_bits);
			}
		}
	}
}

#endif
