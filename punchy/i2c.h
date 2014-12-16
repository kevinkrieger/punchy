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
	}
}

static inline void i2c_handle_TXRX_interrupt(void) {
    if(UCB0RXIFG & IFG2) { // Received full character on I2C
        i2c_rx_buffer[i2c_rx_buffer_pointer++] = UCB0RXBUF;
        if(i2c_rx_buffer_pointer == 32) i2c_rx_buffer_pointer = 0;
	} else if (UCB0TXIFG & IFG2) {	//Transmit buffer ready on I2C
		if(i2c_tx_buffer_counter>0) {
			UCB0TXBUF = i2c_tx_buffer[i2c_tx_buffer_counter-1];
			i2c_tx_buffer_counter--;
		} else {// don't stop if we are receiving next
			if(tx_to_rc) {
				IFG2 &= ~UCB0TXIFG;
			} else {
				UCB0CTL1 |= UCTXSTP;
				IFG2 &= ~UCB0TXIFG;
			}
		}
	}
}

#endif
