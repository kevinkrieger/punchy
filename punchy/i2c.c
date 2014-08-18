#include "i2c.h"
#include <legacymsp430.h>
#include <msp430g2553.h>
/* I2C routines */

void i2c_init() {
  dint();
  UCB0CTL1 |= UCSWRST; //Keep in reset until configured
  UCB0CTL0 = UCMODE0 + UCMODE1 + UCMST + UCSYNC; // Master mode, I2C mode, synchronous mode
  //7 bit addressing
  UCB0CTL1 = UCSWRST + UCSSEL_2;
  UCB0BR0 = 24;
  UCB0BR1 = 0;
  UCB0I2CSA = 0x68; // UCB0I2CSA contains the slave address (7 or 10  bit) of the slave
  // that the msp430 is talking to over I2C bus.

  UCB0CTL1 &= ~(UCSWRST); // need to unset the reset bit before setting interrupts
  UCB0I2CIE = UCNACKIE; // Interrupt when NACK occurs but ACK was expected
     // there is also stop condition, start condition for when msp430 is slave
     // and arbitration lost interrupt when multimaster mode is set
   // IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);
  IE2 |=  UCB0RXIE +UCB0TXIE; // interrupt when USCI_B0 UCB0TXBUF is empty and when UCB0RXBUF has a complete character

 // _enable_interrupts();
  eint();
}

void i2c_start() {

}

void i2c_stop() {
    UCB0CTL1 |= UCTXSTP;
    while(UCB0CTL1 & UCTXSTP);
}

void i2c_write(uint8_t byte) {

}


void i2c_tx_init(){

}


void i2c_rx_init() {

}
