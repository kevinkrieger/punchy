/*
  usci_i2c.c - Modified from usi_i2c.c by Kevin Krieger to work with USCI_B
  Module. Original by Jan Rychter (see copyright below)


  Copyright (C) 2013 Jan Rychter

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <msp430g2553.h>
#include <legacymsp430.h>
#include <stdint.h>
#include "usci_i2c.h"

// Internal state
static uint16_t *i2c_sequence;
static uint16_t i2c_sequence_length;
static uint8_t *i2c_receive_buffer;
static uint16_t i2c_wakeup_sr_bits;
static uint16_t i2c_slave_address;
i2c_state_type i2c_state = I2C_IDLE;
//counterSize = 0;

static inline void i2c_send_stop();
static inline void i2c_prepare_data_xmit_recv();

void i2c_send_sequence(uint16_t *sequence, uint16_t sequence_length, uint8_t *received_data,  uint16_t wakeup_sr_bits, uint16_t slave_address) {
  while(i2c_state != I2C_IDLE); // we can't start another sequence until the current one is done
  i2c_sequence = sequence;
  i2c_sequence_length = sequence_length;
  i2c_receive_buffer = received_data;
  i2c_wakeup_sr_bits = wakeup_sr_bits;
  i2c_state = I2C_START_XMIT;
  i2c_slave_address = slave_address;
  //USICTL1 |= USIIFG;            // actually start communication
  UCB0I2CSA = slave_address; //Send start condition to this slave address
  UCB0CTL1 |= UCTR + UCTXSTT; //Start condition and transmitter mode
}
/*void i2c_receive_sequence(uint16_t *received_data, uint16_t wakeup_sr_bits, uint16_t slave_address) {
  while(i2c_state != I2C_IDLE);
  i2c_receive_buffer = received_data;
  i2c_wakeup_sr_bits = wakeup_sr_bits;
  i2c_state = I2C_START_RECV;
  UCB0I2CSA = slave_address;
  UCB0CTL1 &= ~UCTR;
  UCB0CTL1 |= UCTXSTT; // Set slave address, receiver mode and send start condition
  // UCB0TXIFG is set when we can put data into UCB0TXBUF
}
*/
static inline void i2c_send_stop() {
  /*USICTL0 |= USIOE;             // SDA = output
  USISRL = 0x00;
  USICNT |=  0x01;              // Bit counter= 1, SCL high, SDA low
  */
  UCB0CTL1 |= UCTXSTP;
  while(UCB0CTL1 & UCTXSTP); // Wait for stop to be transmitted (bit is cleared by hdw)
  i2c_state = I2C_IDLE;
//  if(i2c_wakeup_sr_bits) {
      //_bic_SR_register_on_exit(i2c_wakeup_sr_bits); // exit active if prompted to
 //   	_BIC_SR_IRQ(i2c_wakeup_sr_bits);
	//}
}

static inline void i2c_prepare_data_xmit_recv() {
 if(UCB0CTL1 & UCTR) { // Transmitter mode
  if(i2c_sequence_length == 0) {
    i2c_send_stop();    // nothing more to do,  send STOP and we go back to IDLE
  } else {
    if(*i2c_sequence == I2C_RESTART) {

   /*   USICTL0 |= USIOE;         // SDA = output
      USISRL = 0xff;            // prepare and send a dummy bit, so that SDA is high
      USICNT = (USICNT & 0xE0) | 1;*/
      UCB0I2CSA = i2c_slave_address; //Send start condition to this slave address
      UCB0CTL1 &= ~UCTR; // receive next bytes and send start
      UCB0CTL1 |= UCTXSTT;
      if(i2c_sequence_length == 1) {
        while(UCB0CTL1 & UCTXSTT);
        UCB0CTL1 |= UCTXSTP;
      } // Last one! need to set stop while receiving.
      i2c_sequence++;
      i2c_sequence_length--;
      i2c_state = I2C_START_RECV; // When first byte is received, UCB0RXIFG is set, UCNACKIFG is set if slave didn't acknowledge address
    }
    else if(*i2c_sequence == I2C_READ) {
/*      USICTL0 &= ~USIOE;               // SDA = input
      USICNT = (USICNT & 0xE0) | 8;    // Bit counter = 8, RX data*/
          i2c_sequence++;
    i2c_sequence_length--;
      i2c_state = I2C_RECEIVED_DATA;   // next state: Test data and ACK/NACK
    } else {                           // a write
      // at this point we should have a pure data byte, not a command, so (*i2c_sequence >> 8) == 0
     /* USICTL0 |= USIOE;                // SDA = output
      USISRL = (char)(*i2c_sequence);  // Load data byte
      USICNT = (USICNT & 0xE0) | 8;    // Bit counter = 8, start TX*/
      UCB0TXBUF = (char)(*i2c_sequence); // put data into buffer, TXIF is set when it goes out
          i2c_sequence++;
    i2c_sequence_length--;
      i2c_state = I2C_HANDLE_TX; // next state: receive data ACK/NACK
    }

  }
  } else {
    // Receive mode, we have received a byte
    *i2c_receive_buffer = UCB0RXBUF;
    i2c_receive_buffer++;
  //   if(i2c_sequence_length > 1) {
    if(i2c_sequence_length==0) {
      // Last one so we finish up by pollin stop bit and setting i2c state to IDLE
      while(UCB0CTL1 & UCTXSTP);
      i2c_sequence++;
      i2c_sequence_length--;
      i2c_state = I2C_IDLE;
    } else {
      i2c_sequence++;
      i2c_sequence_length--;
      // If this is not the last byte
      i2c_state = I2C_HANDLE_RX;  // Go to next state: data/rcv again
   // } else {                        // last byte: send NACK
     // i2c_state = I2C_STOP; // stop condition is next
    //}

    }

  }
}

//#pragma vector = USI_VECTOR
//__interrupt void USI_TXRX(void)
inline void i2c_state_machine_interrupt()
{
//  switch(__even_in_range(i2c_state,12)) {
  switch(i2c_state) {
  case I2C_IDLE:
    break;

  case I2C_START_XMIT:               // start condition was sent along with address
   /* USISRL = 0x00;
    USICTL0 |= (USIGE|USIOE);
    USICTL0 &= ~USIGE;*/
    i2c_prepare_data_xmit_recv();
    break;

  case I2C_START_RECV:               // start condition was sent along with address
   /* USISRL = 0x00;
    USICTL0 |= (USIGE|USIOE);
    USICTL0 &= ~USIGE;*/
    if((UCB0STAT & UCNACKIFG) != 0) {  // did we get a NACK? if so, do a stop
      UCB0STAT &= ~UCNACKIFG;
      i2c_send_stop();
    } else { // if we got an ACK, then continue sending all data
      i2c_prepare_data_xmit_recv();
    }
    break;

  /*case I2C_PREPARE_ACKNACK:      // prepare to receive ACK/NACK
    USICTL0 &= ~USIOE;           // SDA = input
    USICNT |= 0x01;              // Bit counter=1, receive (N)Ack bit

    i2c_state = I2C_HANDLE_RXTX; // Go to next state: check ACK/NACK and continue xmitting/receiving if necessary
    break;*/

  case I2C_HANDLE_TX:         // Process Address Ack/Nack & handle data TX
    if((UCB0STAT & UCNACKIFG) != 0) {  // did we get a NACK? if so, do a stop
      UCB0STAT &= ~UCNACKIFG;
      i2c_send_stop();
    } else { // if we got an ACK, then continue sending all data
      i2c_prepare_data_xmit_recv();
    }
    break;

/*  case I2C_RECEIVED_DATA:       // received data, send ACK/NACK
    *i2c_receive_buffer = UCB0RXBUF;
    i2c_receive_buffer++;
    //USICTL0 |= USIOE;           // SDA = output
    if(i2c_sequence_length > 1) {
      // If this is not the last byte
      USISRL = 0x00;                // ACK
      i2c_state = I2C_HANDLE_RX;  // Go to next state: data/rcv again
    } else {                        // last byte: send NACK
      USISRL = 0xff;                // NACK
      i2c_state = I2C_PREPARE_STOP; // stop condition is next
    }
    USICNT |= 0x01;             // Bit counter = 1, send ACK/NACK bit
    break;
*/
/*  case I2C_PREPARE_STOP:        // prepare stop condition
    i2c_prepare_stop();         // prepare stop, go to state 14 next
    break;
*/
  /*case I2C_STOP:                // Generate Stop Condition
    USISRL = 0x0FF;             // USISRL = 1 to release SDA
    USICTL0 |= USIGE;           // Transparent latch enabled
    USICTL0 &= ~(USIGE+USIOE);  // Latch/SDA output disabled
    i2c_state = I2C_IDLE;       // Reset state machine for next xmt
    if(i2c_wakeup_sr_bits) {
      //_bic_SR_register_on_exit(i2c_wakeup_sr_bits); // exit active if prompted to
    	_BIC_SR_IRQ(i2c_wakeup_sr_bits);
	}
    break;*/
  }
  //USICTL1 &= ~USIIFG;           // Clear pending flag
}

void i2c_init(uint16_t usi_clock_divider, uint16_t usi_clock_source) {//, uint8_t slave_address) {
 // _disable_interrupts();
  dint();
  UCB0CTL1 |= UCSWRST; //Keep in reset until configured
  UCB0CTL0 = UCMODE0 + UCMODE1 + UCMST + UCSYNC; // Master mode, I2C mode, synchronous mode
  //7 bit addressing
  UCB0CTL1 = UCSWRST + usi_clock_source;
  UCB0BR0 = usi_clock_divider & 0x00FF;
  UCB0BR1 = usi_clock_divider >> 8;
//  UCB0I2CSA = slave_address; // UCB0I2CSA contains the slave address (7 or 10  bit) of the slave
  // that the msp430 is talking to over I2C bus.

  UCB0CTL1 &= ~(UCSWRST); // need to unset the reset bit before setting interrupts
//  UCB0I2CIE = UCNACKIE; // Interrupt when NACK occurs but ACK was expected
     // there is also stop condition, start condition for when msp430 is slave
     // and arbitration lost interrupt when multimaster mode is set

  IE2 =  UCB0RXIE;//UCB0TXIE + // interrupt when USCI_B0 UCB0TXBUF is empty and when UCB0RXBUF has a complete character

 // _enable_interrupts();
  eint();
}

// Use this to check whether a previously scheduled I2C sequence has been fully processed.
inline unsigned int i2c_done() {
  return(i2c_state == I2C_IDLE);
}



void readFromAddress(uint8_t address , unsigned char * Data , unsigned int Size)
{
uint8_t addressWord = address;
counterSize = Size;


	while(UCB0STAT & UCBUSY);
	I2CWriteInit();
	UCB0CTL1 |= UCTXSTT;
	while(UCB0CTL1 & UCTXSTT) {
		if(!(UCNACKIFG & UCB0STAT));
		break;

	}


	UCB0TXBUF = addressWord;// Load TX buffer
	if(UCB0STAT & UCNACKIFG)    // nack received this should not happen if address is correct
	{
	UCB0STAT &= ~UCNACKIFG;
	UCB0CTL1 |= UCTXSTP;       //stop...
	while(UCB0CTL1 & UCTXSTP);
		return;                  //... and exit
	}
	while(!(UCB0TXIFG & IFG2));


	// Read Data byte
	I2CReadInit();


	UCB0CTL1 |= UCTXSTT;                      // I2C start condition


	while(UCB0CTL1 & UCTXSTT)                // Start condition sent?
	{
	if(!(UCNACKIFG & UCB0STAT))           // Break out if ACK received
	break;
	}
	if(UCB0STAT & UCNACKIFG)    // nack received this should not happen if address is correct
	{
	UCB0STAT &= ~UCNACKIFG;
	UCB0CTL1 |= UCTXSTP;       //stop...
	while(UCB0CTL1 & UCTXSTP);
  return;
}
}

void I2CWriteInit(void) {
  UCB0CTL1 |= UCTR;
  IFG2 &= ~UCB0TXIFG;
  IE2 &= ~UCB0RXIE;
  IE2 |= UCB0TXIE;
}

void I2CReadInit(void) {
  UCB0CTL1 &= ~UCTR;
  IFG2 &= ~UCB0RXIFG;
  IE2 &= ~UCB0TXIE;
  IE2 |= UCB0RXIE;
}
