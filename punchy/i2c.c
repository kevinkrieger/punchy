#include "i2c.h"

/* I2C routines */

void i2c_init()
{
    dint();
    P1DIR |= SDA + SCL;
    /* Toggle SCL line a few hundred times to 'reset' the line */
    int i;
    for(i = 0; i < 500; i++)
    {
        P1OUT ^= SCL;
    }
    P1SEL |= SDA + SCL;
    P1SEL2 |= SDA + SCL;
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

void i2c_transmit(void)   // txifg will go right away, so we need to disable interrupts until STT is done!
{
//__dint();
    tx_to_rc = 0;
    while(UCB0CTL1 & UCTXSTP);
    UCB0CTL1 |= UCTR + UCTXSTT;
    while(UCB0CTL1 & UCTXSTT);


//__eint();
    __bis_SR_register(LPM0_bits + GIE);
    // __bis_SR_register(GIE);
    //UCB0CTL1 |= UCTR + UCTXSTT;
}

void i2c_transmit_to_receive()
{
//__dint();
    tx_to_rc = 1;
    while(UCB0CTL1 & UCTXSTP);
    UCB0CTL1 |= UCTR + UCTXSTT;

//__eint();
    __bis_SR_register(LPM0_bits + GIE);
}

void i2c_receive(void)
{
    //mpu6050_buffer_pointer=0;
    i2c_rx_buffer_pointer = 0;
    while(UCB0CTL1 & UCTXSTP);
    UCB0CTL1 &= ~UCTR;
    UCB0CTL1 |= UCTXSTT;
    while(UCB0CTL1 & UCTXSTT);
    UCB0CTL1 |= UCTXSTP;
    __bis_SR_register(LPM0_bits + GIE);
}

void i2c_multireceive(uint8_t receive_amount)
{
    // Last one needs to have STP bit set in middle of reception
    //mpu6050_buffer_pointer=0;
    int i;
    i2c_rx_buffer_pointer = 0;
    while(UCB0CTL1 & UCTXSTP);
    UCB0CTL1 &= ~UCTR;
    UCB0CTL1 |= UCTXSTT;
    for( i = 0; i<receive_amount - 1; i++)
    {
        __bis_SR_register(LPM0_bits + GIE);
        //		IFG2 &= ~UCB0TXIFG; 		// clear txifg
        // RX interrupt
        // will fire when we have received a byte
    }
//	while(UCB0CTL1 &UCTXSTP);
//	UCB0CTL1 &= ~UCTR;
//	UCB0CTL1 |= UCTXSTT;
    //while(UCB0CTL1 & UCTXSTT);
    UCB0CTL1 |= UCTXSTP;
    __bis_SR_register(LPM0_bits + GIE);
}

