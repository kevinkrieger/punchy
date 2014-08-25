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
    UCB0BR0 = 24; // The clock source is divided down by UCB0R register pair + 1
    UCB0BR1 = 0; // Example - if using 1MHz calibrated internal clock, and this is
    // set to '24' then the clock to the UCB0 I2C module will be 40kHz (=1MHz/25)


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
    while(UCB0STAT & UCBBUSY); /* NEED TO DO THIS!!!! */
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
    if(receive_amount == 1) {
        i2c_receive();
    } else {
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
}


void i2c_write_reg(uint8_t reg, uint8_t value) {
    i2c_tx_buffer[1] = reg;
    i2c_tx_buffer[0] = value;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
}

/* Write bytes to a single register address */
void i2c_write_bytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
    //I2C_writeBytesToAddress(devAddr, regAddr, length, data);
  /*  int i;
    for(i = 0; i<length; i++) {
        i2c_write_reg(regAddr,*data);
        data++;
    }*/
   // if(length == 1) {
   //     i2c_write_reg(regAddr,*data);
   // } else {
        i2c_tx_buffer[length] = regAddr;
        for(int i = 0;i<length;i++) {
            i2c_tx_buffer[length-i-1] = data[i];
        }
        i2c_tx_buffer_counter = length+1;
        i2c_transmit();
    //}
}

void i2c_read_reg(uint8_t reg) {
    i2c_tx_buffer[0] = reg;
	i2c_tx_buffer_counter = 1;
	//i2c_transmit_to_receive();
	i2c_transmit();
	i2c_receive();
}

/** Read multiple bytes from an 8-bit device register.
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 */
void i2c_read_bytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
  /*  for (uint8_t k = 0; k < length; k++) {
        i2c_read_reg(regAddr);
        data[k] = i2c_rx_buffer[0];
    }*/
    // Need to do a multireceive...
    if(length < 1) {
        return;
    }
    i2c_tx_buffer[0] = regAddr;
    i2c_tx_buffer_counter = 1;
    i2c_transmit();
    i2c_multireceive(length);
     for (uint8_t k = 0; k < length; k++) {
        //i2c_read_reg(regAddr);
        data[k] = i2c_rx_buffer[k];
    }
}

/** Read multiple bits from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 */
void i2c_read_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b;
    i2c_read_reg(regAddr);
    b = i2c_rx_buffer[0];
    if(b != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
    }
    i2c_rx_buffer[0] = b;
//    if ((count = i2c_read_reg(regAddr)) != 0) {
}

/** Write multiple words to a 16-bit device register.
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */ //0x13,1, (0)
bool i2c_write_words(uint8_t regAddr, uint8_t length, uint16_t* data) {
    i2c_tx_buffer_counter = 1;
    i2c_tx_buffer[0] = regAddr;
    i2c_transmit();

    for (uint8_t i = 0; i < length*2 ; i++) {
        i2c_tx_buffer_counter = 2;
        i2c_tx_buffer[1] = (uint8_t)(data[i++]>>8);
        i2c_tx_buffer[0] = (uint8_t)(data[i++]);
        i2c_transmit();
    }
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool i2c_write_word(uint8_t regAddr, uint16_t data) {
    return i2c_write_words(regAddr, 1, &data);
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool i2c_write_bit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_read_reg(regAddr);

    b = (data != 0) ? (i2c_rx_buffer[0] | (1 << bitNum)) : (i2c_rx_buffer[0] & ~(1 << bitNum));
    i2c_write_reg(regAddr, b);
    return true;
}


/** Write multiple bits in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool i2c_write_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    i2c_read_reg(regAddr);
    b = i2c_rx_buffer[0];
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    i2c_write_reg(regAddr, b);
 }
