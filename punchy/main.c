/* legacymsp430.h includes msp430.h and iomacros.h, it also defines
   other nice syntactic sugar, like interrupt(). */
#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>
#include "i2c.h"
#include "string.h"

/* Defines */
#define FOSC        1000000 // Oscillator speed in Hz
#define RXD 		BIT1	// Which pin on port 2 is the RX pin?
#define TXD 		BIT2	// Which pin on port 2 is the TX pin?
#define BAUD		9600
#define BT_ON 		BIT3
#define SDA			BIT7
#define SCL			BIT6

/* Data */
volatile uint8_t data_received;
char *hello_message = "\r\nHello world! \r\n";
/*uint16_t mpu6050_disable_sleep[] = {0x6B,0x00};
uint16_t mpu6050_read_address[] = {0x75,I2C_RESTART,I2C_READ};
uint16_t mpu6050_read_accel[] = {0x3B,I2C_RESTART,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ};
uint16_t mpu6050_read_gyro[] = {0x43,I2C_RESTART,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ};
uint16_t mpu6050_read_temp[] = {0x41,I2C_RESTART,I2C_READ,I2C_READ};
*/uint8_t mpu6050_buffer[16];
uint8_t mpu6050_tx_buffer[16];
uint8_t mpu6050_buffer_pointer = 0;
uint8_t mpu6050_status;
uint8_t mpu6050_write_7bit_address = 0x68;
uint8_t mpu6050_read_7bit_address = 0x69;

uint16_t ax;
uint16_t ay;
uint16_t az;
uint16_t gx;
uint16_t gy;
uint16_t gz;
uint8_t TXByteCounter = 1;
uint8_t TXByte =0x75;

uint16_t counterSize;
uint8_t tx_to_rc;
int i = 0;
/* Routines */

unsigned char *TI_receive_field;
unsigned char *TI_transmit_field;
signed char byteCtr;
unsigned char addressquery[40] = {0x75,0x00,0x00};
//------------------------------------------------------------------------------
// void TI_USCI_I2C_receiveinit(unsigned char slave_address,
//                              unsigned char prescale)
//
// This function initializes the USCI module for master-receive operation.
//
// IN:   unsigned char slave_address   =>  Slave Address
//       unsigned char prescale        =>  SCL clock adjustment
//-----------------------------------------------------------------------------
void TI_USCI_I2C_receiveinit(unsigned char slave_address,
                             unsigned char prescale){
  //P3SEL |= SDA_PIN + SCL_PIN;                 // Assign I2C pins to USCI_B0
  UCB0CTL1 = UCSWRST;                        // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;       // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;              // Use SMCLK, keep SW reset
  UCB0BR0 = prescale;                         // set prescaler
  UCB0BR1 = 0;
  UCB0I2CSA = slave_address;                  // set slave address
  UCB0CTL1 &= ~UCSWRST;                       // Clear SW reset, resume operation
  UCB0I2CIE = UCNACKIE;
  IE2 = UCB0RXIE;                            // Enable RX interrupt
}


//------------------------------------------------------------------------------
// void TI_USCI_I2C_transmitinit(unsigned char slave_address,
//                               unsigned char prescale)
//
// This function initializes the USCI module for master-transmit operation.
//
// IN:   unsigned char slave_address   =>  Slave Address
//       unsigned char prescale        =>  SCL clock adjustment
//------------------------------------------------------------------------------
void TI_USCI_I2C_transmitinit(unsigned char slave_address,
                          unsigned char prescale){
 // P3SEL |= SDA_PIN + SCL_PIN;                 // Assign I2C pins to USCI_B0
  UCB0CTL1 = UCSWRST;                        // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;       // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;              // Use SMCLK, keep SW reset
  UCB0BR0 = prescale;                         // set prescaler
  UCB0BR1 = 0;
  UCB0I2CSA = slave_address;                  // Set slave address
  UCB0CTL1 &= ~UCSWRST;                       // Clear SW reset, resume operation
  UCB0I2CIE = UCNACKIE;
  IE2 = UCB0TXIE;                            // Enable TX ready interrupt
}


//------------------------------------------------------------------------------
// void TI_USCI_I2C_receive(unsigned char byteCount, unsigned char *field)
//
// This function is used to start an I2C commuincation in master-receiver mode.
//
// IN:   unsigned char byteCount  =>  number of bytes that should be read
//       unsigned char *field     =>  array variable used to store received data
//------------------------------------------------------------------------------
void TI_USCI_I2C_receive(unsigned char byteCount, unsigned char *field){
  TI_receive_field = field;
  if ( byteCount == 1 ){
    byteCtr = 0 ;
    __disable_interrupt();
    UCB0CTL1 |= UCTXSTT;                      // I2C start condition
    while (UCB0CTL1 & UCTXSTT);               // Start condition sent?
    UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
    __enable_interrupt();
  } else if ( byteCount > 1 ) {
    byteCtr = byteCount - 2 ;
    UCB0CTL1 |= UCTXSTT;                      // I2C start condition
  } else
    while (1);                                // illegal parameter
}

//------------------------------------------------------------------------------
// void TI_USCI_I2C_transmit(unsigned char byteCount, unsigned char *field)
//
// This function is used to start an I2C commuincation in master-transmit mode.
//
// IN:   unsigned char byteCount  =>  number of bytes that should be transmitted
//       unsigned char *field     =>  array variable. Its content will be sent.
//------------------------------------------------------------------------------
void TI_USCI_I2C_transmit(unsigned char byteCount, unsigned char *field){
  TI_transmit_field = field;
  byteCtr = byteCount;
  UCB0CTL1 |= UCTR + UCTXSTT;                 // I2C TX, start condition
}

//------------------------------------------------------------------------------
// unsigned char TI_USCI_I2C_slave_present(unsigned char slave_address)
//
// This function is used to look for a slave address on the I2C bus.
//
// IN:   unsigned char slave_address  =>  Slave Address
// OUT:  unsigned char                =>  0: address was not found,
//                                        1: address found
//------------------------------------------------------------------------------
unsigned char TI_USCI_I2C_slave_present(unsigned char slave_address){
  unsigned char ie2_bak, slaveadr_bak, ucb0i2cie, returnValue;
  ucb0i2cie = UCB0I2CIE;                      // restore old UCB0I2CIE
  ie2_bak = IE2;                              // store IE2 register
  slaveadr_bak = UCB0I2CSA;                   // store old slave address
  UCB0I2CIE &= ~ UCNACKIE;                    // no NACK interrupt
  UCB0I2CSA = slave_address;                  // set slave address
  IE2 &= ~(UCB0TXIE + UCB0RXIE);              // no RX or TX interrupts
  __disable_interrupt();
  UCB0CTL1 |= UCTR + UCTXSTT + UCTXSTP;       // I2C TX, start condition
  while (UCB0CTL1 & UCTXSTP);                 // wait for STOP condition

  returnValue = !(UCB0STAT & UCNACKIFG);
  __enable_interrupt();
  IE2 = ie2_bak;                              // restore IE2
  UCB0I2CSA = slaveadr_bak;                   // restore old slave address
  UCB0I2CIE = ucb0i2cie;                      // restore old UCB0CTL1
  return returnValue;                         // return whether or not
                                              // a NACK occured
}

//------------------------------------------------------------------------------
// unsigned char TI_USCI_I2C_notready()
//
// This function is used to check if there is commuincation in progress.
//
// OUT:  unsigned char  =>  0: I2C bus is idle,
//                          1: communication is in progress
//------------------------------------------------------------------------------
unsigned char TI_USCI_I2C_notready(){
  return (UCB0STAT & UCBBUSY);
}


void transmit_uart(unsigned char *string, uint16_t string_length) {
	// send number of bytes specified from string
	while(string_length--) {
		while(!(IFG2 & UCA0TXIFG));
		UCA0TXBUF = *string;
		string++;
	}
}


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
/*BCSCTL1 &= ~XTS;
	BCSCTL1 |= DIVA0 + DIVA1;
	BCSCTL3 &= ~LFXT1S0;
	BCSCTL3 |= LFXT1S1;*/
}

/*
void sendAccel() {
	i2c_send_sequence(mpu6050_read_accel,10,mpu6050_buffer,LPM1_bits);
	_BIS_SR(LPM1_bits + GIE);
	while(!i2c_done());
	for(int i = 0; i < 6; i++) {
		TXData = mpu6050_buffer[i];
		TX_Byte();
	}
}

void sendGyro() {
	i2c_send_sequence(mpu6050_read_gyro,10,mpu6050_buffer,LPM1_bits);
	_BIS_SR(LPM1_bits + GIE);
	while(!i2c_done());
	for(int i = 0; i < 6; i++) {
		TXData = mpu6050_buffer[i];
		TX_Byte();
	}
}


void sendTemp() {
	i2c_send_sequence(mpu6050_read_temp,7,mpu6050_buffer,LPM1_bits);
	_BIS_SR(LPM1_bits + GIE);
	while(!i2c_done());
	for(int i = 0; i < 2; i++) {
		TXData = mpu6050_buffer[i];
		TX_Byte();
	}
}

void sendAck() {
	TXData = 'A';
	TX_Byte();
}
*/
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
	P2DIR = BT_ON; /* Port 2 bluetooth on pin is output and on */
	P2OUT = BT_ON;

	/* Configure uart */
	P1SEL = RXD + TXD;
	P1SEL2 = RXD + TXD; 	// bit 1 = rxd bit 2 = txd
	UCA0CTL1 |= UCSSEL_2; 	// use smclk
	UCA0BR0 = 104; 			// baud 9600 with 1MHz clock
	UCA0BR1 = 0;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt

	/* Configure I2C */
	//P1OUT |= SDA + SCL;
    P1DIR |= SDA + SCL;
    for(i = 0; i < 500; i++) {
        P1OUT ^= SCL;
    }
	P1SEL |= SDA + SCL;
	P1SEL2 |= SDA + SCL;
	transmit_uart(hello_message,strlen(hello_message));
	_BIS_SR(GIE);
	i2c_init(); //divide by 4, USCI clock source select 2 (SMCLK)
	/* Talk to mpu6050: 7 bit I2C address is 0xd0 for write, 0xd1 for read */
	/* [0xd0 0x75 [ 0xd1 r ] will read the address register (0x75) of the mpu6050, you should get 0x68 back */
	//i2c_send_sequence(mpu6050_read_address,3,mpu6050_buffer,LPM1_bits,mpu6050_write_7bit_address);
//	_BIS_SR(LPM1_bits + GIE);
	//while(!i2c_done());
	//TXData = mpu6050_buffer[0];
	transmit_uart("I2C init\r\n",10);
	//readFromAddress(0x75,mpu6050_buffer,1);
	//TI_USCI_I2C_transmitinit(0x68,4);

//while(TI_USCI_I2C_notready());
 /*if ( TI_USCI_I2C_slave_present(0x68) )    // slave address may differ from
  {                                         // initialization
    TI_USCI_I2C_receiveinit(0x68,4);   // init receiving with USCI
    while ( TI_USCI_I2C_notready() );         // wait for bus to be free
    TI_USCI_I2C_receive(4,mpu6050_buffer);
    while ( TI_USCI_I2C_notready() );         // wait for bus to be free

    TI_USCI_I2C_transmitinit(0x68,4);  		// init transmitting with
    while ( TI_USCI_I2C_notready() );         // wait for bus to be free
    TI_USCI_I2C_transmit(1,addressquery);       // start transmitting
  }*/
	//transmit_uart("Address: ",9);
	//transmit_uart(mpu6050_buffer,1);


//	i2c_send_sequence(mpu6050_disable_sleep,3,mpu6050_buffer,LPM1_bits);
//	_BIS_SR(LPM1_bits + GIE);
//	while(!i2c_done());
	//TXData = mpu6050_buffer[0];

/*	i2c_send_sequence(mpu6050_read_accel,10,mpu6050_buffer,LPM1_bits);
	_BIS_SR(LPM1_bits + GIE);
	while(!i2c_done());
	for(int i = 0; i < 6; i++) {
		TXData = mpu6050_buffer[i];
		TX_Byte();
	}

	while(1) {
		RX_Ready();
		_BIS_SR(LPM1_bits + GIE);
		switch(RXData) {
			case 'T':
				sendTemp();
				break;
			case 'A':
				sendAccel();
				break;
			case 'G':
				sendGyro();
				break;
			default:
				sendAck();
				break;
		}

//sendAccel();
	}
	*/
	mpu6050_tx_buffer[0] = 0x75;
	TXByteCounter = 1;
	//i2c_transmit();
	i2c_transmit_to_receive();
	i2c_receive();

	mpu6050_tx_buffer[1] = 0x6B;
	mpu6050_tx_buffer[0] = 0x00;
	TXByteCounter =2;
	i2c_transmit();

//	TXByteCounter = 1;
//	i2c_transmit();

	mpu6050_tx_buffer[0] = 0x3B;
	TXByteCounter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);


	mpu6050_tx_buffer[0] = 0x3B;
	TXByteCounter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);

	mpu6050_tx_buffer[0] = 0x75;
	TXByteCounter = 1;
	i2c_transmit_to_receive();
	i2c_receive();



	while(1) {

transmit_uart("D",1);
		//transmit_uart(hello_message,strlen(hello_message));



		//while(UCB0CTL1 & UCTXSTT);
		//while(!IFG2 & UCB0TXIFG);
		//UCB0TXBUF = 0x75; // address query
		//UCB0CTL1 &= ~UCTR;
		//UCB0CTL1 |= UCTXSTT;
		//IFG2 &= ~UCB0TXIFG;
		//while(UCB0CTL1 & UCTXSTT);

		//UCB0CTL1 |= UCTXSTP;
		//__bis_SR_register(LPM0_bits + GIE);
	//	__bis_SR_register(LPM0_bits + GIE);
	}

}

interrupt (USCIAB0RX_VECTOR) receive_uart_I2C_state_isr(void) {
	// Interrupt for both i2c state and UART receive
	// UART receive
	if(IFG2 & UCA0RXIFG) {
		data_received = UCA0RXBUF;
		transmit_uart("Received: ", 10);
		transmit_uart(&data_received, 1);
		transmit_uart("\r\n",2);
		IFG2 &= ~UCA0RXIFG;
	} else if(UCB0STAT & UCNACKIFG) {
		//Need to send stop
		UCB0CTL1 |= UCTXSTP;
		UCB0STAT &= ~UCNACKIFG;
	// I2C state machine
	//	i2c_state_machine_interrupt();
	}
}

interrupt (USCIAB0TX_VECTOR) transmit_uart_I2C_data_isr(void) {
	// Interrupt for both i2c data and UART transmit
	// UART transmit

	if(IFG2 & UCA0TXIFG) {
		;//IFG2 &= ~UCA0TXIFG;
	}
    if(UCB0RXIFG &IFG2) { // Received full character on I2C
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
	*/	mpu6050_buffer[mpu6050_buffer_pointer++] = UCB0RXBUF;
		if(mpu6050_buffer_pointer == 16) mpu6050_buffer_pointer = 0;
		__bic_SR_register_on_exit(LPM0_bits);
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
		if(TXByteCounter>0) {
			UCB0TXBUF = mpu6050_tx_buffer[TXByteCounter-1];
			TXByteCounter--;
			__bic_SR_register_on_exit(LPM0_bits);
		} else if (TXByteCounter == 1){
			UCB0TXBUF = mpu6050_tx_buffer[0];
			UCB0CTL1 |= UCTXSTP;
			IFG2 &= ~UCB0TXIFG;
			__bic_SR_register_on_exit(LPM0_bits);
		} else {
			if(tx_to_rc) {
				IFG2 &= ~UCB0TXIFG;
				__bic_SR_register_on_exit(LPM0_bits);
			} else {
				UCB0CTL1 |= UCTXSTP;
				IFG2 &= ~UCB0TXIFG;
				__bic_SR_register_on_exit(LPM0_bits);
			}
		}
	}
}

void i2c_transmit(void) { // txifg will go right away, so we need to disable interrupts until STT is done!
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

void i2c_transmit_to_receive() {
//__dint();
    tx_to_rc = 1;
	while(UCB0CTL1 & UCTXSTP);
	UCB0CTL1 |= UCTR + UCTXSTT;

//__eint();
	__bis_SR_register(LPM0_bits + GIE);
}

void i2c_receive(void) {
	while(UCB0CTL1 & UCTXSTP);
	UCB0CTL1 &= ~UCTR;
	UCB0CTL1 |= UCTXSTT;
	while(UCB0CTL1 & UCTXSTT);
	UCB0CTL1 |= UCTXSTP;
	__bis_SR_register(LPM0_bits + GIE);
}

void i2c_multireceive(uint8_t receive_amount) {
	// Last one needs to have STP bit set in middle of reception
	while(UCB0CTL1 & UCTXSTP);
	UCB0CTL1 &= ~UCTR;
	UCB0CTL1 |= UCTXSTT;
	for( i = 0; i<receive_amount - 1; i++) {
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

