#include "hc05.h"
#include "stdlib.h"
//#include "string.h"


void hc05_transmit(char *data, uint16_t transmit_length) {
	// send number of bytes specified from string
	while(transmit_length--) {
		while(!(IFG2 & UCA0TXIFG));
		UCA0TXBUF = *data;
		data++;
	}
}

void hc05_init(uint16_t uca0br) {
	P2DIR |= BT_ON + BT_RESET;  // Port 2 bluetooth on pin is output and on
	P2OUT &= ~BT_ON;            //BT_ON is active low
	P2OUT |= BT_RESET;          //reset is active low
	P1SEL &= ~BT_KEY;
    P1SEL2 &= ~BT_KEY;
	P1DIR |= BT_KEY;
	P1OUT &= ~BT_KEY;

    P1SEL |= BT_RX + BT_TX;
	P1SEL2 |= BT_RX + BT_TX; 	// bit 1 = rxd bit 2 = txd
	UCA0CTL1 |= UCSSEL_2; 	    // use smclk

	UCA0BR0 = uca0br & 0xFF;
	UCA0BR1 = (uca0br >> 8) & 0xFF;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	    //Need to disable transmit interrupt

   hc05_setspeed(115200);
}

void hc05_off() {
    P2OUT |= BT_ON;
}

void hc05_on() {
    P2DIR |= BT_ON; // Port 2 bluetooth on pin is output and on
	P2OUT &= ~BT_ON;
}

void hc05_key() {
    P1OUT ^= BT_KEY;
}

void hc05_setspeed(uint32_t speed) {
    //sprintf(tempbuf,"Setting HC05 speed: %ld",speed);
    //hc05_transmit(tempbuf,strlen(tempbuf));
    uint8_t temp;
    uint16_t uca0br = __baud_to_uca0br(38400);
    UCA0BR0 = uca0br & 0xFF;
	UCA0BR1 = (uca0br >> 8) & 0xFF;
    // Set the BT_KEY pin high
    __bic_SR_register(GIE);
    delay_ms(30);
    /*__delay_us(32000);
    */hc05_off();
    P1SEL &= ~BT_KEY;
    P1SEL2 &= ~BT_KEY;
    P1DIR |= BT_KEY;
    P1OUT |= BT_KEY;
   /* __delay_us(32000);
    __delay_us(32000);
    __delay_us(32000); // TODO: Make this use a timer for a large delay
    __delay_us(32000);
    */
    delay_ms(255);
    hc05_on();
    delay_ms(255);
    delay_ms(255);
    /*__delay_us(32000);__delay_us(32000);__delay_us(32000);__delay_us(32000);__delay_us(32000);
    __delay_us(32000);__delay_us(32000);__delay_us(32000);__delay_us(32000);__delay_us(32000);__delay_us(32000);__delay_us(32000);
    __delay_us(32000);__delay_us(32000);
    __delay_us(32000);__delay_us(32000);
    __delay_us(32000);__delay_us(32000);*/

  //  UCA0CTL1 |= UCSSEL_2; 	// use smclk
  //  UCA0BR0 = 208; // 208 is for 38400 baud at 8MHz
  //  UCA0BR1 = 0;
   // UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	//UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	//IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	//IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
    hc05_transmit("AT+UART=115200,1,0\r\n",20);
    delay_ms(255);
    /*__delay_us(32000);
    __delay_us(32000);    __delay_us(32000);
    __delay_us(32000);    __delay_us(32000);
    __delay_us(32000);
    */while(IFG2 & UCA0RXIFG) {
        //tempbuf[0] = UCA0RXBUF;
        temp = UCA0RXBUF;
        delay_ms(1);
        /*__delay_us(32000);
        __delay_us(32000);*/
    }
    hc05_transmit("AT+UART?\r\n",10);
    delay_ms(100);
    /*__delay_us(32000);
    __delay_us(32000);
    */while(IFG2 & UCA0RXIFG) {
        //tempbuf[0] = UCA0RXBUF;
        temp = UCA0RXBUF;
        delay_ms(100);
        /*__delay_us(32000);
        __delay_us(32000);
    */}
    P1OUT &= ~BT_KEY;
    delay_ms(100);
    /*__delay_us(32000);
    __delay_us(32000);
    */
    //hc05_init(69);
    UCA0CTL1 |= UCSSEL_2; 	// use smclk
	//UCA0BR0 = 104; 			// baud 9600 with 1MHz clock
	//UCA0BR1 = 0;
    uca0br = __baud_to_uca0br(115200);
    UCA0BR0 = uca0br & 0xFF;
	UCA0BR1 = (uca0br >> 8) & 0xFF;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
	P2OUT &= ~BT_RESET; // reset the thing
	delay_ms(30);
	/*__delay_us(32000);
	*/
	P2OUT |= BT_RESET;
    //IFG2 &= ~UCA0RXIFG;
    //__bis_SR_register(GIE);

//    hc05_off();


    //hc05_transmit("Setting HC05 speed\r\n",20);
   // sprintf(hc05_buffer,"%d\r\n",speed);
  //  hc05_transmit(hc05_buffer,strlen(hc05_buffer));
}
