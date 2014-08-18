#include "hc05.h"

#include "stdlib.h"

void hc05_transmit(char *data, uint16_t transmit_length) {
	// send number of bytes specified from string
	while(transmit_length--) {
		while(!(IFG2 & UCA0TXIFG));
		UCA0TXBUF = *data;
		data++;
	}
}

void hc05_init() {
	P2DIR = BT_ON; /* Port 2 bluetooth on pin is output and on */
	P2OUT = BT_ON;
    P1SEL = RXD + TXD;
	P1SEL2 = RXD + TXD; 	// bit 1 = rxd bit 2 = txd
	UCA0CTL1 |= UCSSEL_2; 	// use smclk
	UCA0BR0 = 104; 			// baud 9600 with 1MHz clock
	UCA0BR1 = 0;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
    hc05_transmit("HC05 initialized\r\n",18);
}

void hc05_off() {

}

void hc05_on() {

}

void hc05_setspeed(uint32_t speed) {
    hc05_transmit("Setting HC05 speed\r\n",20);
   // sprintf(hc05_buffer,"%d\r\n",speed);
  //  hc05_transmit(hc05_buffer,strlen(hc05_buffer));
}
