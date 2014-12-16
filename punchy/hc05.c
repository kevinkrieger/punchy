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
	while((UCA0STAT & UCBUSY)); // Wait until last byte finished transmitting
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
    uint8_t temp;
    uint16_t uca0br = __baud_to_uca0br(38400);
    UCA0BR0 = uca0br & 0xFF;
	UCA0BR1 = (uca0br >> 8) & 0xFF;
    // Set the BT_KEY pin high
    __bic_SR_register(GIE);
    delay_ms(30);
    hc05_off();
    P1SEL &= ~BT_KEY;
    P1SEL2 &= ~BT_KEY;
    P1DIR |= BT_KEY;
    P1OUT |= BT_KEY;

    delay_ms(255);
    hc05_on();
    delay_ms(255);
    delay_ms(255);

    hc05_transmit("AT+UART=115200,1,0\r\n",20);
    delay_ms(255);

    while(IFG2 & UCA0RXIFG) {
        temp = UCA0RXBUF;
        delay_ms(1);
    }
    hc05_transmit("AT+UART?\r\n",10);
    delay_ms(100);

    while(IFG2 & UCA0RXIFG) {
        temp = UCA0RXBUF;
        delay_ms(100);

    }
    P1OUT &= ~BT_KEY;
    delay_ms(100);
    UCA0CTL1 |= UCSSEL_2; 	// use smclk
    uca0br = __baud_to_uca0br(115200);
    UCA0BR0 = uca0br & 0xFF;
	UCA0BR1 = (uca0br >> 8) & 0xFF;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
	P2OUT &= ~BT_RESET; // reset the thing
	delay_ms(30);
	P2OUT |= BT_RESET;
}
