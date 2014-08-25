#include "hc05.h"
#include "stdlib.h"
//#include "string.h"

//extern char tempbuf[32];

void hc05_transmit(char *data, uint16_t transmit_length) {
	// send number of bytes specified from string
	while(transmit_length--) {
		while(!(IFG2 & UCA0TXIFG));
		UCA0TXBUF = *data;
		data++;
	}
}

void hc05_init(uint16_t uca0br) {
	P2DIR |= BT_ON + BT_RESET; /* Port 2 bluetooth on pin is output and on */
	P2OUT |= BT_ON;
	P2OUT |= BT_RESET;//reset is active low
	P1SEL &= ~BT_KEY;
    P1SEL2 &= ~BT_KEY;
	P1DIR |= BT_KEY;
	P1OUT &= ~BT_KEY;

    P1SEL |= BT_RX + BT_TX;
	P1SEL2 |= BT_RX + BT_TX; 	// bit 1 = rxd bit 2 = txd
	UCA0CTL1 |= UCSSEL_2; 	// use smclk
	//UCA0BR0 = 104; 			// baud 9600 with 1MHz clock
	//UCA0BR1 = 0;
	UCA0BR0 = uca0br & 0xFF; // 65 and 3  : baud 9600 with 8MHz clock
	UCA0BR1 = (uca0br >> 8) & 0xFF;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
    hc05_transmit("HC05 initialized\r\n",18);

   // hc05_setspeed(115200);
}

void hc05_off() {
    P2OUT &= ~BT_ON; /* Doesn't work in board rev-1 due to DMG1012t-7 diode from source to drain. */
}

void hc05_on() {
    P2DIR = BT_ON; /* Port 2 bluetooth on pin is output and on */
	P2OUT = BT_ON;
}

void hc05_key() {
    P1OUT ^= BT_KEY;
}

void hc05_setspeed(uint32_t speed) {
    //sprintf(tempbuf,"Setting HC05 speed: %ld",speed);
    //hc05_transmit(tempbuf,strlen(tempbuf));
    uint8_t temp;
    // Set the BT_KEY pin high
    __bic_SR_register(GIE);
    P1SEL &= ~BT_KEY;
    P1SEL2 &= ~BT_KEY;
    P1DIR |= BT_KEY;
    P1OUT |= BT_KEY;
    __delay_us(8000);
    __delay_us(8000);
  //  UCA0CTL1 |= UCSSEL_2; 	// use smclk
  //  UCA0BR0 = 208; // 208 is for 38400 baud at 8MHz
  //  UCA0BR1 = 0;
   // UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	//UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	//IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	//IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
    hc05_transmit("AT+UART=115200,0,0\r\n",20);
    __delay_us(8000);
    while(IFG2 & UCA0RXIFG) {
        //tempbuf[0] = UCA0RXBUF;
        temp = UCA0RXBUF;
        __delay_us(8000);
        __delay_us(8000);
    }
    P1OUT &= ~BT_KEY;
    __delay_us(800);
    //hc05_init(69);
    UCA0CTL1 |= UCSSEL_2; 	// use smclk
	//UCA0BR0 = 104; 			// baud 9600 with 1MHz clock
	//UCA0BR1 = 0;
	UCA0BR0 = 69; // 69  : baud 115200 with 8MHz clock
	UCA0BR1 = 0;
	UCA0MCTL = UCBRS0; 		//modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 	// init usci state machine
	IE2 |= UCA0RXIE; 		//enable USCI_A0 rx interrupt
	IE2 &= ~UCA0TXIE;	//Need to disable transmit interrupt
    IFG2 &= ~UCA0RXIFG;
    __bis_SR_register(GIE);

//    hc05_off();


    //hc05_transmit("Setting HC05 speed\r\n",20);
   // sprintf(hc05_buffer,"%d\r\n",speed);
  //  hc05_transmit(hc05_buffer,strlen(hc05_buffer));
}
