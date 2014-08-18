
#ifndef HC05_H
#define HC05_H
#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>


#define BT_ON 		BIT3
#define BAUD		9600
#define RXD 		BIT1	// Which pin on port 2 is the RX pin?
#define TXD 		BIT2	// Which pin on port 2 is the TX pin?

uint8_t TXData;
uint8_t hc05_buffer[32];

void hc05_transmit(char *data, uint16_t transmit_length);

void hc05_init();

void hc05_off();

void hc05_on();

void hc05_setspeed(uint32_t speed);

static inline void sendAck() {
//	TXData = "A";
	hc05_transmit("A",1);
}


#endif
