
#ifndef HC05_H
#define HC05_H
#include <legacymsp430.h>
#include <stdint.h>
#include <msp430g2553.h>

#define BT_KEY          BIT0    // Port 1
#define BT_RED          BIT3    // Port 1
#define BT_BLUE         BIT4    // Port 1
#define BT_STATUS       BIT5    // Port 1
#define BT_DISCONNECT   BIT0    // Port 2
#define BT_RESET        BIT4    // Port 2
#define BT_ON 		    BIT3    // Port 2
#define BAUD		    9600
#define BT_RX 		    BIT1	// Which pin on port 2 is the RX pin?
#define BT_TX 		    BIT2	// Which pin on port 2 is the TX pin?
#define CYCLES_PER_US (FOSC/1000000)
#define __delay_us(delay) __delay_cycles((CYCLES_PER_US*delay))
#define __baud_to_uca0br(baud) (FOSC/(baud))

uint8_t TXData;
uint8_t hc05_buffer[32];

void hc05_transmit(char *data, uint16_t transmit_length);

void hc05_init(uint16_t uca0br);

void hc05_off();

void hc05_on();

void hc05_setspeed(uint32_t speed);

static inline void sendAck() {
//	TXData = "A";
	hc05_transmit("A",1);
}


#endif
