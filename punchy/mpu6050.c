#include "mpu6050.h"
#include "i2c.h"
#include "hc05.h"

uint8_t j;

void mpu6050_init() {

	/* Configure I2C */
	i2c_init();

    i2c_tx_buffer[0] = 0x75;
	i2c_tx_buffer_counter = 1;
	//i2c_transmit();
	i2c_transmit_to_receive();
	i2c_receive();

	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x00;
	i2c_tx_buffer_counter = 2;
	i2c_transmit();

// This doesn't work but the receive 6 works.
	i2c_tx_buffer[0] = 0x3B;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);


	i2c_tx_buffer[0] = 0x3B;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);

	i2c_tx_buffer[0] = 0x75;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_receive();

}

void mpu6050_getAddress() {
    i2c_tx_buffer[0] = 0x75;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_receive();
	//TXData = i2c_rx_buffer[0];
	hc05_transmit((char*)i2c_rx_buffer,1);
}

void mpu6050_wakeup() {
	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x00;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	sendAck();
}
void mpu6050_sleep() {

	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x40;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	sendAck();
}

void mpu6050_reset() {

	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x80;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	sendAck();
}
void mpu6050_accel() {

    i2c_tx_buffer[0] = 0x3B;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);
//	for(j = 0; j< 6; j++) {
	//	TXData = i2c_rx_buffer[j];
    hc05_transmit((char*)i2c_rx_buffer,6);
	//}
}

void mpu6050_gyro() {
	i2c_tx_buffer[0] = 0x43;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);
	//for(j = 0; j < 6; j++) {
	//	TXData = i2c_rx_buffer[j];
    hc05_transmit((char*)i2c_rx_buffer,6);
	//}
}


void mpu6050_temp() {
    i2c_tx_buffer[0] = 0x41;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(2);
	//for(j = 0; j < 2; j++) {
	//	TXData = i2c_rx_buffer[j];
		hc05_transmit((char*)i2c_rx_buffer,2);
	//}
}
