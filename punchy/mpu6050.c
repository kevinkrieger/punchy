#include "mpu6050.h"
#include "i2c.h"
#include "hc05.h"
#include "stdio.h"
#include "string.h"
uint8_t j;
char tempbuf[64];

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

    /* Set up accel range */
    mpu6050_configAccel(MPU6050_ACCEL_FS_16<<(MPU6050_ACONFIG_AFS_SEL_BIT-1));
    /* Set up sample rate for accel */

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

/* These are div 16384 if +/-2g, 8192 if +/-4g, 4096 if +/-8g and 2048 if +/-16g*/
ax = (i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1]);
ay = (i2c_rx_buffer[2]<<8 | i2c_rx_buffer[3]);
az = (i2c_rx_buffer[4]<<8 | i2c_rx_buffer[5]);
sprintf(tempbuf,"%d %d %d\r\n",ax,ay,az);
hc05_transmit(tempbuf,strlen(tempbuf));
//    hc05_transmit((char*)ax,1);
	//hc05_transmit((char*)ay,1);
	//hc05_transmit((char*)az,1);
	//}
}

void mpu6050_gyro() {
	i2c_tx_buffer[0] = 0x43;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(6);
	//for(j = 0; j < 6; j++) {
	//	TXData = i2c_rx_buffer[j];
gx = (i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1]);
gy = (i2c_rx_buffer[2]<<8 | i2c_rx_buffer[3]);
gz = (i2c_rx_buffer[4]<<8 | i2c_rx_buffer[5]);
  sprintf(tempbuf,"%d %d %d\r\n",gx,gy,gz);
hc05_transmit(tempbuf,strlen(tempbuf));
	//}
}


void mpu6050_temp() {
    i2c_tx_buffer[0] = 0x41;
	i2c_tx_buffer_counter = 1;
	i2c_transmit_to_receive();
	i2c_multireceive(2);
	//for(j = 0; j < 2; j++) {
	//	TXData = i2c_rx_buffer[j];
temperature = i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1];
temperature = (temperature/340) + 37;
sprintf(tempbuf,"%d\r\n",temperature);
hc05_transmit(tempbuf,strlen(tempbuf));
}

void mpu6050_configAccel(uint8_t accel_config) {
    i2c_tx_buffer[1] = MPU6050_RA_ACCEL_CONFIG;
	i2c_tx_buffer[0] = accel_config;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	//sendAck();
}
