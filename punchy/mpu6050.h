
#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>
#include "i2c.h"
/*uint16_t mpu6050_disable_sleep[] = {0x6B,0x00};
uint16_t mpu6050_read_address[] = {0x75,I2C_RESTART,I2C_READ};
uint16_t mpu6050_read_accel[] = {0x3B,I2C_RESTART,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ};
uint16_t mpu6050_read_gyro[] = {0x43,I2C_RESTART,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ,I2C_READ};
uint16_t mpu6050_read_temp[] = {0x41,I2C_RESTART,I2C_READ,I2C_READ};
*/
//uint8_t mpu6050_buffer[16];
//uint8_t mpu6050_tx_buffer[16];
//uint8_t mpu6050_buffer_pointer = 0;
uint8_t mpu6050_status;
//uint8_t mpu6050_write_7bit_address = 0x68;
//uint8_t mpu6050_read_7bit_address = 0x69;

uint16_t ax;
uint16_t ay;
uint16_t az;
uint16_t gx;
uint16_t gy;
uint16_t gz;


/* Routines */

void mpu6050_init();

void mpu6050_getAddress();

void mpu6050_wakeup();

void mpu6050_reset();

void mpu6050_sleep();

void mpu6050_accel();

void mpu6050_gyro();

void mpu6050_temp();


#endif
