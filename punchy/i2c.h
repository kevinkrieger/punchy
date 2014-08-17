#ifndef I2C_H
#define I2C_H
#include <stdint.h>
/* I2C routines */
void i2c_init();

void i2c_start();

void i2c_stop();

void i2c_write(uint8_t byte);
void i2c_tx_init();
void i2c_rx_init();

void i2c_stop();


#endif
