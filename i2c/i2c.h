/*
 * i2c.h
 *
 * Created: 2023-02-10 8:27:52 PM
 *  Author: joung
 */

/* wiring */
// ATmega128, 64
// PD0 = SCL
// PD1 = SDA

// ATmega163, 323, 16, 32, etc
// PC0 = SCL
// PC1 = SDA

#ifndef _I2C_H_
#define _I2C_H_

#include <avr/io.h>

/* ================ config ================ */

/* hardware info (To change a configuration, comment or uncomment the line.) */
#define I2C_CONFIG_ATMEGA128_64 // ATmega128, 64

/* bit rate */
#define I2C_CONFIG_BITRATE_KHZ 100 // [kHz]

/* ================ end of config ================ */

/* status code */

// master
#define I2C_MT_START     0x08
#define I2C_MT_REP_START 0x10

// master transmitter
#define I2C_MT_SLAW_ACK  0x18
#define I2C_MT_SLAW_NACK 0x20
#define I2C_MT_DATA_ACK  0x28
#define I2C_MT_DATA_NACK 0x30
#define I2C_MT_ARB_LOST  0x38

// master receiver
#define I2C_MR_ARB_LOST  0x38
#define I2C_MR_SLAR_ACK  0x40
#define I2C_MR_SLAR_NACK 0x48
#define I2C_MR_DATA_ACK  0x50
#define I2C_MR_DATA_NACK 0x58

/* deprecated macro */
#ifndef sbi
#define sbi(port, bit) (port) |= (1 << (bit))
#endif

#ifndef cbi
#define cbi(port, bit) (port) &= ~(1 << (bit))
#endif

/* inline function */
void i2c_setBitrate(unsigned short bitrate_kHz);
void i2c_init(void);

void i2c_sendStart(void);
void i2c_sendStop (void);

void i2c_wait(void);

void i2c_sendByte   (unsigned char data);
void i2c_receiveByte(unsigned char ACK);

/* interrupt based function */
//...

/* non-interrupt based function */
void i2c_write(unsigned char slaveAddress, unsigned char* data,   unsigned char length);
void i2c_read (unsigned char slaveAddress, unsigned char* buffer, unsigned char length);

#endif // _I2C_H_