/*
 * i2c.c
 *
 * Created: 2023-02-10 8:28:03 PM
 *  Author: joung
 */

#include <util/delay.h>

#include "i2c.h"

/* inline function */

void i2c_setBitrate(unsigned short bitrate_kHz)
{
	// f_SCL = f_CPU / (16 + 2 * TWBR * 4^TWPS)
	// TWPS  = 0
	#ifdef TWPS0 // ATmega128
		cbi(TWSR, TWPS0);
		cbi(TWSR, TWPS1);
	#endif
	
	// f_SCL = f_CPU / (16 + 2 * TWBR)
	// TWBR  = f_CPU / (2 * f_SCL) - 8
	TWBR = F_CPU / (2 * 1000 * bitrate_kHz) - 8;
}

inline void i2c_init(void)
{
	cbi(TWCR, TWEN); // disable TWI
	
	// internal pull-up
	#ifdef I2C_CONFIG_ATMEGA128_64
		sbi(PORTD, 0); // PD0 = SCL
		sbi(PORTD, 1); // PD1 = SDA
	#else
		sbi(PORTD, 0); // PC0 = SCL
		sbi(PORTD, 1); // PC1 = SDA
	#endif
	
	// set bit rate
	i2c_setBitrate(I2C_CONFIG_BITRATE_KHZ);
	
	sbi(TWCR, TWIE); // enable TWI interrupt
	sbi(TWCR, TWEA); // enable TWI ACK bit
	sbi(TWCR, TWEN); // enable TWI
}

inline void i2c_sendStart(void) { TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); }
inline void i2c_sendStop (void) { TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); }

inline void i2c_wait(void)
{
	while( !(TWCR & (1<<TWINT)) );
}

inline void i2c_sendByte(unsigned char data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
}

inline void i2c_receiveByte(unsigned char ACK)
{
	if(ACK)
	{
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	}
	else // last byte
	{
		TWCR = (1<<TWINT) | (1<<TWEN);
	}
}

/* non-interrupt based function */

void i2c_write(unsigned char slaveAddress, unsigned char* data, unsigned char length)
{
	cbi(TWCR, TWIE); // disable TWI interrupt
	
	// START
	i2c_sendStart();
	i2c_wait();
	
	// DEVICE ADDRESS
	i2c_sendByte(slaveAddress & 0xFE);
	i2c_wait();
	if((TWSR & 0xF8) == I2C_MT_SLAW_ACK) // ACK
	{
		// WRITE DATA
		while(length)
		{
			i2c_sendByte(*data);
			i2c_wait();
			
			data++;
			length--;
		}
	}
	else // NOT ACK
	{
		// ERROR
	}
	
	// STOP
	i2c_sendStop();
	
	sbi(TWCR, TWIE); // enable TWI interrupt
}

void i2c_read(unsigned char slaveAddress, unsigned char* buffer, unsigned char length)
{
	cbi(TWCR, TWIE); // disable TWI interrupt
	
	// START
	i2c_sendStart();
	i2c_wait();
	
	// DEVICE ADDRESS
	i2c_sendByte(slaveAddress | 0x1);
	i2c_wait();
	if((TWSR & 0xF8) == 0x40) // ACK
	{
		// READ DATA
		while(length)
		{
			i2c_receiveByte(length > 1);
			i2c_wait();
			*buffer = TWDR;
			
			buffer++;
			length--;
		}
	}
	else // NOT ACK
	{
		// ERROR
	}
	
	// STOP
	i2c_sendStop();
	
	sbi(TWCR, TWIE); // enable TWI interrupt
}