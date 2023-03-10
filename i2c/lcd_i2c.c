/*
 * lcd_i2c.c
 *
 * Created: 2023-02-08 11:39:58 PM
 *  Author: joung
 */

#include "lcd_i2c.h"

/* low-level function */

// write high 4 bits into register without busy waiting (dataFrame = RS_bitValue | high 4 bits)
void lcd_i2c_write4bits(LCD_I2C* lcd, unsigned char dataFrame)
{
	// prepare data frame
	cbi(dataFrame, LCD_CTRL_RW); // write
	dataFrame |= lcd->BL_bitValue & (1 << LCD_CTRL_BL); // back light
	
	// pulse enable
	cbi(dataFrame, LCD_CTRL_E); // disable
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_us(1);
	
	sbi(dataFrame, LCD_CTRL_E); // enable
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_us(1);
	
	cbi(dataFrame, LCD_CTRL_E); // disable
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_us(1);
}

// read high 4 bits from register without busy waiting (dataFrame = RS_bitValue)
unsigned char lcd_i2c_read4bits(LCD_I2C* lcd, unsigned char dataFrame)
{
	unsigned char data; // return value
	
	// prepare data frame
	sbi(dataFrame, LCD_CTRL_RW); // read
	dataFrame |= lcd->BL_bitValue & (1 << LCD_CTRL_BL); // back light
	dataFrame &= 0x0F; // clear high nibble
	
	// pulse enable
	cbi(dataFrame, LCD_CTRL_E); // disable
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_us(1);
	
	sbi(dataFrame, LCD_CTRL_E); // enable
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_us(1);
	i2c_read(lcd->slaveAddress, &data, sizeof(data)); // read byte
	
	cbi(dataFrame, LCD_CTRL_E); // disable
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_us(1);
	
	// return high nibble
	return data & 0xF0;
}

// write 1 byte into register without busy waiting
void lcd_i2c_writeByte(LCD_I2C* lcd, unsigned char RS_bitValue, unsigned char data)
{
	unsigned char dataFrame; // I2C data frame
	
	RS_bitValue &= (1 << LCD_CTRL_RS); // prevent illegal argument
	
	// high 4 bits
	dataFrame = RS_bitValue | (data & 0xF0);
	lcd_i2c_write4bits(lcd, dataFrame);
	
	// low 4 bits
	dataFrame = RS_bitValue | (data << 4);
	lcd_i2c_write4bits(lcd, dataFrame);
}

// read 1 byte from register without busy waiting
unsigned char lcd_i2c_readByte(LCD_I2C* lcd, unsigned char RS_bitValue)
{
	unsigned char data; // return value
	
	RS_bitValue &= (1 << LCD_CTRL_RS); // prevent illegal argument
	
	// high 4 bits
	data = lcd_i2c_read4bits(lcd, RS_bitValue);
	
	// low 4 bits
	data |= lcd_i2c_read4bits(lcd, RS_bitValue) >> 4;
	
	return data;
}

// initialization by instruction
void lcd_i2c_reset(LCD_I2C* lcd)
{
	unsigned char dataFrame; // I2C data frame
	
	// clear PCF8574 output
	dataFrame = lcd->BL_bitValue & (1 << LCD_CTRL_BL); // back light
	i2c_write(lcd->slaveAddress, &dataFrame, sizeof(dataFrame));
	_delay_ms(50); // wait for more than 40 ms
	
	// 1st
	lcd_i2c_write4bits(lcd, 0x30);
	_delay_ms(5); // wait for more than 4.1 ms
	
	// 2nd
	lcd_i2c_write4bits(lcd, 0x30);
	_delay_us(200); // wait for more than 100 us
	
	// 3rd
	lcd_i2c_write4bits(lcd, 0x30);
	
	// set interface to be 4 bits long
	lcd_i2c_write4bits(lcd, 0x20);
}

// wait until LCD is not busy
void lcd_i2c_busyWait(LCD_I2C* lcd)
{
	unsigned char dataFrame; // I2C data frame
	unsigned char isBusy;    // Busy Flag
	
	// prepare data frame
	dataFrame = (1 << LCD_CTRL_RW) | (lcd->BL_bitValue & (1 << LCD_CTRL_BL)); // read, back light
	i2c_write(lcd->slaveAddress, &dataFrame, 1);
	_delay_us(1);
	
	while(1)
	{
		// high 4 bits
		sbi(dataFrame, LCD_CTRL_E);  // enable
		i2c_write(lcd->slaveAddress, &dataFrame, 1);
		_delay_us(1);
		
		i2c_read(lcd->slaveAddress, &isBusy, 1); // read Busy Flag
		isBusy &= (1 << LCD_BUSY);
		
		cbi(dataFrame, LCD_CTRL_E);  // disable
		i2c_write(lcd->slaveAddress, &dataFrame, 1);
		_delay_us(1);
		
		// low 4 bits
		sbi(dataFrame, LCD_CTRL_E);  // enable
		i2c_write(lcd->slaveAddress, &dataFrame, 1);
		_delay_us(1);
		
		cbi(dataFrame, LCD_CTRL_E);  // disable
		i2c_write(lcd->slaveAddress, &dataFrame, 1);
		_delay_us(1);
		
		if(!isBusy) { break; }
	}
}

// send an instruction
void lcd_i2c_writeControl(LCD_I2C* lcd, unsigned char data)
{
	lcd_i2c_busyWait(lcd);
	lcd_i2c_writeByte(lcd, (0 << LCD_CTRL_RS), data); // instruction register
}

// write data into DDRAM or CGRAM
void lcd_i2c_writeData(LCD_I2C* lcd, unsigned char data)
{
	lcd_i2c_busyWait(lcd);
	lcd_i2c_writeByte(lcd, (1 << LCD_CTRL_RS), data); // data register
}

// read busy flag (BF) and address counter (AC)
unsigned char lcd_i2c_readControl(LCD_I2C* lcd)
{
	lcd_i2c_busyWait(lcd);
	return lcd_i2c_readByte(lcd, (0 << LCD_CTRL_RS)); // instruction register
}

// read data into DDRAM or CGRAM
unsigned char lcd_i2c_readData(LCD_I2C* lcd)
{
	lcd_i2c_busyWait(lcd);
	return lcd_i2c_readByte(lcd, (1 << LCD_CTRL_RS)); // data register
}

/* high-level function */

void lcd_i2c_init(LCD_I2C* lcd,
                  unsigned char slaveAddress, unsigned char row, unsigned char col,
                  unsigned char functionSet_code, unsigned char displayControl_code)
{
	// initialize LCD_I2C struct
	lcd->slaveAddress = slaveAddress;
	lcd->geometry[0]  = row;
	lcd->geometry[1]  = col;
	lcd->BL_bitValue  = (1 << LCD_CTRL_BL);
	
	// initialization by instruction
	lcd_i2c_reset(lcd);
	
	// function set
	functionSet_code &= (LCD_FUNCTION - 1); // prevent illegal argument
	functionSet_code &= ~LCD_FUNCTION_8BIT; // use 4-bit interface
	
	lcd_i2c_writeByte(lcd, (0 << LCD_CTRL_RS), LCD_FUNCTION | functionSet_code);
	// end of function set
	
	lcd_i2c_writeControl(lcd, LCD_CLEAR);
	lcd_i2c_writeControl(lcd, LCD_HOME);
	lcd_i2c_writeControl(lcd, LCD_ENTRY | LCD_ENTRY_INC);
	
	// display on/off control
	displayControl_code &= (LCD_ON - 1); // prevent illegal argument
	
	lcd_i2c_writeControl(lcd, LCD_ON | displayControl_code);
	// end of display on/off control
	
	lcd_i2c_writeControl(lcd, LCD_DDRAM | 0x00);
}

// move cursor. (0, 0): top left character
void lcd_i2c_gotoXY(LCD_I2C* lcd, unsigned char x, unsigned char y)
{
	unsigned char DDRAMAddr;
	
	// clamp coordinates
	if(x >= lcd->geometry[1]) { x = lcd->geometry[1] - 1; }
	if(y >= lcd->geometry[0]) { y = lcd->geometry[0] - 1; }
	
	switch(y)
	{
		case 0: DDRAMAddr = LCD_ROW0_DDRAMADDR + x; break;
		case 1: DDRAMAddr = LCD_ROW1_DDRAMADDR + x; break;
		case 2: DDRAMAddr = LCD_ROW2_DDRAMADDR + x; break;
		case 3: DDRAMAddr = LCD_ROW3_DDRAMADDR + x; break;
		default: return; // trivial
	}
	
	lcd_i2c_writeControl(lcd, LCD_DDRAM | DDRAMAddr);
}

void lcd_i2c_putchar(LCD_I2C* lcd, char c) { lcd_i2c_writeData(lcd, c); }

void lcd_i2c_puts(LCD_I2C* lcd, const char* str)
{
	if(!str) { return; }
	
	for(int i=0; str[i] != '\0'; i++)
	{
		lcd_i2c_writeData(lcd, str[i]);
	}
}

void lcd_i2c_printf(LCD_I2C* lcd, const char* fmt, ...)
{
	char str[LCD_MAX_ROW * LCD_MAX_COL + 1];
	va_list ap; // argument pointer
	
	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);
	
	lcd_i2c_puts(lcd, str);
}

void lcd_i2c_loadCustomFont(LCD_I2C* lcd, unsigned char code, const char* pattern)
{
	unsigned char currCursorPos = lcd_i2c_readControl(lcd) & 0x7F; // store current cursor position as DDRAM address
	
	// load custom font
	// TODO: 5x10 dots?
	code <<= 3;
	for(int i=0; i<8; i++)
	{
		lcd_i2c_writeControl(lcd, LCD_CGRAM | (code + i));
		lcd_i2c_writeData(lcd, pattern[i]);
	}
	
	lcd_i2c_writeControl(lcd, LCD_DDRAM | currCursorPos);
}
