/*
 * lcd.h
 *
 * Created: 2022-12-26 7:42:19 AM
 *  Author: Park Joung-hyeon
 *
 * Description:
 * Simple library for character LCD.
 * Edition of Procyon AVRlib by Pascal Stang.
 * Use 8-bit interface.
 * Implement putchar(), puts(), and printf().
 *
 * License:
 * GNU General Public License v3.0
 */

#include "lcd.h"

/* low-level function */

// setup port
void lcd_setupPort(void)
{
	// clear control port
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS);
	cbi(LCD_CTRL_PORT, LCD_CTRL_RW);
	cbi(LCD_CTRL_PORT, LCD_CTRL_E);
	
	// set control port as output
	sbi(LCD_CTRL_DDR, LCD_CTRL_RS);
	sbi(LCD_CTRL_DDR, LCD_CTRL_RW);
	sbi(LCD_CTRL_DDR, LCD_CTRL_E);
	
	#ifdef LCD_CONFIG_8BIT
		LCD_DATA_DDR  = 0x00; // set data port as input
		LCD_DATA_POUT = 0xFF; // internal pull-up
	#else // 4-bit interface
		LCD_DATA_DDR  &= 0x0F; // set data port as input
		LCD_DATA_POUT |= 0xF0; // internal pull-up
	#endif
}

// wait until LCD is not busy
void lcd_busyWait(void)
{
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS); // instruction register
	sbi(LCD_CTRL_PORT, LCD_CTRL_RW); // read
	
	#ifdef LCD_CONFIG_8BIT
		LCD_DATA_DDR  = 0x00; // set data port as input
		LCD_DATA_POUT = 0xFF; // internal pull-up
	#else // 4-bit interface
		LCD_DATA_DDR  &= 0x0F; // set data port as input
		LCD_DATA_POUT |= 0xF0; // internal pull-up
	#endif
	
	#ifdef LCD_CONFIG_8BIT
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		_delay_us(1);
		while(LCD_DATA_PIN & (1 << LCD_BUSY)) // check busy flag
		{
			cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
			_delay_us(1);
			sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
			_delay_us(1);
		}
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
	#else // 4-bit interface
		unsigned char isBusy;
		while(1)
		{
			// high 4 bits
			sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
			_delay_us(1);
			isBusy = LCD_DATA_PIN & (1 << LCD_BUSY);
			cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
			_delay_us(1);
			
			// low 4 bits
			sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
			_delay_us(1);
			cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
			_delay_us(1);
			
			if(!isBusy) { return; }
		}
	#endif
}

// write 1 byte into register without busy waiting
void lcd_writeByte(unsigned char data)
{
	cbi(LCD_CTRL_PORT, LCD_CTRL_RW); // write
	
	#ifdef LCD_CONFIG_8BIT
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		LCD_DATA_DDR  = 0xFF; // set data port as output
		LCD_DATA_POUT = data; // output data
		_delay_us(1);
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
	#else // 4-bit interface
		// high 4 bits
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		LCD_DATA_DDR |= 0xF0; // set data port as output
		LCD_DATA_POUT = data & 0xF0; // output data
		_delay_us(1);
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
		
		// low 4 bits
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		LCD_DATA_POUT = data << 4; // output data
		_delay_us(1);
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
	#endif
	
	#ifdef LCD_CONFIG_8BIT
		LCD_DATA_DDR  = 0x00; // set data port as input
		LCD_DATA_POUT = 0xFF; // internal pull-up
	#else // 4-bit interface
		LCD_DATA_DDR  &= 0x0F; // set data port as input
		LCD_DATA_POUT |= 0xF0; // internal pull-up
	#endif
}

// read 1 byte from register without busy waiting
unsigned char lcd_readByte(void)
{
	unsigned char data;
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_RW); // read
	
	#ifdef LCD_CONFIG_8BIT
		LCD_DATA_DDR  = 0x00; // set data port as input
		LCD_DATA_POUT = 0xFF; // internal pull-up
	#else // 4-bit interface
		LCD_DATA_DDR  &= 0x0F; // set data port as input
		LCD_DATA_POUT |= 0xF0; // internal pull-up
	#endif
	
	#ifdef LCD_CONFIG_8BIT
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		_delay_us(1);
		data = LCD_DATA_PIN; // input data
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
	#else // 4-bit interface
		// high 4 bits
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		_delay_us(1);
		data = LCD_DATA_PIN & 0xF0;
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
		
		// low 4 bits
		sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
		_delay_us(1);
		data |= LCD_DATA_PIN >> 4;
		cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
		_delay_us(1);
	#endif
	
	return data;
}

// send an instruction
void lcd_writeControl(unsigned char data)
{
	lcd_busyWait();
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS); // instruction register
	lcd_writeByte(data);
}

// write data into DDRAM or CGRAM
void lcd_writeData(unsigned char data)
{
	lcd_busyWait();
	sbi(LCD_CTRL_PORT, LCD_CTRL_RS); // data register
	lcd_writeByte(data);
}

// read busy flag (BF) and address counter
unsigned char lcd_readControl(void)
{
	lcd_busyWait();
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS); // instruction register
	return lcd_readByte();
}

// read data from DDRAM or CGRAM
unsigned char lcd_readData(void)
{
	lcd_busyWait();
	sbi(LCD_CTRL_PORT, LCD_CTRL_RS); // data register
	return lcd_readByte();
}

/* high-level function */

void lcd_init(void)
{
	unsigned char code;
	
	lcd_setupPort();
	_delay_ms(50); // wait for more than 40 ms
	
	// function set
	code = LCD_FUNCTION;
	
	#ifdef LCD_CONFIG_8BIT
		code |= LCD_FUNCTION_8BIT;
	#endif
	
	#ifdef LCD_CONFIG_2LINE
		code |= LCD_FUNCTION_2LINE;
	#endif
	
	#ifdef LCD_CONFIG_5X10
		code |= LCD_FUNCTION_5X10;
	#endif
	
	lcd_writeControl(code);
	// end of function set
	
	lcd_writeControl(LCD_CLEAR);
	lcd_writeControl(LCD_HOME);
	lcd_writeControl(LCD_ENTRY | LCD_ENTRY_INC);
	
	// display on/off control
	code = LCD_ON | LCD_ON_DISPLAY;
	
	#ifdef LCD_CONFIG_CURSOR_ON
		code |= LCD_ON_CURSOR;
	#endif
	
	#ifdef LCD_CONFIG_BLINK
		code |= LCD_ON_BLINK;
	#endif
	
	lcd_writeControl(code);
	// end of display on/off control
	
	lcd_writeControl(LCD_DDRAM | 0x00);
}

// move cursor. (0, 0): top left character
void lcd_gotoXY(unsigned char x, unsigned char y)
{
	unsigned char DDRAMAddr;
	
	// clamp coordinates
	if(x >= LCD_COL) { x = LCD_COL - 1; }
	if(y >= LCD_ROW) { y = LCD_ROW - 1; }
	
	switch(y)
	{
		case 0: DDRAMAddr = LCD_ROW0_DDRAMADDR + x; break;
		case 1: DDRAMAddr = LCD_ROW1_DDRAMADDR + x; break;
		case 2: DDRAMAddr = LCD_ROW2_DDRAMADDR + x; break;
		case 3: DDRAMAddr = LCD_ROW3_DDRAMADDR + x; break;
		default: return; // trivial
	}
	
	lcd_writeControl(LCD_DDRAM | DDRAMAddr);
}

void lcd_putchar(char c) { lcd_writeData(c); }

void lcd_puts(const char* str)
{
	if(!str) { return; }
	
	for(int i=0; str[i] != '\0'; i++)
	{
		lcd_writeData(str[i]);
	}
}

void lcd_printf(const char* fmt, ...)
{
	char str[LCD_ROW * LCD_COL + 1];
	va_list ap; // argument pointer
	
	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);
		
	lcd_puts(str);
}

void lcd_loadCustomFont(unsigned char code, const char* pattern)
{
	unsigned char currCursorPos = lcd_readControl() & 0x7F; // store current cursor position as DDRAM address
	
	// load custom font
	// TODO: 5x10 dots?
	code <<= 3;
	for(int i=0; i<8; i++)
	{
		lcd_writeControl(LCD_CGRAM | (code + i));
		lcd_writeData(pattern[i]);
	}
	
	lcd_writeControl(LCD_DDRAM | currCursorPos);
}
