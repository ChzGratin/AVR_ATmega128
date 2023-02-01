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
	
	LCD_DATA_DDR  = 0x00; // set data port as input
	LCD_DATA_POUT = 0xFF; // internal pull-up
}

// wait until LCD is not busy
void lcd_busyWait(void)
{
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS); // control
	sbi(LCD_CTRL_PORT, LCD_CTRL_RW); // read
	LCD_DATA_DDR  = 0x00; // set data port as input
	LCD_DATA_POUT = 0xFF; // internal pull-up
	
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
}

// send an instruction
void lcd_writeControl(unsigned char data)
{
	lcd_busyWait();
	
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS); // control
	cbi(LCD_CTRL_PORT, LCD_CTRL_RW); // write
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
	LCD_DATA_DDR  = 0xFF; // set data port as output
	LCD_DATA_POUT = data; // output data
	_delay_us(1);
	cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
	
	LCD_DATA_DDR  = 0x00; // set data port as input
	LCD_DATA_POUT = 0xFF; // internal pull-up
}

// write data into DDRAM or CGRAM
void lcd_writeData(unsigned char data)
{
	lcd_busyWait();
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_RS); // data
	cbi(LCD_CTRL_PORT, LCD_CTRL_RW); // write
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
	LCD_DATA_DDR  = 0xFF; // set data port as output
	LCD_DATA_POUT = data; // output data
	_delay_us(1);
	cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
	
	LCD_DATA_DDR  = 0x00; // set data port as input
	LCD_DATA_POUT = 0xFF; // internal pull-up
}

// read busy flag (BF) and address counter
unsigned char lcd_readControl(void)
{
	unsigned char data;
	
	lcd_busyWait();
	
	LCD_DATA_DDR  = 0x00; // set data port as input
	LCD_DATA_POUT = 0xFF; // internal pull-up
	
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS); // control
	sbi(LCD_CTRL_PORT, LCD_CTRL_RW); // read
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
	_delay_us(1);
	data = LCD_DATA_PIN; // input data
	cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
	
	return data;
}

// read data from DDRAM or CGRAM
unsigned char lcd_readData(void)
{
	unsigned char data;
	
	lcd_busyWait();
	
	LCD_DATA_DDR  = 0x00; // set data port as input
	LCD_DATA_POUT = 0xFF; // internal pull-up
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_RS); // data
	sbi(LCD_CTRL_PORT, LCD_CTRL_RW); // read
	
	sbi(LCD_CTRL_PORT, LCD_CTRL_E); // enable
	_delay_us(1);
	data = LCD_DATA_PIN; // input data
	cbi(LCD_CTRL_PORT, LCD_CTRL_E); // disable
	
	return data;
}

/* high-level function */

void lcd_init(void)
{
	lcd_setupPort();
	_delay_ms(50); // wait for more than 40 ms
	
	lcd_writeControl(LCD_FUNCTION | LCD_FUNCTION_8BIT | LCD_FUNCTION_2LINE);
	lcd_writeControl(LCD_CLEAR);
	lcd_writeControl(LCD_HOME);
	lcd_writeControl(LCD_ENTRY | LCD_ENTRY_INC);
	lcd_writeControl(LCD_ON | LCD_ON_DISPLAY | LCD_ON_CURSOR | LCD_ON_BLINK);
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
		default:
		case 0: DDRAMAddr = LCD_ROW0_DDRAMADDR + x; break;
		case 1: DDRAMAddr = LCD_ROW1_DDRAMADDR + x; break;
		case 2: DDRAMAddr = LCD_ROW2_DDRAMADDR + x; break;
		case 3: DDRAMAddr = LCD_ROW3_DDRAMADDR + x; break;
	}
	
	lcd_writeControl(LCD_DDRAM | DDRAMAddr);
}

void lcd_putchar(char c)
{
	lcd_writeData(c);
}

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
	
	// TODO: load custom font
	code <<= 3;
	for(int i=0; i<8; i++)
	{
		lcd_writeControl(LCD_CGRAM | (code + i));
		lcd_writeData(pattern[i]);
	}
	
	lcd_writeControl(LCD_DDRAM | currCursorPos);
}