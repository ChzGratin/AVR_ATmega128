/*
 * lcd_i2c.h
 *
 * Created: 2023-02-04 3:00:41 PM
 *  Author: joung
 */

#ifndef _LCD_I2C_H_
#define _LCD_I2C_H_

#ifndef F_CPU
#error "F_CPU not defined for lcd.h"
#endif

#include <util/delay.h>
#include <stdarg.h>

#include "i2c.h"

/* wiring */
// ATmega128, 64
// PD0 = SCL
// PD1 = SDA

// ATmega163, 323, 16, 32, etc
// PC0 = SCL
// PC1 = SDA

/* ================ config ================ */

/* MCU info (To change a configuration, comment or uncomment the line.) */
// #define LCD_I2C_CONFIG_ATMEGA128_64
#define LCD_I2C_CONFIG_SLAVE_ADDRESS 0x7E

/* hardware info (To change a configuration, comment or uncomment the line.) */
#define LCD_CONFIG_2LINE
// #define LCD_CONFIG_5X10

/* initial cursor option (To change a configuration, comment or uncomment the line.) */
// #define LCD_CONFIG_CURSOR_ON
// #define LCD_CONFIG_BLINK

/* ================ end of config ================ */

/* maximal LCD geometry */
#define LCD_MAX_ROW 4
#define LCD_MAX_COL 20

/* typical DDRAM address */
#define LCD_ROW0_DDRAMADDR 0x00
#define LCD_ROW1_DDRAMADDR 0x40
#define LCD_ROW2_DDRAMADDR 0x14
#define LCD_ROW3_DDRAMADDR 0x54

/* I2C LCD with PCF8574 */
#define LCD_CTRL_RS 0
#define LCD_CTRL_RW 1
#define LCD_CTRL_E  2
#define LCD_CTRL_BL 3 // Back Light

/* instruction */
#define LCD_CLEAR    (1 << 0) // clear display
#define LCD_HOME     (1 << 1) // return home
#define LCD_ENTRY    (1 << 2) // entry mode set
#define LCD_ON       (1 << 3) // display on/off control
#define LCD_MOVE     (1 << 4) // cursor or display shift
#define LCD_FUNCTION (1 << 5) // function set
#define LCD_CGRAM    (1 << 6) // set CGRAM address
#define LCD_DDRAM    (1 << 7) // set DDRAM address

/* code */
#define LCD_ENTRY_INC      (1 << 1) // 1: increment (0: decrement)
#define LCD_ENTRY_SHIFT    (1 << 0) // 1: accompanies display shift
#define LCD_ON_DISPLAY     (1 << 2) // 1: entire display on (0: off)
#define LCD_ON_CURSOR      (1 << 1) // 1: cursor on (0: off)
#define LCD_ON_BLINK       (1 << 0) // 1: blink cursor position character (0: don't)
#define LCD_MOVE_DISPLAY   (1 << 3) // 1: display shift (0: cursor move)
#define LCD_MOVE_RIGHT     (1 << 2) // 1: shift to the right (0: left)
#define LCD_FUNCTION_8BIT  (1 << 4) // 1: 8 bits (0: 4 bits)
#define LCD_FUNCTION_2LINE (1 << 3) // 1: 2 lines (0: 1 line)
#define LCD_FUNCTION_5X10  (1 << 2) // 1: 5x10 dots (0: 5x8 dots)

/* busy flag */
#define LCD_BUSY 7
// #define LCD_BUSY (1 << 7)

/* deprecated macro */
#ifndef sbi
#define sbi(port, bit) (port) |= (1 << (bit))
#endif

#ifndef cbi
#define cbi(port, bit) (port) &= ~(1 << (bit))
#endif

/* LCD_I2C struct */
typedef struct
{
	unsigned char slaveAddress;
	unsigned char geometry[2]; // [0] = row, [1] = col
	unsigned char BL_bitValue; // (1 << LCD_CTRL_BL) or 0
	unsigned char ddramAddress[4]; // [n] = DDRAM address of n-th row
} LCD_I2C;

/* low-level function */
void          lcd_i2c_write4bits(LCD_I2C* lcd, unsigned char dataFrame); // write high 4 bits into register without busy waiting (dataFrame = RS_bitValue | high 4 bits)
unsigned char lcd_i2c_read4bits (LCD_I2C* lcd, unsigned char dataFrame); // read high 4 bits from register without busy waiting (dataFrame = RS_bitValue)

void          lcd_i2c_writeByte(LCD_I2C* lcd, unsigned char RS_bitValue, unsigned char data); // write 1 byte into register without busy waiting
unsigned char lcd_i2c_readByte (LCD_I2C* lcd, unsigned char RS_bitValue);                     // read 1 byte from register without busy waiting

void lcd_i2c_reset(LCD_I2C* lcd); // initialization by instruction
void lcd_i2c_busyWait(LCD_I2C* lcd); // wait until LCD is not busy

void lcd_i2c_writeControl(LCD_I2C* lcd, unsigned char data); // send an instruction
void lcd_i2c_writeData   (LCD_I2C* lcd, unsigned char data); // write data into DDRAM or CGRAM

unsigned char lcd_i2c_readControl(LCD_I2C* lcd); // read busy flag (BF) and address counter (AC)
unsigned char lcd_i2c_readData   (LCD_I2C* lcd); // read data into DDRAM or CGRAM

/* high-level function */
void lcd_i2c_init(LCD_I2C* lcd,
                  unsigned char slaveAddress, unsigned char row, unsigned char col,
                  unsigned char functionSet_code, unsigned char displayControl_code);
				  
void lcd_i2c_gotoXY(LCD_I2C* lcd, unsigned char x, unsigned char y); // move cursor. (0, 0): top left character
void lcd_i2c_putchar(LCD_I2C* lcd, char c);
void lcd_i2c_puts(LCD_I2C* lcd, const char* str);
void lcd_i2c_printf(LCD_I2C* lcd, const char* fmt, ...);

void lcd_i2c_loadCustomFont(LCD_I2C* lcd, unsigned char code, const char* pattern);

#endif // _LCD_I2C_H_