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

#ifndef _LCD_H_
#define _LCD_H_

#ifndef F_CPU
#error "F_CPU not defined for lcd.h"
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdarg.h>

/* ================ config ================ */

/* hardware info (To change a configuration, uncomment the line.) */
 #define LCD_CONFIG_8BIT
// #define LCD_CONFIG_1LINE
// #define LCD_CONFIG_5X10

/* initial cursor option (To change a configuration, uncomment the line.) */
// #define LCD_CONFIG_HIDE_CURSOR
// #define LCD_CONFIG_NO_BLINK

/* port */
// control
#define LCD_CTRL_PORT PORTA
#define LCD_CTRL_DDR  DDRA
#define LCD_CTRL_RS   0
#define LCD_CTRL_RW   1
#define LCD_CTRL_E    2

// data
#define LCD_DATA_POUT PORTB
#define LCD_DATA_PIN  PINB
#define LCD_DATA_DDR  DDRB

/* LCD geometry */
#define LCD_ROW 4
#define LCD_COL 20

/* DDRAM address */
#define LCD_ROW0_DDRAMADDR 0x00
#define LCD_ROW1_DDRAMADDR 0x40
#define LCD_ROW2_DDRAMADDR 0x14
#define LCD_ROW3_DDRAMADDR 0x54

/* ================ end of config ================ */

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

/* low-level function */
void lcd_setupPort(void); // setup port

void lcd_busyWait(void); // wait until LCD is not busy

void lcd_writeControl(unsigned char data); // send an instruction
void lcd_writeData   (unsigned char data); // write data into DDRAM or CGRAM

unsigned char lcd_readControl(void); // read busy flag (BF) and address counter
unsigned char lcd_readData   (void); // read data into DDRAM or CGRAM

/* high-level function */
void lcd_init(void);
void lcd_gotoXY(unsigned char x, unsigned char y); // move cursor. (0, 0): top left character
void lcd_putchar(char c);
void lcd_puts(const char* str);
void lcd_printf(const char* fmt, ...);

void lcd_loadCustomFont(unsigned char code, const char* pattern);

#endif // _LCD_H_
