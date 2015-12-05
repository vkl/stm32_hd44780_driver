/*
  @file    stm32f10x_lcd.h
  @author  vkl 
  @version V0.0.1
  @date    18 November 2015
  @brief   HD44780 driver
*/

#ifndef __STM32F10x_LCD_H
#define __STM32F10x_LCD_H

#include "delay.h"

/* User defined settings */

#define LCD_DATA_PORT GPIOC
#define LCD_COM_PORT GPIOC

#define LCD_RS 0
#define LCD_E 1
#define LCD_RW 6

#define LCD_D4 2
#define LCD_D5 3
#define LCD_D6 4
#define LCD_D7 5
	
#define LCD_ROWS 4
#define DDRAM_ADDR0 0x00
#define DDRAM_ADDR1 0x40
#define DDRAM_ADDR2 0x10
#define DDRAM_ADDR3 0x50

#define COMPORT_PUSH_PULL  (LCD_COM_PORT->CRL &=~ (GPIO_CRL_CNF0_0 | GPIO_CRL_CNF1_0 | GPIO_CRL_CNF6_0 | \
                                                  GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1 | GPIO_CRL_CNF6_1))
#define COMPORT_OUTPUT     (LCD_COM_PORT->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE6_1)) 

#define DATAPORT_PUSH_PULL (LCD_COM_PORT->CRL &=~ (GPIO_CRL_CNF2_0 | GPIO_CRL_CNF3_0 | GPIO_CRL_CNF4_0 | GPIO_CRL_CNF5_0 | \
                                                  GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1)) 
#define DATAPORT_OUTPUT    (LCD_COM_PORT->CRL |= (GPIO_CRL_MODE2_1 | GPIO_CRL_MODE3_1 | \
                                                 GPIO_CRL_MODE4_1 | GPIO_CRL_MODE5_1))

#define DATAPORT_PULL      (LCD_COM_PORT->CRL |= (GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1))
#define DATAPORT_INPUT     (LCD_COM_PORT->CRL &=~ (GPIO_CRL_MODE2_1 | GPIO_CRL_MODE3_1 | \
                                                  GPIO_CRL_MODE4_1 | GPIO_CRL_MODE5_1))
 
/* End of user defined settings */

typedef enum {
  Command = 0,
  Data = 1
} LCDSend_TypeDef;
#define set_data_type(expr) ((expr) ? (LCD_COM_PORT->ODR |= (1 << LCD_RS)) : (LCD_COM_PORT->ODR &= ~(1 << LCD_RS)))
 
typedef enum {
  LCD_Mode_4bit = 0x0,
  LCD_Mode_8bit = 0x1
} LCDMode_TypeDef;
#define IS_LCD_MODE(MODE) (((MODE) == LCD_Mode_4bit) || ((MODE) == LCD_Mode_8bit))

typedef enum {
  LCD_Display_off = 0x0,
  LCD_Display_on = 0x1
} LCDDisplay_TypeDef;
#define IS_LCD_DISPLAY(DISPLAY) (((DISPLAY) == LCD_Display_off) || ((DISPLAY) == LCD_Display_on))

typedef enum {
  LCD_Cursor_off = 0x0,
  LCD_Cursor_off_blinking = 0x1,
  LCD_Cursor_on_noblinking = 0x2,
  LCD_Cursor_on_blinking = 0x3
} LCDCursor_TypeDef;
#define IS_LCD_CURSOR(CURSOR) (((CURSOR) == LCD_Cursor_off) || ((CURSOR) == LCD_Cursor_off_blinking) || \
                               ((CURSOR) == LCD_Cursor_on_noblinking) || ((CURSOR) == LCD_Cursor_on_blinking))

typedef enum {
  LCD_Lines_1 = 0x0,
  LCD_Lines_2 = 0x1
} LCDLines_TypeDef;
#define IS_LCD_LINES(LINES) (((LINES) == LCD_Lines_1) || ((LINES) == LCD_Lines_2))

typedef enum {
  LCD_Font_5x8 = 0x0,
  LCD_Font_5x11 = 0x1
} LCDFont_TypeDef;
#define IS_LCD_FONT(FONT) (((FONT) == LCD_Font_5x8) || ((FONT) == LCD_Font_5x11))

typedef enum {
  LCD_Entry_AddrIncrement_NoShiftDisplay = 0x2,
  LCD_Entry_AddrDecrement_NoShiftDisplay = 0x0,
  LCD_Entry_AddrIncrement_ShiftDisplay = 0x3,
  LCD_Entry_AddrDecrement_ShiftDisplay = 0x4
} LCDEntryMode_TypeDef;
#define IS_LCD_ENTRY_MODE(MODE) (((MODE) == LCD_Entry_AddrIncrement_NoShiftDisplay) || \
                                 ((MODE) == LCD_Entry_AddrDecrement_NoShiftDisplay) || \
                                 ((MODE) == LCD_Entry_AddrIncrement_ShiftDisplay) || \
                                 ((MODE) == LCD_Entry_AddrDecrement_ShiftDisplay))

typedef struct {
  LCDMode_TypeDef LCD_Mode;
  LCDLines_TypeDef LCD_Lines;
  LCDFont_TypeDef LCD_Font;
  LCDDisplay_TypeDef LCD_Display;
  LCDCursor_TypeDef LCD_Cursor;
  LCDEntryMode_TypeDef LCD_EntryMode;
} LCD_InitTypeDef;

/* LCD command */
#define LCD_CLEAR() (lcd_send_byte(0x01, Command))
#define LCD_RETURN_HOME() (lcd_send_byte(0x02, Command))
#define LCD_SET_ADDR(ADDR) (lcd_send_byte((1 << 7) | (ADDR), Command));

void lcd_send_byte(char data, LCDSend_TypeDef);
void lcd_read_byte(char*, LCDSend_TypeDef);
void lcd_init(LCD_InitTypeDef*);
void lcd_out(char*);
void lcd_in(char*, unsigned char);
void lcd_write_custom_char(unsigned char, unsigned char*);

#endif

