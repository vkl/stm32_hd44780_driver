/*
  @file    stm32f10x_lcd.c
  @author  vkl 
  @version V0.0.1
  @date    18 November 2015
  @brief   HD44780 driver
*/

#include "stm32f10x_lcd.h"
#include "stm32f10x.h"

void lcd_send_byte(char data, LCDSend_TypeDef data_type)
{
  /* RW -> 0 */
  LCD_COM_PORT->ODR &= ~(1 << LCD_RW);

  set_data_type(data_type);
  LCD_DATA_PORT->ODR &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));
  LCD_DATA_PORT->ODR |= ((data >> 7 & 0x1) << LCD_D7) | \
                   ((data >> 6 & 0x1) << LCD_D6) | \
                   ((data >> 5 & 0x1) << LCD_D5) | \
                   ((data >> 4 & 0x1) << LCD_D4);
  LCD_COM_PORT->ODR |= (1 << LCD_E);
  delay_us(2);
  LCD_COM_PORT->ODR &= ~(1 << LCD_E);
  delay_us(10);

  LCD_DATA_PORT->ODR &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));
  LCD_DATA_PORT->ODR |= ((data >> 3 & 0x1) << LCD_D7) | \
                   ((data >> 2 & 0x1) << LCD_D6) | \
                   ((data >> 1 & 0x1) << LCD_D5) | \
                   ((data >> 0 & 0x1) << LCD_D4);
  LCD_COM_PORT->ODR |= (1 << LCD_E);
  delay_us(2);
  LCD_COM_PORT->ODR &= ~(1 << LCD_E);
  delay_us(10);

  if ((data == 0x01) || (data == 0x02)) delay_ms(2); else delay_us(40);
}

void lcd_read_byte(char* data, LCDSend_TypeDef data_type)
{
  *data = 0x0;

  /* Configure GPIO */
  DATAPORT_INPUT;
  DATAPORT_PULL;

  /* RW -> 1 */
  LCD_COM_PORT->ODR |= (1 << LCD_RW);
  
  set_data_type(data_type);
 
  LCD_COM_PORT->ODR |= (1 << LCD_E);
  delay_us(2);
  /* Read high bites */
  *data |= ((((LCD_DATA_PORT->IDR & (1 << LCD_D4)) >> LCD_D4) << 4) | \
            (((LCD_DATA_PORT->IDR & (1 << LCD_D5)) >> LCD_D5) << 5) | \
            (((LCD_DATA_PORT->IDR & (1 << LCD_D6)) >> LCD_D6) << 6) | \
            (((LCD_DATA_PORT->IDR & (1 << LCD_D7)) >> LCD_D7) << 7));
 
  LCD_COM_PORT->ODR &= ~(1 << LCD_E);
  delay_us(10);

  LCD_COM_PORT->ODR |= (1 << LCD_E);
  delay_us(2);
 /* Read low bites */
  *data |= ((((LCD_DATA_PORT->IDR & (1 << LCD_D4)) >> LCD_D4) << 0) | \
            (((LCD_DATA_PORT->IDR & (1 << LCD_D5)) >> LCD_D5) << 1) | \
            (((LCD_DATA_PORT->IDR & (1 << LCD_D6)) >> LCD_D6) << 2) | \
            (((LCD_DATA_PORT->IDR & (1 << LCD_D7)) >> LCD_D7) << 3));
 
  LCD_COM_PORT->ODR &= ~(1 << LCD_E);
  delay_us(10);

  /* RW -> 0 */
  LCD_COM_PORT->ODR &= ~(1 << LCD_RW);

  if (LCD_COM_PORT->ODR & (1 << LCD_RS)) delay_us(43); else delay_us(10);

  /* Return back data GPIO */
  DATAPORT_OUTPUT;
  DATAPORT_PUSH_PULL;
  
}

void lcd_init(LCD_InitTypeDef *LCD_InitStructure)
{
  delay_ms(45);

  /* Check params If param is not defined set it by default */
  if (!IS_LCD_MODE(LCD_InitStructure->LCD_Mode)) LCD_InitStructure->LCD_Mode = LCD_Mode_4bit;
  if (!IS_LCD_DISPLAY(LCD_InitStructure->LCD_Display)) LCD_InitStructure->LCD_Display = LCD_Display_on;
  if (!IS_LCD_CURSOR(LCD_InitStructure->LCD_Cursor)) LCD_InitStructure->LCD_Cursor = LCD_Cursor_on_noblinking;
  if (!IS_LCD_LINES(LCD_InitStructure->LCD_Lines)) LCD_InitStructure->LCD_Lines = LCD_Lines_2;
  if (!IS_LCD_FONT(LCD_InitStructure->LCD_Font)) LCD_InitStructure->LCD_Font = LCD_Font_5x8;
  if (!IS_LCD_ENTRY_MODE(LCD_InitStructure->LCD_EntryMode)) 
    LCD_InitStructure->LCD_EntryMode = LCD_Entry_AddrIncrement_NoShiftDisplay;

  /* Configure GPIO */
  COMPORT_OUTPUT;
  COMPORT_PUSH_PULL;
  DATAPORT_OUTPUT;
  DATAPORT_PUSH_PULL;

  /* Set mode 4bit/8bit, line numbers 1/2, font 5x8/5x11 */ 
  lcd_send_byte((1 << 5) | (LCD_InitStructure->LCD_Mode << 4) | (LCD_InitStructure->LCD_Lines << 3) | (LCD_InitStructure->LCD_Font << 2), Command); 
  lcd_send_byte((1 << 5) | (LCD_InitStructure->LCD_Mode << 4) | (LCD_InitStructure->LCD_Lines << 3) | (LCD_InitStructure->LCD_Font << 2), Command);

  /* Set display and cursor */
  lcd_send_byte((1 << 3) | (LCD_InitStructure->LCD_Display << 2) | LCD_InitStructure->LCD_Cursor, Command);
  
  /* Clear display */
  LCD_CLEAR();

  delay_ms(2);

  /* Set entry mode */
  lcd_send_byte((1 << 2) | LCD_InitStructure->LCD_EntryMode, Command);
}

void lcd_out(char *text)
{
  while(*text)
  {
    lcd_send_byte(*text, Data);
    text++;
  }
}

void lcd_in(char *text, unsigned char num)
{
  do 
  {
    num--;
    lcd_read_byte(&text[num], Data);
  } while (num);
}

void lcd_write_custom_char(unsigned char addr, unsigned char *s)
{
  unsigned char row;

  /* Set Init CGRAM address */
  lcd_send_byte((1 << 6) | (addr << 3), Command);
  for (row=0; row<=7; row++)
  {
    /* Send row */
    lcd_send_byte(s[row], Data);
  }
}

