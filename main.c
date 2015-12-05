/**
  @file    main.c
  @author  vkl 
  @version V0.0.1
  @date    18 November 2015
  @brief   STM32VLDiscovery demo. Simple clock with one-button interface
**/

#include "stm32f10x.h"
#include "delay.h"
#include "stm32f10x_lcd.h"
#include "stm32f10x_rtc.h"

EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;

unsigned char hours;
unsigned char minutes;
unsigned char seconds;
unsigned char init_addr;

volatile uint8_t BTN_FLAGS = 0x00;  

#define TEN(NUMBER) ((3 << 4) | NUMBER / 10)
#define ONE(NUMBER) ((3 << 4) | NUMBER % 10)

void display_time(addr)
{
  LCD_SET_ADDR(addr);
  lcd_send_byte(TEN(hours), Data);
  lcd_send_byte(ONE(hours), Data);
  lcd_send_byte(':', Data);
  lcd_send_byte(TEN(minutes), Data);
  lcd_send_byte(ONE(minutes), Data);
}

void set_time(addr)
{
  seconds++;
  if (seconds > 59) 
  { 
    seconds = 0; minutes++; 
    if (minutes > 59) 
    { 
      minutes = 0; hours++; 
      if (hours > 23) 
      { 
        hours = 0; 
      }
      LCD_SET_ADDR(addr);
      lcd_send_byte(TEN(hours), Data);
      lcd_send_byte(ONE(hours), Data);
    }
    LCD_SET_ADDR(addr + 3);
    lcd_send_byte(TEN(minutes), Data);
    lcd_send_byte(ONE(minutes), Data);
  }
}

void EXT0_Config(void) 
{
   /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  //NVIC_InitStructure.NVIC_IRQC4hannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
}

void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    /* Clear the  EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
    
    delay_ms(200);

    if ((GPIOA->IDR & GPIO_IDR_IDR0) == 0)
    {
      if (BTN_FLAGS & 0x1) 
      {
        hours++;
        if (hours > 23) hours = 0;
        LCD_SET_ADDR(init_addr);
        lcd_send_byte(TEN(hours), Data);
        lcd_send_byte(ONE(hours), Data);
        LCD_SET_ADDR(init_addr + 1);
      } 
      else if (BTN_FLAGS & 0x2)
      {
        minutes++;
        if (minutes > 59) minutes = 0;
        LCD_SET_ADDR(init_addr + 3);
        lcd_send_byte(TEN(minutes), Data);
        lcd_send_byte(ONE(minutes), Data);
        LCD_SET_ADDR(init_addr + 4);
      }
    }

    while(GPIOA->IDR & GPIO_IDR_IDR0) 
    {
      delay_ms(100);
      BTN_FLAGS = (((BTN_FLAGS >> 4) + 1) << 4) | (0xF & BTN_FLAGS);
      if ((BTN_FLAGS & (15 << 4)) == 0xF0)
      {
        BTN_FLAGS &= ~(15 << 4);
        if (BTN_FLAGS == 0x0) 
          BTN_FLAGS = 0x1;
        else
          BTN_FLAGS = BTN_FLAGS << 1;
        if (BTN_FLAGS & (1 << 2)) { BTN_FLAGS = 0x0; LCD_SET_ADDR(init_addr + 5); } 
        else if (BTN_FLAGS & (1 << 0)) { LCD_SET_ADDR(init_addr + 1); }
        else if (BTN_FLAGS & (1 << 1)) { LCD_SET_ADDR(init_addr + 4); }
      }
    }
    BTN_FLAGS &= ~(15 << 4);

    delay_ms(100);
    
  }
}

void RTC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;

   /* Configure one bit for preemption priority */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

   /* Enable the RTC Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

   PWR_BackupAccessCmd(ENABLE);
   RCC_LSEConfig(RCC_LSE_ON);
   while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
   RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
   RCC_RTCCLKCmd(ENABLE);

   RTC_WaitForSynchro();
   RTC_WaitForLastTask();
   RTC_ITConfig(RTC_IT_SEC, ENABLE);
   RTC_WaitForLastTask();
   RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
   RTC_WaitForLastTask();
}

void RTC_IRQHandler(void)
{
  if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {
    /* Clear the RTC Second interrupt */
    RTC_ClearITPendingBit(RTC_IT_SEC);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    set_time(init_addr);
  }
}

void main(void)
{
  hours = 0;
  minutes = 0;
  seconds = 0;
  init_addr = DDRAM_ADDR0;

  //Enabling clock for GPIOC, GPIOB
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  // Enable clock for timer
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  // Enable RTC
  RTC_Configuration();

  LCD_InitTypeDef LCD_InitStructure;

  LCD_InitStructure.LCD_Mode = LCD_Mode_4bit;
  LCD_InitStructure.LCD_Display = LCD_Display_on; 
  LCD_InitStructure.LCD_Cursor = LCD_Cursor_on_noblinking;
  LCD_InitStructure.LCD_EntryMode = LCD_Entry_AddrIncrement_NoShiftDisplay;

  lcd_init(&LCD_InitStructure);

  EXT0_Config();

  display_time(init_addr);
 
  while (1)
  {
  }
}

