/*===========================================================================
��ַ ��http://yhmcu.taobao.com/
���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾ 
�ʼ� ��yihe_liyong@126.com
�绰 ��18615799380
===========================================================================*/

#ifndef _BSP_H_
#define _BSP_H_

#include "STM8l15x_conf.h"
#include "CC1101.h"
#include "mytypedef.h"

// RTCʱ��Դѡ��
#define RTC_CLK_LSE 1
#define RTC_CLK_LSI 0

#define RTC_CLK RTC_CLK_LSI

// ADC �ο���ѹ
#define Voltage_Refer     3.3655f
#define Voltage_Bat_Full  8.4000f      // 3408
#define Voltage_Bat_Empty 7.4000f      // 3002

// SPI���Ŷ��� SCLK(PB5), MOSI(PB6), MISO(PB7)
#define PORT_SPI        GPIOB
#define PIN_SCLK        GPIO_Pin_5        
#define PIN_MOSI        GPIO_Pin_6
#define PIN_MISO        GPIO_Pin_7

// LED �� SWITCH���Ŷ��壬LED(PC4), CSB(PC0), SWITCH(PD1), SMG_EN(PD2)
#define PORT_LED        GPIOC
#define PIN_LED         GPIO_Pin_4

#define PORT_CSB        GPIOC
#define PIN_CSB         GPIO_Pin_0

#define PORT_SWITCH     GPIOD
#define PIN_SWITCH      GPIO_Pin_1

#define PORT_SMGEN      GPIOD
#define PIN_SMGEN       GPIO_Pin_2    // SMG_EN

// LED SWITCH����������(ON)��, (OFF)�رգ�(TOG)��ת
#define CSB_Wakeup()    GPIO_ResetBits(GPIOC, GPIO_Pin_0)    // wakeup
#define CSB_Sleep()     GPIO_SetBits(GPIOC, GPIO_Pin_0)      // sleep
#define CSB_TOG()       GPIO_ToggleBits(GPIOC, GPIO_Pin_0)

#define LED_ON()        GPIO_ResetBits(PORT_LED, PIN_LED)        
#define LED_OFF()       GPIO_SetBits(PORT_LED, PIN_LED)
#define LED_TOG()       GPIO_ToggleBits(PORT_LED, PIN_LED)

#define SWITCH_ON()     GPIO_ResetBits(PORT_SMGEN, PIN_SMGEN);GPIO_ResetBits(PORT_SWITCH, PIN_SWITCH)        
#define SWITCH_OFF()    GPIO_SetBits(PORT_SWITCH, PIN_SWITCH);GPIO_SetBits(PORT_SMGEN, PIN_SMGEN)

void SClK_Initial(void);                // ��ʼ��ϵͳʱ�ӣ�ϵͳʱ�� = 16MHZ
void GPIO_Initial(void);                // ��ʼ��ͨ��IO�˿� LED KEY
void USART1_Initial(void);              // ��ʼ��USART1
void ADC_Initial(void);                  // ADC��ʼ��
void SPI_Initial(void);                 // ��ʼ��SPI
void TIM3_Initial(void);                // ��ʼ����ʱ��3����ʱʱ��Ϊ1ms
INT8U SPI_ExchangeByte(INT8U input);     // ͨ��SPI�������ݽ��� 
uint16_t ADC_Data_Read(void);           // ��ȡADC���һ��ģ��ת�����

void TIM3_Set(u8 sta);                  // ����TIM3�Ŀ���   sta:0���ر�   1������     
void U1_Set(u8 sta);                    // ����USART1�Ŀ��� sta:0���ر�   1������
void CSB_Initial(void);

void RTC_Initial(void);
void RTC_Set(unsigned char hour , unsigned char min , unsigned char second , unsigned int year ,unsigned char month ,unsigned char day ,unsigned char week);
#endif //_BSP_H_

/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/
