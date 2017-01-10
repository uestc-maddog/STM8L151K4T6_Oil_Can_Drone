/*===========================================================================
网址 ：http://yhmcu.taobao.com/
作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司 
邮件 ：yihe_liyong@126.com
电话 ：18615799380
===========================================================================*/

#ifndef _BSP_H_
#define _BSP_H_

#include "STM8l15x_conf.h"
#include "CC1101.h"
#include "mytypedef.h"

// RTC时钟源选择
#define RTC_CLK_LSE 1
#define RTC_CLK_LSI 0

#define RTC_CLK RTC_CLK_LSI

// ADC 参考电压
#define Voltage_Refer 3.41f

// SPI引脚定义 SCLK(PB5), MOSI(PB6), MISO(PB7)
#define PORT_SPI        GPIOB
#define PIN_SCLK        GPIO_Pin_5        
#define PIN_MOSI        GPIO_Pin_6
#define PIN_MISO        GPIO_Pin_7

// LED 和 SWITCH引脚定义，LED(PC4), CSB(PC0), SWITCH(PD1), SMG_EN(PD2)
#define PORT_LED        GPIOC
#define PIN_LED         GPIO_Pin_4

#define PORT_CSB        GPIOC
#define PIN_CSB         GPIO_Pin_0

#define PORT_SWITCH     GPIOD
#define PIN_SWITCH      GPIO_Pin_1

#define PORT_SMGEN      GPIOD
#define PIN_SMGEN       GPIO_Pin_2    // SMG_EN

// LED SWITCH操作函数，(ON)打开, (OFF)关闭，(TOG)翻转
#define CSB_Wakeup()    GPIO_ResetBits(PORT_CSB, PIN_CSB)    // wakeup
#define CSB_Sleep()     GPIO_SetBits(PORT_CSB, PIN_CSB)      // sleep
#define CSB_TOG()       GPIO_ToggleBits(PORT_CSB, PIN_CSB)

#define LED_ON()        GPIO_ResetBits(PORT_LED, PIN_LED)        
#define LED_OFF()       GPIO_SetBits(PORT_LED, PIN_LED)
#define LED_TOG()       GPIO_ToggleBits(PORT_LED, PIN_LED)

#define SWITCH_ON()     GPIO_ResetBits(PORT_SMGEN, PIN_SMGEN);GPIO_ResetBits(PORT_SWITCH, PIN_SWITCH)        
#define SWITCH_OFF()    GPIO_SetBits(PORT_SWITCH, PIN_SWITCH);GPIO_SetBits(PORT_SMGEN, PIN_SMGEN)

void SClK_Initial(void);                // 初始化系统时钟，系统时钟 = 16MHZ
void GPIO_Initial(void);                // 初始化通用IO端口 LED KEY
void USART1_Initial(void);              // 初始化USART1
void ADC_Initial(void);                  // ADC初始化
void SPI_Initial(void);                 // 初始化SPI
void TIM3_Initial(void);                // 初始化定时器3，定时时间为1ms
INT8U SPI_ExchangeByte(INT8U input);     // 通过SPI进行数据交换 
uint16_t ADC_Data_Read(void);           // 读取ADC完成一次模数转换结果

void TIM3_Set(u8 sta);                  // 设置TIM3的开关   sta:0，关闭   1，开启     
void U1_Set(u8 sta);                    // 设置USART1的开关 sta:0，关闭   1，开启
void CSB_Initial(void);

void RTC_Initial(void);
void RTC_Set(unsigned char hour , unsigned char min , unsigned char second , unsigned int year ,unsigned char month ,unsigned char day ,unsigned char week);
#endif //_BSP_H_

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/
