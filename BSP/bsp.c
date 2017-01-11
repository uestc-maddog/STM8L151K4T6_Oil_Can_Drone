/*===========================================================================
* 网址 ：http://www.cdebyte.com/   http://yhmcu.taobao.com/                 *
* 作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司                 * 
* 邮件 ：yihe_liyong@126.com                                                *
* 电话 ：18615799380                                                        *
============================================================================*/

#include "bsp.h"

extern void TIM3_Set(u8 sta);                         // 设置TIM3的开关   sta:0，关闭   1，开启

/*===========================================================================
* 函数 ：SClK_Initial() => 初始化系统时钟，系统时钟 = 4MHZ                  *
============================================================================*/
void SClK_Initial(void)
{
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_4); //内部时钟为4分频 = 4Mhz 
}

/*===========================================================================
* 函数 ：GPIO_Initial() => 初始化通用IO端口                                 *
============================================================================*/
void GPIO_Initial(void)
{
    // 配置LED引脚    PC4
    GPIO_Init(PORT_LED, PIN_LED, GPIO_Mode_Out_PP_High_Fast);
    LED_OFF();        // 熄灭LED
    
    // 配置CSB引脚    PC0
    GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);
    CSB_Sleep();
    
    // 配置SWITCH引脚 PD1 PD2
    GPIO_Init(PORT_SWITCH, PIN_SWITCH, GPIO_Mode_Out_PP_High_Fast);
    GPIO_Init(PORT_SMGEN, PIN_SMGEN, GPIO_Mode_Out_PP_High_Fast);
    SWITCH_ON();     // 关闭CC1101电源
     
    // 配置CC1101相关控制引脚 CSN(PB4), IRQ(PB3), GDO2(PA3)
    GPIO_Init(PORT_CC_IRQ, PIN_CC_IRQ, GPIO_Mode_In_FL_No_IT);
    GPIO_Init(PORT_CC_GDO2, PIN_CC_GDO2, GPIO_Mode_In_PU_No_IT);
    
    GPIO_Init(PORT_CC_CSN, PIN_CC_CSN, GPIO_Mode_Out_PP_High_Fast);
    GPIO_SetBits(PORT_CC_CSN, PIN_CC_CSN);
}

/*===========================================================================
* 函数 USART1_Initial() => 初始化串口                                 *
============================================================================*/
void USART1_Initial(void)
{
    // 串口初始化
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE); //使能外设时钟，STM8L外设时钟默认关闭
    USART_Init(USART1,9600,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);//USART初始化，9600，8N1
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// 使能接收中断
    U1_Set(1);                                    // 使能USART1 
}

// ADC初始化     PA4  ADC1_IN2
void ADC_Initial(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);  // 使能ADC1时钟
    GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);     // 设置PA->4 为悬空输入，并中断禁止
    ADC_Init(ADC1,
             ADC_ConversionMode_Single,   // 单次转换模式
             ADC_Resolution_12Bit,        // 12位精度转换械
             ADC_Prescaler_2              // 时钟设置为2分频
             );  

    ADC_ChannelCmd(ADC1,
                   ADC_Channel_2,         // 设置为通道2进行采样
                   ENABLE);

    ADC_Cmd(ADC1 , ENABLE);               // 使能ADC  
    
    ADC_Data_Read();                      // 预先读取两次误差较大的值
    ADC_Data_Read();                      
}

// 读取ADC完成一次模数转换结果
uint16_t ADC_Data_Read(void)
{
  ADC_SoftwareStartConv(ADC1);      //启动ADC

  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);   // 等待转换结束
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);                   // 清除中断标志
  return ADC_GetConversionValue(ADC1);                // 读取ADC1，通道1的转换结果
}

/*===========================================================================
* 函数 ：SPI_Initial() => 初始化SPI                                         *
============================================================================*/
void SPI_Initial(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
    
    SPI_DeInit(SPI1);
    
    // 配置SPI相关参数,2分频（2MHZ）
    SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2,
         SPI_Mode_Master, SPI_CPOL_Low, SPI_CPHA_1Edge,
         SPI_Direction_2Lines_FullDuplex, SPI_NSS_Soft, 7);
  
    SPI_Cmd(SPI1,ENABLE);
    
    // SPI相关IO口配置
    GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_In_PU_No_IT);       // MISO (PB7)
    GPIO_Init(PORT_SPI, PIN_SCLK, GPIO_Mode_Out_PP_High_Fast);  // SCLK (PB5)
    GPIO_Init(PORT_SPI, PIN_MOSI, GPIO_Mode_Out_PP_High_Fast);  // MOSI (PB6)
}

/*===========================================================================
* 函数 ：TIM3_Initial() => 初始化定时器3，定时时间为1ms                     *
============================================================================*/
void TIM3_Initial(void)
{
    TIM3_DeInit();

    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

    // 配置Timer3相关参数，时钟为4/4 = 1MHZ，定时时间 = 1000/1000000 = 1ms
    TIM3_TimeBaseInit(TIM3_Prescaler_4, TIM3_CounterMode_Up, 1000);
    TIM3_ARRPreloadConfig(ENABLE);     // 使能定时器3自动重载功能  
    TIM3_Set(0);                       // 关闭TIM3
}

/*===========================================================================
* 函数 ：SPI_ExchangeByte() => 通过SPI进行数据交换                          * 
* 输入 ：需要写入SPI的值                                                    * 
* 输出 ：通过SPI读出的值                                                    * 
============================================================================*/
INT8U SPI_ExchangeByte(INT8U input)
{
    SPI_SendData(SPI1, input);
    while(RESET == SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE));   // 等待数据传输完成	
    while(RESET == SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE));  // 等待数据接收完成
    return(SPI_ReceiveData(SPI1));
}

void RTC_Initial(void)
{
#if RTC_CLK == RTC_CLK_LSI   // 内部38K时钟
    
    printf("RTC_CLK_LSI\r\n");
    
    CLK_LSICmd(ENABLE);                                          // 打开芯片内部的低速振荡器LSI
    while(CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);         // 等待振荡器稳定
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);   // 选择LSI作为RTC时钟源   1分频
                       
#else                       // 外部32K时钟
    
    printf("RTC_CLK_LSE\r\n");
    CLK_LSEConfig(CLK_LSE_ON);  
    while(CLK_GetFlagStatus(CLK_FLAG_LSERDY) == RESET);          // 等待振荡器稳定 
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSE, CLK_RTCCLKDiv_1);   // 选择LSE作为RTC时钟源   1分频

#endif
    
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC , ENABLE);    //使能实时时钟RTC时钟
    
    RTC_Set(22 , 15 , 26 , 16 , 7 , 23 , 6); //向实时时钟里设置，时分秒，年月日，星期分别是：22时15分20秒，2016年7月23日星期6
}

void RTC_Set(unsigned char hour , unsigned char min , unsigned char second , unsigned int year ,unsigned char month ,unsigned char day ,unsigned char week)
{ 
  RTC_InitTypeDef  RTCInit;
  RTC_TimeTypeDef RTCTime;
  RTC_DateTypeDef RTCData;
  
  RTC_WriteProtectionCmd(DISABLE);  //解除RTC数据保护
  
  RTC_EnterInitMode();    //设置RTC进入初始化模式，允许对RTC时间和日期寄存器进行设置
  while(RTC_GetFlagStatus(RTC_FLAG_INITF) == RESET);  //等待设置允许 等待INITF == 1完成允许设置

  RTCInit.RTC_HourFormat = RTC_HourFormat_24;
  RTCInit.RTC_AsynchPrediv = 37;
  RTCInit.RTC_SynchPrediv = 999;
  RTC_Init(&RTCInit);

  RTC_RatioCmd(ENABLE);

  RTCTime.RTC_Hours = hour;
  RTCTime.RTC_Minutes = min;
  RTCTime.RTC_Seconds = second;
  RTCTime.RTC_H12 = RTC_H12_AM;     //24
  RTC_SetTime(RTC_Format_BIN , &RTCTime);
  
  RTCData.RTC_WeekDay = (RTC_Weekday_TypeDef)week;   //RTC_Weekday_Sunday;    //sunday
  RTCData.RTC_Month = (RTC_Month_TypeDef)month;    //RTC_Month_August;      //8month
  RTCData.RTC_Date = day;      //14days
  RTCData.RTC_Year = year;
  RTC_SetDate(RTC_Format_BIN , &RTCData);
   
  RTC_ExitInitMode(); //不允许设置
  
  RTC_WriteProtectionCmd(ENABLE);//向密钥寄存器里进行写保护
}

// 设置TIM3的开关
// sta:0，关闭   1，开启
void TIM3_Set(u8 sta)
{
    if(sta)
    {  
        TIM3_SetCounter(0);     // 计数器清空
        TIM3_ITConfig(TIM3_IT_Update,ENABLE);   // 使能TIM3更新中断
        TIM3_Cmd(ENABLE);      // 使能TIM3	
    }
    else 
    {
        TIM3_Cmd(DISABLE);     // 关闭TIM3		   
        TIM3_ITConfig(TIM3_IT_Update,DISABLE);  // 关闭TIM3更新中断
    }
}

void CSB_Initial(void)
{
    GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);
    CSB_Sleep();
}

// 设置USART1的开关
// sta:0，关闭   1，开启
void U1_Set(u8 sta)
{
    if(sta) USART_Cmd(USART1, ENABLE);   // 使能USART1 
    else    USART_Cmd(USART1, DISABLE);  // 关闭USART1 
}
/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/
