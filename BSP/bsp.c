/*===========================================================================
* ��ַ ��http://www.cdebyte.com/   http://yhmcu.taobao.com/                 *
* ���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾                 * 
* �ʼ� ��yihe_liyong@126.com                                                *
* �绰 ��18615799380                                                        *
============================================================================*/

#include "bsp.h"

extern void TIM3_Set(u8 sta);                         // ����TIM3�Ŀ���   sta:0���ر�   1������

/*===========================================================================
* ���� ��SClK_Initial() => ��ʼ��ϵͳʱ�ӣ�ϵͳʱ�� = 4MHZ                  *
============================================================================*/
void SClK_Initial(void)
{
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_4); //�ڲ�ʱ��Ϊ4��Ƶ = 4Mhz 
}

/*===========================================================================
* ���� ��GPIO_Initial() => ��ʼ��ͨ��IO�˿�                                 *
============================================================================*/
void GPIO_Initial(void)
{
    // ����LED����       PC4
    GPIO_Init(PORT_LED, PIN_LED, GPIO_Mode_Out_PP_Low_Slow);          // ��ʼ��Ϊ�͵�ƽ    ����LED 
    
    // ����SWITCH����    PD1 PD2
    GPIO_Init(PORT_SWITCH, PIN_SWITCH, GPIO_Mode_Out_PP_Low_Slow);    // ��ʼ��Ϊ�͵�ƽ    �൱��SWITCH_ON
    GPIO_Init(PORT_SMGEN, PIN_SMGEN, GPIO_Mode_Out_PP_Low_Slow);      
     
    // ����CC1101��ؿ������� CSN(PB4), IRQ(PB3), GDO2(PA3)
    GPIO_Init(PORT_CC_IRQ, PIN_CC_IRQ, GPIO_Mode_In_FL_No_IT);
    GPIO_Init(PORT_CC_GDO2, PIN_CC_GDO2, GPIO_Mode_In_PU_No_IT);
    
    GPIO_Init(PORT_CC_CSN, PIN_CC_CSN, GPIO_Mode_Out_PP_High_Fast);  // CSN����
}

/*===========================================================================
* ���� USART1_Initial() => ��ʼ������                                 *
============================================================================*/
void USART1_Initial(void)
{
    // ���ڳ�ʼ��
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE); //ʹ������ʱ�ӣ�STM8L����ʱ��Ĭ�Ϲر�
    USART_Init(USART1,9600,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);//USART��ʼ����9600��8N1
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// ʹ�ܽ����ж�
    U1_Set(1);                                    // ʹ��USART1 
}

// ADC��ʼ��     PA4  ADC1_IN2
void ADC_Initial(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);  // ʹ��ADC1ʱ��
    GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);     // ����PA->4 Ϊ�������룬���жϽ�ֹ
    ADC_Init(ADC1,
             ADC_ConversionMode_Single,   // ����ת��ģʽ
             ADC_Resolution_12Bit,        // 12λ����ת��
             ADC_Prescaler_2              // ʱ������Ϊ2��Ƶ
             );  

    ADC_ChannelCmd(ADC1,
                   ADC_Channel_2,         // ����Ϊͨ��2���в���
                   ENABLE);

    ADC_Cmd(ADC1 , ENABLE);               // ʹ��ADC  
    
    ADC_Data_Read();                      // Ԥ�ȶ�ȡ�������ϴ��ֵ
    ADC_Data_Read();                      
}

// ��ȡADC���һ��ģ��ת�����
uint16_t ADC_Data_Read(void)
{
  ADC_SoftwareStartConv(ADC1);      //����ADC

  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);   // �ȴ�ת������
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);                   // ����жϱ�־
  return ADC_GetConversionValue(ADC1);                // ��ȡADC1��ͨ��1��ת�����
}

/*===========================================================================
* ���� ��SPI_Initial() => ��ʼ��SPI                                         *
============================================================================*/
void SPI_Initial(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
    
    SPI_DeInit(SPI1);
    
    // ����SPI��ز���,2��Ƶ��2MHZ��
    SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2,
         SPI_Mode_Master, SPI_CPOL_Low, SPI_CPHA_1Edge,
         SPI_Direction_2Lines_FullDuplex, SPI_NSS_Soft, 7);
  
    SPI_Cmd(SPI1,ENABLE);
    
    // SPI���IO������
    GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_In_PU_No_IT);       // MISO (PB7)
    GPIO_Init(PORT_SPI, PIN_SCLK, GPIO_Mode_Out_PP_High_Fast);  // SCLK (PB5)
    GPIO_Init(PORT_SPI, PIN_MOSI, GPIO_Mode_Out_PP_High_Fast);  // MOSI (PB6)
}

/*===========================================================================
* ���� ��TIM3_Initial() => ��ʼ����ʱ��3����ʱʱ��Ϊ1ms                     *
============================================================================*/
void TIM3_Initial(void)
{
    TIM3_DeInit();

    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

    // ����Timer3��ز�����ʱ��Ϊ4/4 = 1MHZ����ʱʱ�� = 1000/1000000 = 1ms
    TIM3_TimeBaseInit(TIM3_Prescaler_4, TIM3_CounterMode_Up, 1000);
    TIM3_ARRPreloadConfig(ENABLE);     // ʹ�ܶ�ʱ��3�Զ����ع���  
    TIM3_Set(0);                       // �ر�TIM3
}

/*===========================================================================
* ���� ��SPI_ExchangeByte() => ͨ��SPI�������ݽ���                          * 
* ���� ����Ҫд��SPI��ֵ                                                    * 
* ��� ��ͨ��SPI������ֵ                                                    * 
============================================================================*/
INT8U SPI_ExchangeByte(INT8U input)
{
    SPI_SendData(SPI1, input);
    while(RESET == SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE));   // �ȴ����ݴ������	
    while(RESET == SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE));  // �ȴ����ݽ������
    return(SPI_ReceiveData(SPI1));
}

void RTC_Initial(void)
{
#if RTC_CLK == RTC_CLK_LSI   // �ڲ�38Kʱ��
    
    printf("RTC_CLK_LSI\r\n");
    
    CLK_LSICmd(ENABLE);                                          // ��оƬ�ڲ��ĵ�������LSI
    while(CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);         // �ȴ������ȶ�
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);   // ѡ��LSI��ΪRTCʱ��Դ   1��Ƶ
                       
#else                       // �ⲿ32Kʱ��
    
    printf("RTC_CLK_LSE\r\n");
    CLK_LSEConfig(CLK_LSE_ON);  
    while(CLK_GetFlagStatus(CLK_FLAG_LSERDY) == RESET);          // �ȴ������ȶ� 
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSE, CLK_RTCCLKDiv_1);   // ѡ��LSE��ΪRTCʱ��Դ   1��Ƶ

#endif
    
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC , ENABLE);    //ʹ��ʵʱʱ��RTCʱ��
    
    RTC_Set(22 , 15 , 26 , 16 , 7 , 23 , 6); //��ʵʱʱ�������ã�ʱ���룬�����գ����ڷֱ��ǣ�22ʱ15��20�룬2016��7��23������6
}

void RTC_Set(unsigned char hour , unsigned char min , unsigned char second , unsigned int year ,unsigned char month ,unsigned char day ,unsigned char week)
{ 
  RTC_InitTypeDef  RTCInit;
  RTC_TimeTypeDef RTCTime;
  RTC_DateTypeDef RTCData;
  
  RTC_WriteProtectionCmd(DISABLE);  //���RTC���ݱ���
  
  RTC_EnterInitMode();    //����RTC�����ʼ��ģʽ�������RTCʱ������ڼĴ�����������
  while(RTC_GetFlagStatus(RTC_FLAG_INITF) == RESET);  //�ȴ��������� �ȴ�INITF == 1�����������

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
   
  RTC_ExitInitMode(); //����������
  
  RTC_WriteProtectionCmd(ENABLE);//����Կ�Ĵ��������д����
}

// ����TIM3�Ŀ���
// sta:0���ر�   1������
void TIM3_Set(u8 sta)
{
    if(sta)
    {  
        TIM3_SetCounter(0);     // ���������
        TIM3_ITConfig(TIM3_IT_Update,ENABLE);   // ʹ��TIM3�����ж�
        TIM3_Cmd(ENABLE);      // ʹ��TIM3	
    }
    else 
    {
        TIM3_Cmd(DISABLE);     // �ر�TIM3		   
        TIM3_ITConfig(TIM3_IT_Update,DISABLE);  // �ر�TIM3�����ж�
    }
}

void CSB_Initial(void)
{
    GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);   // ��ʼ��Ϊ�ߵ�ƽ ��Ч��CSB_Sleep 
}

// ����USART1�Ŀ���
// sta:0���ر�   1������
void U1_Set(u8 sta)
{
    if(sta) USART_Cmd(USART1, ENABLE);   // ʹ��USART1 
    else    USART_Cmd(USART1, DISABLE);  // �ر�USART1 
}
/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/
