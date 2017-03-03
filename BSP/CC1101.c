/*
================================================================================
Copyright   : Ebyte electronic co.,LTD
Website     : http://yh-ebyte.taobao.com
              http://yiheliyong.cn.alibaba.com
Description : This module contains the low level operations for CC1101
================================================================================
*/
#include "CC1101.h"
#include "bsp.h" 
#include "STM8l15x_conf.h"
#include "stdio.h"

////10, 7, 5, 0, -5, -10, -15, -20, dbm output power, 0x12 == -30dbm
//INT8U PaTabel[] = { 0xc0, 0xC8, 0x84, 0x60, 0x68, 0x34, 0x1D, 0x0E};

INT8U PaTabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0}; // 915MHz   10dBm
//INT8U PaTabel[8] = {0x03,0x0e,0x1e,0x27,0x8e,0x84,0xcc,0xc0};      // 915MHz   10dBm

// Sync word qualifier mode = 30/32 sync word bits detected 
// CRC autoflush = false 
// Channel spacing = 199.951172 
// Data format = Normal mode 
// Data rate = 2.00224 
// RX filter BW = 58.035714 
// PA ramping = false 
// Preamble count = 4 
// Whitening = false 
// Address config = No address check 
// Carrier frequency = 400.199890 
// Device address = 0 
// TX power = 10 
// Manchester enable = false 
// CRC enable = true 
// Deviation = 5.157471 
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Packet length = 255 
// Modulation format = GFSK 
// Base frequency = 399.999939 
// Modulated = true 
// Channel number = 1 

// RF = 915MHz
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
    INT8U FSCTRL2;		//自已加的
    INT8U FSCTRL1;   // Frequency synthesizer control.
    INT8U FSCTRL0;   // Frequency synthesizer control.
    INT8U FREQ2;     // Frequency control word, high INT8U.
    INT8U FREQ1;     // Frequency control word, middle INT8U.
    INT8U FREQ0;     // Frequency control word, low INT8U.
    INT8U MDMCFG4;   // Modem configuration.
    INT8U MDMCFG3;   // Modem configuration.
    INT8U MDMCFG2;   // Modem configuration.
    INT8U MDMCFG1;   // Modem configuration.
    INT8U MDMCFG0;   // Modem configuration.
    INT8U CHANNR;    // Channel number.
    INT8U DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    INT8U FREND1;    // Front end RX configuration.
    INT8U FREND0;    // Front end RX configuration.
    INT8U MCSM0;     // Main Radio Control State Machine configuration.
    INT8U FOCCFG;    // Frequency Offset Compensation Configuration.
    INT8U BSCFG;     // Bit synchronization Configuration.
    INT8U AGCCTRL2;  // AGC control.
    INT8U AGCCTRL1;  // AGC control.
    INT8U AGCCTRL0;  // AGC control.
    INT8U FSCAL3;    // Frequency synthesizer calibration.
    INT8U FSCAL2;    // Frequency synthesizer calibration.
    INT8U FSCAL1;    // Frequency synthesizer calibration.
    INT8U FSCAL0;    // Frequency synthesizer calibration.
    INT8U FSTEST;    // Frequency synthesizer calibration control
    INT8U TEST2;     // Various test settings.
    INT8U TEST1;     // Various test settings.
    INT8U TEST0;     // Various test settings.
    INT8U IOCFG2;    // GDO2 output pin configuration
    INT8U IOCFG0;    // GDO0 output pin configuration
    INT8U PKTCTRL1;  // Packet automation control.
    INT8U PKTCTRL0;  // Packet automation control.
    INT8U ADDR;      // Device address.
    INT8U PKTLEN;    // Packet length.
} RF_SETTINGS;

/////////////////////////////////////////////////////////////////
const RF_SETTINGS rfSettings = 
{
    0x00,
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x23,   // FREQ2     Frequency control word, high byte.
    0x31,   // FREQ1     Frequency control word, middle byte.
    0x3B,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.  空中波特率是100K
    0x03,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.

    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end RX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.

    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x11,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. Refer to SmartRF?Studio User Manual for detailed pseudo register explanation.

    0x04,   // PKTCTRL1  Packet automation control.
    0x45,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0x0c    // PKTLEN    Packet length.
};

/*read a byte from the specified register*/
INT8U CC1101ReadReg(INT8U addr);

/*Read some bytes from the rigisters continously*/
void CC1101ReadMultiReg(INT8U addr, INT8U *buff, INT8U size);

/*Write a byte to the specified register*/
void CC1101WriteReg(INT8U addr, INT8U value);

/*Flush the TX buffer of CC1101*/
void CC1101ClrTXBuff(void);

/*Flush the RX buffer of CC1101*/
void CC1101ClrRXBuff(void);

/*Get received count of CC1101*/
INT8U CC1101GetRXCnt(void);

/*Reset the CC1101 device*/
void CC1101Reset(void);

/*Write some bytes to the specified register*/
void CC1101WriteMultiReg(INT8U addr, INT8U *buff, INT8U size);

extern INT8U SPI_ExchangeByte(INT8U input); // 通过SPI进行数据交换,见bsp.c
extern void DelayMs(u16 x); 

/*
================================================================================
Function : CC1101WORInit( )
    Initialize the WOR function of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void  CC1101WORInit( void )
{

    CC1101WriteReg(CC1101_MCSM0,0x18);
    CC1101WriteReg(CC1101_WORCTRL,0x78); //Wake On Radio Control
    CC1101WriteReg(CC1101_MCSM2,0x00);
    CC1101WriteReg(CC1101_WOREVT1,0x8C);
    CC1101WriteReg(CC1101_WOREVT0,0xA0);
	
    CC1101WriteCmd(CC1101_SWORRST);
}
/*
================================================================================
Function : CC1101ReadReg( )
    read a byte from the specified register
INPUT    : addr, The address of the register
OUTPUT   : the byte read from the rigister
================================================================================
*/
INT8U CC1101ReadReg(INT8U addr)
{
    INT8U i;
    CC_CSN_LOW();
    SPI_ExchangeByte(addr | READ_SINGLE);
    i = SPI_ExchangeByte(0xFF);
    CC_CSN_HIGH();
    return i;
}
/*
================================================================================
Function : CC1101ReadMultiReg()
    Read some bytes from the rigisters continously
INPUT    : addr, The address of the register
           buff, The buffer stores the data
           size, How many bytes should be read
OUTPUT   : None
================================================================================
*/
void CC1101ReadMultiReg( INT8U addr, INT8U *buff, INT8U size )
{
    INT8U i, j;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_BURST);
    for( i = 0; i < size; i ++ )
    {
        for( j = 0; j < 20; j ++ );
        *( buff + i ) = SPI_ExchangeByte( 0xFF );
    }
    CC_CSN_HIGH( );
}
/*
================================================================================
Function : CC1101ReadStatus( )
    Read a status register
INPUT    : addr, The address of the register
OUTPUT   : the value read from the status register
================================================================================
*/
INT8U CC1101ReadStatus( INT8U addr )
{
    INT8U i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_BURST);
    i = SPI_ExchangeByte( 0xFF );
    CC_CSN_HIGH( );
    return i;
}
/*
================================================================================
Function : CC1101SetTRMode( )
    Set the device as TX mode or RX mode
INPUT    : mode selection
OUTPUT   : None
================================================================================
*/
void CC1101SetTRMode( TRMODE mode )
{
    if( mode == TX_MODE )
    {
        CC1101WriteReg(CC1101_IOCFG0,0x46);
        CC1101WriteCmd( CC1101_STX );
    }
    else if( mode == RX_MODE )
    {
        CC1101WriteReg(CC1101_IOCFG0,0x46);
        CC1101WriteCmd( CC1101_SRX );
    }
}
/*
================================================================================
Function : CC1101WriteReg( )
    Write a byte to the specified register
INPUT    : addr, The address of the register
           value, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteReg(INT8U addr, INT8U value)
{
    CC_CSN_LOW();
    SPI_ExchangeByte(addr);
    SPI_ExchangeByte(value);
    CC_CSN_HIGH();
}
/*
================================================================================
Function : CC1101WriteMultiReg()
    Write some bytes to the specified register
INPUT    : addr, The address of the register
           buff, a buffer stores the values
           size, How many byte should be written
OUTPUT   : None
================================================================================
*/
void CC1101WriteMultiReg( INT8U addr, INT8U *buff, INT8U size )
{
    INT8U i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | WRITE_BURST );
    for( i = 0; i < size; i ++ )
    {
        SPI_ExchangeByte( *( buff + i ) );
    }
    CC_CSN_HIGH( );
}
/*
================================================================================
Function : CC1101WriteCmd( )
    Write a command byte to the device
INPUT    : command, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteCmd( INT8U command )
{
    CC_CSN_LOW();
    SPI_ExchangeByte(command);
    CC_CSN_HIGH();
}
/*
================================================================================
Function : CC1101Reset( )
    Reset the CC1101 device
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Reset(void)
{
    u8 x;

    CC_CSN_HIGH();
    CC_CSN_LOW();
    CC_CSN_HIGH();
    for(x = 0; x < 100; x ++);        // 至少40us
    CC1101WriteCmd(CC1101_SRES);
}
/*
================================================================================
Function : CC1101SetIdle( )
    Set the CC1101 into IDLE mode
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101SetIdle( void )
{
    CC1101WriteCmd(CC1101_SIDLE);
}
/*
================================================================================
Function : CC1101ClrTXBuff( )
    Flush the TX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrTXBuff( void )
{
    CC1101SetIdle();//MUST BE IDLE MODE
    CC1101WriteCmd( CC1101_SFTX );
}
/*
================================================================================
Function : CC1101ClrRXBuff( )
    Flush the RX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrRXBuff( void )
{
    CC1101SetIdle();//MUST BE IDLE MODE
    CC1101WriteCmd( CC1101_SFRX );
}
/*
================================================================================
Function : CC1101SendPacket( )
    Send a packet
INPUT    : txbuffer, The buffer stores data to be sent
           size, How many bytes should be sent
           mode, Broadcast or address check packet
OUTPUT   : None
================================================================================
*/
void CC1101SendPacket( INT8U *txbuffer, INT8U size, TX_DATA_MODE mode )
{
    uint8_t address;
    static uint8_t flag = 0;
	
    if(mode == BROADCAST)          address = 0;
    else if(mode == ADDRESS_CHECK) address = CC1101ReadReg(CC1101_ADDR);  // 本机地址

    if(flag == 0)    // 仅第一次打印
    {
        printf("local_address:%d\r\n", (int)address);
        flag = 1;
    }
    CC1101ClrTXBuff();
    
    if((CC1101ReadReg(CC1101_PKTCTRL1)& ~0x03)!= 0)
    { 
        address = RX_Address;
        CC1101WriteReg(CC1101_TXFIFO, size + 1);
        CC1101WriteReg(CC1101_TXFIFO, address);
    }
    else
    {
        CC1101WriteReg(CC1101_TXFIFO, size);
    }

    CC1101WriteMultiReg(CC1101_TXFIFO, txbuffer, size);
    CC1101SetTRMode(TX_MODE);
    while(CC_IRQ_READ()!= 0);
    while(CC_IRQ_READ()== 0);

    CC1101ClrTXBuff();
}
/*
================================================================================
Function : CC1101GetRXCnt( )
    Get received count of CC1101
INPUT    : None
OUTPUT   : How many bytes hae been received
================================================================================
*/
INT8U CC1101GetRXCnt( void )
{
    return ( CC1101ReadStatus( CC1101_RXBYTES )  & BYTES_IN_RXFIFO );
}
/*
================================================================================
Function : CC1101SetAddress( )
    Set the address and address mode of the CC1101
INPUT    : address, The address byte
           AddressMode, the address check mode
OUTPUT   : None
================================================================================
*/
void CC1101SetAddress( INT8U address, ADDR_MODE AddressMode)
{
    INT8U btmp = CC1101ReadReg( CC1101_PKTCTRL1 ) & ~0x03;
    CC1101WriteReg(CC1101_ADDR, address);
    if     ( AddressMode == BROAD_ALL )     {}
    else if( AddressMode == BROAD_NO  )     { btmp |= 0x01; }
    else if( AddressMode == BROAD_0   )     { btmp |= 0x02; }
    else if( AddressMode == BROAD_0AND255 ) { btmp |= 0x03; }   
}
/*
================================================================================
Function : CC1101SetSYNC( )
    Set the SYNC bytes of the CC1101
INPUT    : sync, 16bit sync 
OUTPUT   : None
================================================================================
*/
void CC1101SetSYNC( INT16U sync )
{
    CC1101WriteReg(CC1101_SYNC1, 0xFF & ( sync>>8 ) );
    CC1101WriteReg(CC1101_SYNC0, 0xFF & sync ); 
}
/*
================================================================================
Function : CC1101RecPacket( )
    Receive a packet
INPUT    : rxBuffer, A buffer store the received data
OUTPUT   : 1:received count, 0:no data
================================================================================
*/
INT8U CC1101RecPacket(INT8U *rxBuffer)
{
    uint8_t status[2], pktLen;

    if(CC1101GetRXCnt()!= 0)
    {
        pktLen = CC1101ReadReg(CC1101_RXFIFO) & 0xff;        // Read length byte
        if((CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03)!= 0)
        {
            CC1101ReadReg(CC1101_RXFIFO);
        }
        if(pktLen <= 0 || pktLen > 10) return 0;
        else                           pktLen --;
        CC1101ReadMultiReg(CC1101_RXFIFO, rxBuffer, pktLen); // Pull data
        CC1101ReadMultiReg(CC1101_RXFIFO, status, 2);        // Read  status bytes

        CC1101ClrRXBuff();

        if(status[1] & CRC_OK) return pktLen; 
        else                   return 0; 
    }
    else return 0;                               // Error
}
/*
================================================================================
Function : CC1101Init( )
    Initialize the CC1101, User can modify it
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Init( void )
{
    TIM3_Initial();         // 初始化定时器3，基准1ms  
    SPI_Initial();          // 初始化SPI  
    
    CC1101Reset();          // CC1101复位
    
    CC1101_Settings();
    printf("CC1101_FSCTRL1：%d\r\n", (int)CC1101ReadReg(CC1101_FSCTRL1));
    
    CC1101SetAddress(TX_Address, BROAD_0AND255);  // 从机地址
    CC1101SetSYNC(0xD391);                        // 8799
    CC1101WriteReg(CC1101_MDMCFG1, 0x22);         // Modem Configuration      
    CC1101WriteReg(CC1101_MDMCFG0, 0xF8);

//    CC1101WriteMultiReg(CC1101_PATABLE, PaTabel+1, 1);  // 发射功率
    CC1101WriteMultiReg(CC1101_PATABLE, PaTabel, 8);
}

void CC1101_Settings(void) 
{
    CC1101WriteReg(CC1101_FSCTRL0,  rfSettings.FSCTRL2);//自已加的
    // Write register settings
    CC1101WriteReg(CC1101_FSCTRL1,  rfSettings.FSCTRL1);
    CC1101WriteReg(CC1101_FSCTRL0,  rfSettings.FSCTRL0);
    CC1101WriteReg(CC1101_FREQ2,    rfSettings.FREQ2);
    CC1101WriteReg(CC1101_FREQ1,    rfSettings.FREQ1);
    CC1101WriteReg(CC1101_FREQ0,    rfSettings.FREQ0);
    CC1101WriteReg(CC1101_MDMCFG4,  rfSettings.MDMCFG4);
    CC1101WriteReg(CC1101_MDMCFG3,  rfSettings.MDMCFG3);
    CC1101WriteReg(CC1101_MDMCFG2,  rfSettings.MDMCFG2);
    CC1101WriteReg(CC1101_MDMCFG1,  rfSettings.MDMCFG1);
    CC1101WriteReg(CC1101_MDMCFG0,  rfSettings.MDMCFG0);
    CC1101WriteReg(CC1101_CHANNR,   rfSettings.CHANNR);
    CC1101WriteReg(CC1101_DEVIATN,  rfSettings.DEVIATN);
    CC1101WriteReg(CC1101_FREND1,   rfSettings.FREND1);
    CC1101WriteReg(CC1101_FREND0,   rfSettings.FREND0);
    CC1101WriteReg(CC1101_MCSM0 ,   rfSettings.MCSM0 );
    CC1101WriteReg(CC1101_FOCCFG,   rfSettings.FOCCFG);
    CC1101WriteReg(CC1101_BSCFG,    rfSettings.BSCFG);
    CC1101WriteReg(CC1101_AGCCTRL2, rfSettings.AGCCTRL2);
    CC1101WriteReg(CC1101_AGCCTRL1, rfSettings.AGCCTRL1);
    CC1101WriteReg(CC1101_AGCCTRL0, rfSettings.AGCCTRL0);
    CC1101WriteReg(CC1101_FSCAL3,   rfSettings.FSCAL3);
    CC1101WriteReg(CC1101_FSCAL2,   rfSettings.FSCAL2);
    CC1101WriteReg(CC1101_FSCAL1,   rfSettings.FSCAL1);
    CC1101WriteReg(CC1101_FSCAL0,   rfSettings.FSCAL0);
    CC1101WriteReg(CC1101_FSTEST,   rfSettings.FSTEST);
    CC1101WriteReg(CC1101_TEST2,    rfSettings.TEST2);
    CC1101WriteReg(CC1101_TEST1,    rfSettings.TEST1);
    CC1101WriteReg(CC1101_TEST0,    rfSettings.TEST0);
    CC1101WriteReg(CC1101_IOCFG2,   rfSettings.IOCFG2);
    CC1101WriteReg(CC1101_IOCFG0,   rfSettings.IOCFG0);    
    CC1101WriteReg(CC1101_PKTCTRL1, rfSettings.PKTCTRL1);
    CC1101WriteReg(CC1101_PKTCTRL0, rfSettings.PKTCTRL0);
    CC1101WriteReg(CC1101_ADDR,     rfSettings.ADDR);
    CC1101WriteReg(CC1101_PKTLEN,   rfSettings.PKTLEN);
}

// 设置cc1101进入低功耗模式
void CC1101SetLowPower(void)
{
    CC1101WriteCmd(CC1101_SPWD);
    CC_CSN_HIGH();
}

/*
================================================================================
------------------------------------THE END-------------------------------------
================================================================================
*/
