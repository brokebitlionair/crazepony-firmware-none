#include "RF.H"
#include "stdio.h"
#include "ReceiveData.h"
	
#define IRQ	  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)
#define MISO	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)

#define CE_H GPIO_SetBits(GPIOA,GPIO_Pin_12)
#define CE_L GPIO_ResetBits(GPIOA,GPIO_Pin_12)

#define CSN_H GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define CSN_L GPIO_ResetBits(GPIOA,GPIO_Pin_4)

#define SCK_H GPIO_SetBits(GPIOA,GPIO_Pin_5)
#define SCK_L GPIO_ResetBits(GPIOA,GPIO_Pin_5)

#define MOSI_H GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define MOSI_L GPIO_ResetBits(GPIOA,GPIO_Pin_7)

//uint8_t  TX_ADDRESS_DEF[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};    		//RF 地址：接收端和发送端需一致
uint8_t  TX_ADDRESS_DEF[5] = {0xF5,0x1F,0x00,0xBF,0x03};    		//RF 地址：接收端和发送端需一致

uint8_t ucCurrent_Channel;
uint8_t RF_RXDATA[7];

/******************************************************************************/
//            SPI_init
//               init spi pin and IRQ  CE input/out mode
/******************************************************************************/	
void spi_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_12;//CE   CSN    SCK   MOSI
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 |GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //MISO  IRQ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	CE_L;
	CSN_H;
	SCK_L;

	printf("SPI bus init success...\r\n");
}

/******************************************************************************/
//            SPI_RW
//                SPI Write/Read Data
//            SPI写入一个BYTE的同时，读出一个BYTE返回
/******************************************************************************/
unsigned char SPI_RW(uint8_t R_REG)
{
    uint8_t	  i;
    for(i = 0; i < 8; i++)
    {
        SCK_L;
        if(R_REG & 0x80)MOSI_H;
        else MOSI_L;	
        R_REG = R_REG << 1;
        SCK_H;
        if(MISO==1)
        {
          R_REG = R_REG | 0x01;
        }
    }
    SCK_L;
    return R_REG;
}

/******************************************************************************/
//            RF_WriteReg
//                Write Data(1 Byte Address ,1 byte data)
/******************************************************************************/
void RF_WriteReg( uint8_t reg,  uint8_t wdata)
{
    CSN_L;
    SPI_RW(reg);
    SPI_RW(wdata);
    CSN_H;
}


/******************************************************************************/
//            RF_ReadReg
//                Read Data(1 Byte Address ,1 byte data return)
/******************************************************************************/
 uint8_t ucRF_ReadReg( uint8_t reg)
{
    uint8_t tmp;   
    CSN_L;
    SPI_RW(reg);
    tmp = SPI_RW(0);
    CSN_H;
    
    return tmp;
}

/******************************************************************************/
//            RF_WriteBuf
//                Write Buffer
/******************************************************************************/
void RF_WriteBuf( uint8_t reg, uint8_t *pBuf, uint8_t length)
{
    uint8_t j;
    CSN_L;
    j = 0;
    SPI_RW(reg);
    for(j = 0;j < length; j++)
    {
        SPI_RW(pBuf[j]);
    }
    j = 0;
    CSN_H;
}

/******************************************************************************/
//            RF_ReadBuf
//                Read Data(1 Byte Address ,length byte data read)
/******************************************************************************/
void RF_ReadBuf( uint8_t reg, unsigned char *pBuf,  uint8_t length)
{
    uint8_t byte_ctr;

    CSN_L;                    		                               			
    SPI_RW(reg);       		                                                		
    for(byte_ctr=0;byte_ctr<length;byte_ctr++)
    pBuf[byte_ctr] = SPI_RW(0);                                                 		
    CSN_H;                                                                   		
}



/******************************************************************************/
//            RF_TxMode
//                Set RF into TX mode
/******************************************************************************/
void RF_TxMode(void)
{
    CE_L;
    RF_WriteReg(W_REGISTER + CONFIG,  0X8E);							// 将RF设置成TX模式
    delay_us(10);
}


/******************************************************************************/
//            RF_RxMode
//            将RF设置成RX模式，准备接收数据
/******************************************************************************/
void RF_RxMode(void)
{
    CE_L;
    RF_WriteReg(W_REGISTER + CONFIG,  0X8F );							// 将RF设置成RX模式
    CE_H;											// Set CE pin high 开始接收数据
    delay_ms(2);
}

/******************************************************************************/
//            RF_GetStatus
//            read RF IRQ status,3bits return
/******************************************************************************/
unsigned char ucRF_GetStatus(void)
{
   uint8_t state;
   state=ucRF_ReadReg(STATUS)&0x70;
   return state;								//读取RF的状态 
}

/******************************************************************************/
//            RF_ClearStatus
//                clear RF IRQ
/******************************************************************************/
void RF_ClearStatus(void)
{
    RF_WriteReg(W_REGISTER + STATUS,0x70);							//清除RF的IRQ标志 
}

/******************************************************************************/
//            RF_ClearFIFO
//                clear RF TX/RX FIFO
/******************************************************************************/
void RF_ClearFIFO(void)
{
    RF_WriteReg(FLUSH_TX, 0);			                                		//清除RF 的 TX FIFO		
    RF_WriteReg(FLUSH_RX, 0);                                                   		//清除RF 的 RX FIFO	
}

/******************************************************************************/
//            RF_SetChannel
//                Set RF TX/RX channel:Channel
/******************************************************************************/
void RF_SetChannel( uint8_t Channel)
{    
    CE_L;
		ucCurrent_Channel = Channel;
    RF_WriteReg(W_REGISTER + RF_CH, Channel);
}

/******************************************************************************/
//            发送数据：
//            参数：
//              1. ucPayload：需要发送的数据首地址
//              2. length:  需要发送的数据长度
//              Return:
//              1. MAX_RT: TX Failure  (Enhance mode)
//              2. TX_DS:  TX Successful (Enhance mode)
//              note: Only use in Tx Mode
//              length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/

void ucRF_TxData( uint8_t *ucPayload,  uint8_t length)
{
    /*if(0==ucRF_GetStatus())                                                                        // rf free status                                                                                                                                                                   
   {
    RF_WriteBuf(W_TX_PAYLOAD, ucPayload, length); 
    CE=1;                                                                    		//rf entery tx mode start send data 
    Delay20us();                                                              		//keep ce high at least 600us
    CE=0;                                                                                     //rf entery stb3                                                        			
   } */
		uint8_t   Status_Temp;
    
    CE_H;                                                                    		//rf entery tx mode start send data 
    delay_us(20);                                                             		//keep ce high at least 16us
    CE_L;
		RF_WriteBuf(W_TX_PAYLOAD, ucPayload, length);                               		//write data to txfifo        
  
		CE_H;                                                                    		//rf entery tx mode start send data 
    delay_us(20);                                                             		//keep ce high at least 16us
    CE_L;
			
				                                                                   		//rf entery stb3
    while(IRQ);                                                          		//waite irq pin low 
    Status_Temp = ucRF_ReadReg(STATUS) & 0x70;                                                  //读取发送完成后的status
    RF_WriteReg(W_REGISTER + STATUS, Status_Temp);                                 		//清除Status
    RF_WriteReg(FLUSH_TX,0); 
	/* if(ucRF_ReadReg(OBSERVE_TX) > 0xCF)						
    {
	RF_SetChannel(ucCurrent_Channel);					//清0 OBSERVE_TX
    } */                                                  			//清 FIFO
     	
    //return Status_Temp;
   
   	
}	


/******************************************************************************/
//            ucRF_DumpRxData
//            读出接收到的数据：
//            参数：
//              1. ucPayload：存储读取到的数据的Buffer
//              2. length:    读取的数据长度
//              Return:
//              1. 0: 没有接收到数据
//              2. 1: 读取接收到的数据成功
//              note: Only use in Rx Mode
//              length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/
unsigned char ucRF_DumpRxData( uint8_t *ucPayload,  uint8_t length)
{
    if(IRQ==1)
    {
      return 0;                                                                 		//若IRQ PIN为高，则没有接收到数据
    }

    CE_L;
    RF_ReadBuf(R_RX_PAYLOAD, ucPayload, length);                                		//将接收到的数据读出到ucPayload，且清除rxfifo	
    RF_WriteReg(FLUSH_RX, 0);	
    RF_WriteReg(W_REGISTER + STATUS, 0x70);                                     		//清除Status
    CE_H;                                                                    		//继续开始接收  		
		
		ReceiveDataFormRF();
		
    return 1;
}


////////////////////////////////////////////////////////////////////////////////

//          以下部分与RF通信相关，不建议修改
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
//            PN006_Initial
//                Initial RF
/******************************************************************************/
void RF_Init(void)
{
    uint8_t  BB_cal_data[]    = { 0x12,0xec,0x6f,0xa1,0x46}; 
    uint8_t  RF_cal_data[]    = {0xf6,0x37,0x5d};
    uint8_t  RF_cal2_data[]   = {0xd5,0x21,0xeb,0x2c,0x5a,0x40};
    uint8_t  Dem_cal_data[]   = {0x1f};   
    uint8_t  Dem_cal2_data[]  = {0x0b,0xdf,0x02};   


	//IRQ_STATUS=0;
    spi_init();
    RF_Check();
		CE_L;
    CSN_H;
    SCK_L;		
    ucCurrent_Channel = DEFAULT_CHANNEL;                
    RF_WriteReg(RST_FSPI, 0x5A);								//Software Reset    			
    RF_WriteReg(RST_FSPI, 0XA5);
    
    RF_WriteReg(FLUSH_TX, 0);									// CLEAR TXFIFO		    			 
    RF_WriteReg(FLUSH_RX, 0);									// CLEAR  RXFIFO
    RF_WriteReg(W_REGISTER + STATUS, 0x70);							// CLEAR  STATUS	
    RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);							// Enable Pipe0
    RF_WriteReg(W_REGISTER + SETUP_AW,  0x03);							// address witdth is 5 bytes
    RF_WriteReg(W_REGISTER + RF_CH,     DEFAULT_CHANNEL);                                       // 2478M HZ
    RF_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);						// 8 bytes
    RF_WriteBuf(W_REGISTER + TX_ADDR,   ( uint8_t*)TX_ADDRESS_DEF, sizeof(TX_ADDRESS_DEF));	// Writes TX_Address to PN006
    RF_WriteBuf(W_REGISTER + RX_ADDR_P0,( uint8_t*)TX_ADDRESS_DEF, sizeof(TX_ADDRESS_DEF));	// RX_Addr0 same as TX_Adr for Auto.Ack   
    RF_WriteBuf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
    RF_WriteReg(W_REGISTER + DYNPD, 0x00);					
    RF_WriteReg(W_REGISTER + FEATURE, 0x00);
    RF_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);						//DBM  		
  
    
//#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)      
//    RF_WriteReg(W_REGISTER + SETUP_RETR,0x03);							//  3 retrans... 	
//    RF_WriteReg(W_REGISTER + EN_AA,     0x01);							// Enable Auto.Ack:Pipe0  	
//#elif(TRANSMIT_TYPE == TRANS_BURST_MODE)                                                                
    RF_WriteReg(W_REGISTER + SETUP_RETR,0x00);							// Disable retrans... 	
    RF_WriteReg(W_REGISTER + EN_AA,     0x00);							// Disable AutoAck 
//#endif

if(PAYLOAD_WIDTH <33)											
{
	RF_WriteReg(W_REGISTER +FEATURE, 0x00);							//切换到32byte模式
}
else
{
	RF_WriteReg(W_REGISTER +FEATURE, 0x18);							//切换到64byte模式	   
}

}


/******************************************************************************/
//            		进入载波模式
/******************************************************************************/
void RF_Carrier( uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46}; 
    uint8_t RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t RF_cal2_data[]   = {0x45,0x21,0xEF,0xAC,0x5A,0x50};
    uint8_t Dem_cal_data[]   = {0xE1}; 								
    uint8_t Dem_cal2_data[]  = {0x0B,0xDF,0x02};      
    
    CE_L;
    RF_WriteReg(RST_FSPI, 0x5A);								//Software Reset    			
    RF_WriteReg(RST_FSPI, 0XA5);
    //Delay200ms();
    RF_WriteReg(W_REGISTER + RF_CH, ucChannel_Set);						//单载波频点	   
    RF_WriteReg(W_REGISTER + RF_SETUP, RF_POWER);      						//dbm
    RF_WriteBuf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
    delay_us(5000);	
}

uint8_t RF_Check(void) 
{ 
   uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2}; 
   uint8_t buf1[5]; 
   uint8_t i=0; 

   RF_WriteBuf(W_REGISTER + TX_ADDR, buf, 5);     
   RF_ReadBuf(TX_ADDR,buf1,5); 
    
   for (i=0;i<5;i++) 
   { 
      if (buf1[i]!=0xC2) 
      break; 
   } 
  
   if (i==5)   {printf("RF found...\r\n");return 1 ;}
   else        {printf("RF check failed...\r\n");return 0 ;}
} 

/***************************************end of file ************************************/
