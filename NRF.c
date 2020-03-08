#include "NRF.h"
//------------------------------------------------
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
//------------------------------------------------
#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 5
uint8_t TX_ADDRESS0[TX_ADR_WIDTH] = {0xb7,0xb5,0xa1};
uint8_t TX_ADDRESS1[TX_ADR_WIDTH] = {0xb5,0xb5,0xa1};
uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};
volatile uint8_t rx_flag = 0, tx_flag = 0;

uint8_t Rx_str[200];
uint8_t Rx_index=0;

//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 9;
  /* Wait till done */
  while (micros--) ;
}
//--------------------------------------------------
uint8_t NRF24_ReadReg(uint8_t addr)
{
  uint8_t dt=0, cmd;
  CS_ON;
  HAL_SPI_TransmitReceive(&hspi1,&addr,&dt,1,1000);
  if (addr!=STATUS)//åñëè àäðåñ ðàâåí àäðåñ ðåãèñòðà ñòàòóñ òî è âîçâàðùàåì åãî ñîñòîÿíèå
  {
    cmd=0xFF;
    HAL_SPI_TransmitReceive(&hspi1,&cmd,&dt,1,1000);
  }
  CS_OFF;
  return dt;
}
//------------------------------------------------
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
  addr |= W_REGISTER;//âêëþ÷èì áèò çàïèñè â àäðåñ
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//îòïðàâèì àäðåñ â øèíó
  HAL_SPI_Transmit(&hspi1,&dt,1,1000);//îòïðàâèì äàííûå â øèíó
  CS_OFF;
}
//------------------------------------------------
void NRF24_ToggleFeatures(void)
{
  uint8_t dt[1] = {ACTIVATE};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  CS_OFF;
}
//-----------------------------------------------
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//îòïðàâèì àäðåñ â øèíó
  HAL_SPI_Receive(&hspi1,pBuf,bytes,1000);//îòïðàâèì äàííûå â áóôåð
  CS_OFF;
}
//------------------------------------------------
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  addr |= W_REGISTER;//âêëþ÷èì áèò çàïèñè â àäðåñ
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//îòïðàâèì àäðåñ â øèíó
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//îòïðàâèì äàííûå â áóôåð
  CS_OFF;
}
//------------------------------------------------
void NRF24_FlushRX(void)
{
  uint8_t dt[1] = {FLUSH_RX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
void NRF24_FlushTX(void)
{
  uint8_t dt[1] = {FLUSH_TX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
void NRF24L01_RX_Mode(void)
{
  uint8_t regval=0x00;
  regval = NRF24_ReadReg(CONFIG);
  //ðàçáóäèì ìîäóëü è ïåðåâåä¸ì åãî â ðåæèì ïðè¸ìíèêà, âêëþ÷èâ áèòû PWR_UP è PRIM_RX
  regval |= (1<<PWR_UP)|(1<<PRIM_RX);
  NRF24_WriteReg(CONFIG,regval);
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS1, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS1, TX_ADR_WIDTH);
  CE_SET;
  DelayMicro(150);
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//------------------------------------------------
void NRF24L01_TX_Mode(uint8_t *pBuf)
{
  NRF24_Write_Buf(TX_ADDR, TX_ADDRESS0, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS0, TX_ADR_WIDTH);
  CE_RESET;
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//------------------------------------------------
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CE_RESET;
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//îòïðàâèì àäðåñ â øèíó
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//îòïðàâèì äàííûå â áóôåð
  CS_OFF;
  CE_SET;
}
//------------------------------------------------
uint8_t NRF24L01_Send(uint8_t *pBuf)
{
 uint8_t regval=0x00;
  NRF24L01_TX_Mode(pBuf);
  regval = NRF24_ReadReg(CONFIG);
  //åñëè ìîäóëü óøåë â ñïÿùèé ðåæèì, òî ðàçáóäèì åãî, âêëþ÷èâ áèò PWR_UP è âûêëþ÷èâ PRIM_RX
  regval |= (1<<PWR_UP);
  regval &= ~(1<<PRIM_RX);
  NRF24_WriteReg(CONFIG,regval);
  DelayMicro(150); //Çàäåðæêà ìèíèìóì 130 ìêñ
  //Îòïðàâèì äàííûå â âîçäóõ
  NRF24_Transmit(WR_TX_PLOAD, pBuf, TX_PLOAD_WIDTH);
  CE_SET;
  DelayMicro(15); //minimum 10us high pulse (Page 21)
  CE_RESET;
  return 0;
}
//------------------------------------------------
void NRF24L01_Receive(void)
{
// if(rx_flag==1)
//	{
//		HAL_Delay(1);
//				LED_TGL;
//		for(int i=0;i<Tx_index;i+=5)
//		{
//			for(int j=0;j<5;j++)
//			{
//				if((i+j)<Tx_index)
//				TX_BUF[j]=Tx_str[i+j];
//				else
//					TX_BUF[j]=0;
//			}		
//			NRF24L01_Send(TX_BUF);
//		HAL_Delay(1);
//		}
//	//	NRF24L01_Send(Tx_str);
//		rx_flag = 0;
//	}
}

void IRQ_Callback(void)
{
  uint8_t status=0x01;
  DelayMicro(10);
  status = NRF24_ReadReg(STATUS);
  if(status & 0x40)
  { 
    NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
    NRF24_WriteReg(STATUS, 0x40);
		for(int i=0;i<5;i++)
		{
			if(RX_BUF[i]==';')
			{
				//LED_TGL;
				Rx_str[Rx_index]=RX_BUF[i];
				Rx_index++;
				rx_flag = 1;
				
			}
			else
			{
				Rx_str[Rx_index]=RX_BUF[i];
				Rx_index++;
			}
		}
  }
  if(status&TX_DS) //tx_ds == 0x20
  {
   // LED_TGL;
    NRF24_WriteReg(STATUS, 0x20);
    NRF24L01_RX_Mode();
    tx_flag = 1;
  }
  else if(status&MAX_RT)
  {
    NRF24_WriteReg(STATUS, 0x10);
    NRF24_FlushTX();
    //Óõîäèì â ðåæèì ïðè¸ìíèêà
    NRF24L01_RX_Mode();
  }

}
//------------------------------------------------
void NRF24_ini(void)
{
	CE_RESET;
  DelayMicro(5000);
	NRF24_WriteReg(CONFIG, 0x0a); // Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
  DelayMicro(5000);
	NRF24_WriteReg(EN_AA, 0x01); // Enable Pipe0
	NRF24_WriteReg(EN_RXADDR, 0x01); // Enable Pipe0
	NRF24_WriteReg(SETUP_AW, 0x01); // Setup address width=3 bytes
	NRF24_WriteReg(SETUP_RETR, 0x5F); // // 1500us, 15 retrans
	NRF24_ToggleFeatures();
	NRF24_WriteReg(FEATURE, 0);
	NRF24_WriteReg(DYNPD, 0);
	NRF24_WriteReg(STATUS, 0x70); //Reset flags for IRQ
	NRF24_WriteReg(RF_CH, 76); // ÷àñòîòà 2476 MHz
	NRF24_WriteReg(RF_SETUP, 0x06); //TX_PWR:0dBm, Datarate:1Mbps
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS0, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS0, TX_ADR_WIDTH);
	NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH); //Number of bytes in RX payload in data pipe 0
 //ïîêà óõîäèì â ðåæèì ïðè¸ìíèêà
  NRF24L01_RX_Mode();
  LED_OFF;
}
//--------------------------------------------------
