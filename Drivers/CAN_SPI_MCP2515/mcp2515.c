#include "mcp2515.h"
#include "main.h"
#include "stm32wbxx_hal.h"

/* SPI Tx wrapper function  */
static void SPI_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Tx wrapper function */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);    
}

/* SPI Rx wrapper function */
static uint8_t SPI_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx wrapper function */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}

void MCP2515_WriteByte(uint8_t address, uint8_t data)
{    
  MCP_CS_OFF();  
  
  SPI_Tx(MCP_WRITE);
  SPI_Tx(address);
  SPI_Tx(data);  
    
  MCP_CS_ON();
}
void MCP2515_Reset(void)
{    
  MCP_CS_OFF();
      
  SPI_Tx(MCP_RESET);
      
  MCP_CS_ON();
}

/* read single byte */
uint8_t MCP2515_ReadByte (uint8_t address)
{
  uint8_t retVal;
  
  MCP_CS_OFF();
  SPI_Tx(MCP_READ);
  SPI_Tx(address);
  retVal = SPI_Rx();
      
  MCP_CS_ON();
  
  return retVal;
}

void MCP2515_ReadReg(uint8_t addr, uint8_t* buff, uint8_t size) {
	MCP_CS_OFF();
	SPI_Tx(MCP_READ);
	SPI_Tx(addr);
	while(size--) {
		*buff ++= SPI_Rx();
	}
	MCP_CS_ON();
}

void MCP2515_WriteReg(uint8_t addr, uint8_t* buff, uint8_t size) {
	MCP_CS_OFF();
	SPI_Tx(MCP_WRITE);
	SPI_Tx(addr);
	while(size--) {
		SPI_Tx(buff++);
	}
	MCP_CS_ON();
}

void MCP2515_RegModify(uint8_t addr, uint8_t mask, uint8_t data) {
	MCP_CS_OFF();
	SPI_Tx(MCP_BITMOD);
	SPI_Tx(addr);
	SPI_Tx(mask);
	SPI_Tx(data);
	MCP_CS_ON();
}

void MCP2515_RateConfig(void) {
	MCP2515_WriteReg(MCP_CNF1, MCP_8M_125K_CF1, 1);
	MCP2515_WriteReg(MCP_CNF2, MCP_8M_125K_CF2, 1);
	MCP2515_WriteReg(MCP_CNF3, MCP_8M_125K_CF3, 1);
	HAL_Delay(5);
}

const DWORD mcp2515_bufffers_init[][2] =
{
	{MCP_RXM0SIDH, 0x00},
	{MCP_RXM1SIDH, 0x00},
	{MCP_RXF0SIDH, 0x00},
	{MCP_RXF1SIDH, 0x00},
	{MCP_RXF2SIDH, 0x00},
	{MCP_RXF3SIDH, 0x00},
	{MCP_RXF4SIDH, 0x00},
	{MCP_RXF5SIDH, 0x00},
};

void MCP2515_AddrReset(void) {
	for (int i = 0; i < sizeof(mcp2515_bufffers_init)/(2*sizeof(DWORD)); i++) {
		MCP_CS_OFF();
		SPI_Tx(MCP_WRITE);
		SPI_Tx(mcp2515_bufffers_init[i][0]);
		SPI_Tx(mcp2515_bufffers_init[i][1]);
		SPI_Tx(mcp2515_bufffers_init[i][1] >> 8);
		SPI_Tx(mcp2515_bufffers_init[i][1] >> 16);
		SPI_Tx(mcp2515_bufffers_init[i][1] >> 24);
		MCP_CS_ON();
	}
}
void MCP2515_CanInit(void) {
	MCP2515_Reset();
	MCP2515_RegModify(MCP_CANCTRL, MODE_MASK, MODE_CONFIG);
	MCP2515_AddrReset();
	MCP2515_RateConfig();
	MCP2515_RegModify(MCP_CANINTE, 0xFF, 0xFF);
	MCP2515_RegModify(MCP_CANCTRL, MODE_MASK, MODE_NORMAL);
	MCP2515_RegModify(MCP_RXB0CTRL,0x04,0x04);

}

/* read RX STATUS register */
void MCP2515_GetRxStatus(uint16_t* buffer)
{
  MCP_CS_OFF();
  SPI_Tx(MCP_RX_STATUS);
  *buffer = HAL_SPI_Receive(SPI_CAN, buffer, 2, SPI_TIMEOUT);
  MCP_CS_ON();
}

void MCP2515_ReadStatus(uint8_t* buffer)
{
  MCP_CS_OFF();
  SPI_Tx(MCP_READ_STATUS);
  *buffer = SPI_Rx(); 
  MCP_CS_ON();
}

void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  MCP_CS_OFF();
  SPI_Tx(instruction);
  SPI_Tx(data);  
  MCP_CS_ON();
}

/* request to send */
void MCP2515_RequestToSend(uint8_t instruction)
{
  MCP_CS_OFF();
  SPI_Tx(MCP_WRITE);
  SPI_Tx(instruction); 
  MCP_CS_ON();
}
