#include "mcp2515.h"
#include "main.h"
#include "stm32wbxx_hal.h"

/* This array stores the init values for the buffers of SPI1 and SPI2*/
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

/* SPI Tx wrapper function  */
static void SPI1_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Rx wrapper function */
static uint8_t SPI1_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

void MCP2515_SPI1_Reset(void)
{    
  SPI1_CS_OFF();
      
  SPI1_Tx(MCP_RESET);
      
  SPI1_CS_ON();
}

void MCP2515_SPI1_ReadReg(uint8_t addr, uint8_t* buff, uint8_t size) {
	SPI1_CS_OFF();
	SPI1_Tx(MCP_READ);
	SPI1_Tx(addr);
	while(size--) {
		*buff ++= SPI1_Rx();
	}
	SPI1_CS_ON();
}

void MCP2515_SPI1_WriteReg(uint8_t addr, uint8_t* buff, uint8_t size) {
	SPI1_CS_OFF();
	SPI1_Tx(MCP_WRITE);
	SPI1_Tx(addr);
	while(size--) {
		SPI1_Tx(buff++);
	}
	SPI1_CS_ON();
}

void MCP2515_SPI1_RegModify(uint8_t addr, uint8_t mask, uint8_t data) {
	SPI1_CS_OFF();
	SPI1_Tx(MCP_BITMOD);
	SPI1_Tx(addr);
	SPI1_Tx(mask);
	SPI1_Tx(data);
	SPI1_CS_ON();
}

void MCP2515_SPI1_RateConfig(void) {
	MCP2515_SPI1_WriteReg(MCP_CNF1, MCP_8M_125K_CF1, 1);
	MCP2515_SPI1_WriteReg(MCP_CNF2, MCP_8M_125K_CF2, 1);
	MCP2515_SPI1_WriteReg(MCP_CNF3, MCP_8M_125K_CF3, 1);
	HAL_Delay(5);
}

void MCP2515_SPI1_AddrReset(void) {
	for (int i = 0; i < sizeof(mcp2515_bufffers_init)/(2*sizeof(DWORD)); i++) {
		SPI1_CS_OFF();
		SPI1_Tx(MCP_WRITE);
		SPI1_Tx(mcp2515_bufffers_init[i][0]);
		SPI1_Tx(mcp2515_bufffers_init[i][1]);
		SPI1_Tx(mcp2515_bufffers_init[i][1] >> 8);
		SPI1_Tx(mcp2515_bufffers_init[i][1] >> 16);
		SPI1_Tx(mcp2515_bufffers_init[i][1] >> 24);
		SPI1_CS_ON();
	}
}
void MCP2515_SPI1_CanInit(void) {
	MCP2515_SPI1_Reset();
	MCP2515_SPI1_RegModify(MCP_CANCTRL, MODE_MASK, MODE_CONFIG);
	MCP2515_SPI1_AddrReset();
	MCP2515_SPI1_RateConfig();
	MCP2515_SPI1_RegModify(MCP_CANINTE, 0xFF, 0xFF);
	MCP2515_SPI1_RegModify(MCP_CANCTRL, MODE_MASK, MODE_NORMAL);
	MCP2515_SPI1_RegModify(MCP_RXB0CTRL,0xFF,0x00);

}

/* read RX STATUS register */
void MCP2515_SPI1_GetRxStatus(uint16_t* buffer)
{
  SPI1_CS_OFF();
  SPI1_Tx(MCP_RX_STATUS);
  *buffer = HAL_SPI_Receive(SPI_CAN, buffer, 2, SPI_TIMEOUT);
  SPI1_CS_ON();
}

void MCP2515_SPI1_ReadStatus(uint8_t* buffer)
{
  SPI1_CS_OFF();
  SPI1_Tx(MCP_READ_STATUS);
  *buffer = SPI1_Rx(); 
  SPI1_CS_ON();
}

void MCP2515_SPI1_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  SPI1_CS_OFF();
  SPI1_Tx(instruction);
  SPI1_Tx(data);  
  SPI1_CS_ON();
}

/* request to send */
void MCP2515_SPI1_RequestToSend(uint8_t instruction)
{
  SPI1_CS_OFF();
  SPI1_Tx(MCP_WRITE);
  SPI1_Tx(instruction); 
  SPI1_CS_ON();
}


/* SPI Tx wrapper function  */
static void SPI2_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_EMS, &data, 1, SPI_TIMEOUT);    
}

/* SPI Rx wrapper function */
static uint8_t SPI2_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_EMS, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

void MCP2515_SPI2_Reset(void)
{    
  SPI2_CS_OFF();
      
  SPI2_Tx(MCP_RESET);

  SPI2_CS_ON();
}

void MCP2515_SPI2_ReadReg(uint8_t addr, uint8_t* buff, uint8_t size) {
	SPI2_CS_OFF();
	SPI2_Tx(MCP_READ);
	SPI2_Tx(addr);
	while(size--) {
		*buff ++= SPI2_Rx();
	}
	SPI2_CS_ON();
}

void MCP2515_SPI2_WriteReg(uint8_t addr, uint8_t* buff, uint8_t size) {
	SPI2_CS_OFF();
	SPI2_Tx(MCP_WRITE);
	SPI2_Tx(addr);
	while(size--) {
		SPI2_Tx(buff++);
	}
	SPI2_CS_ON();
}

void MCP2515_SPI2_RegModify(uint8_t addr, uint8_t mask, uint8_t data) {
	SPI2_CS_OFF();
	SPI2_Tx(MCP_BITMOD);
	SPI2_Tx(addr);
	SPI2_Tx(mask);
	SPI2_Tx(data);
	SPI2_CS_ON();
}

void MCP2515_SPI2_RateConfig(void) {
	MCP2515_SPI2_WriteReg(MCP_CNF1, MCP_8M_125K_CF1, 1);
	MCP2515_SPI2_WriteReg(MCP_CNF2, MCP_8M_125K_CF2, 1);
	MCP2515_SPI2_WriteReg(MCP_CNF3, MCP_8M_125K_CF3, 1);
	HAL_Delay(5);
}

void MCP2515_SPI2_AddrReset(void) {
	for (int i = 0; i < sizeof(mcp2515_bufffers_init)/(2*sizeof(DWORD)); i++) {
		SPI2_CS_OFF();
		SPI2_Tx(MCP_WRITE);
		SPI2_Tx(mcp2515_bufffers_init[i][0]);
		SPI2_Tx(mcp2515_bufffers_init[i][1]);
		SPI2_Tx(mcp2515_bufffers_init[i][1] >> 8);
		SPI2_Tx(mcp2515_bufffers_init[i][1] >> 16);
		SPI2_Tx(mcp2515_bufffers_init[i][1] >> 24);
		SPI2_CS_ON();
	}
}

void MCP2515_SPI2_CanInit(void) {
	MCP2515_SPI2_Reset();
	MCP2515_SPI2_RegModify(MCP_CANCTRL, MODE_MASK, MODE_CONFIG);
	MCP2515_SPI2_AddrReset();
	MCP2515_SPI2_RateConfig();
	MCP2515_SPI2_RegModify(MCP_CANINTE, 0xFF, 0xFF);
	MCP2515_SPI2_RegModify(MCP_CANCTRL, MODE_MASK, MODE_NORMAL);
	MCP2515_SPI1_RegModify(MCP_RXB0CTRL,0xFF,0x00);

}

/* read RX STATUS register */
void MCP2515_SPI2_GetRxStatus(uint16_t* buffer)
{
  SPI2_CS_OFF();
  SPI2_Tx(MCP_RX_STATUS);
  *buffer = HAL_SPI_Receive(SPI_EMS, buffer, 2, SPI_TIMEOUT);
  SPI2_CS_ON();
}

void MCP2515_SPI2_ReadStatus(uint8_t* buffer)
{
  SPI2_CS_OFF();
  SPI2_Tx(MCP_READ_STATUS);
  *buffer = SPI2_Rx(); 
  SPI2_CS_ON();
}

void MCP2515_SPI2_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  SPI2_CS_OFF();
  SPI2_Tx(instruction);
  SPI2_Tx(data);  
  SPI2_CS_ON();
}

/* request to send */
void MCP2515_SPI2_RequestToSend(uint8_t instruction)
{
  SPI2_CS_OFF();
  SPI2_Tx(MCP_WRITE);
  SPI2_Tx(instruction); 
  SPI2_CS_ON();
}