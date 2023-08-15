#include "typedef.h"
#include "main.h"
#ifndef _MCP2515_H_
#define _MCP2515_H_


#define MCP_RXF0SIDH	0x00
#define MCP_RXF0SIDL	0x01
#define MCP_RXF0EID8	0x02
#define MCP_RXF0EID0	0x03
#define MCP_RXF1SIDH	0x04
#define MCP_RXF1SIDL	0x05
#define MCP_RXF1EID8	0x06
#define MCP_RXF1EID0	0x07
#define MCP_RXF2SIDH	0x08
#define MCP_RXF2SIDL	0x09
#define MCP_RXF2EID8	0x0A
#define MCP_RXF2EID0	0x0B
#define MCP_CANSTAT		0x0E
#define MCP_CANCTRL		0x0F
#define MCP_RXF3SIDH	0x10
#define MCP_RXF3SIDL	0x11
#define MCP_RXF3EID8	0x12
#define MCP_RXF3EID0	0x13
#define MCP_RXF4SIDH	0x14
#define MCP_RXF4SIDL	0x15
#define MCP_RXF4EID8	0x16
#define MCP_RXF4EID0	0x17
#define MCP_RXF5SIDH	0x18
#define MCP_RXF5SIDL	0x19
#define MCP_RXF5EID8	0x1A
#define MCP_RXF5EID0	0x1B
#define MCP_TEC			0x1C
#define MCP_REC			0x1D
#define MCP_RXM0SIDH	0x20
#define MCP_RXM0SIDL	0x21
#define MCP_RXM0EID8	0x22
#define MCP_RXM0EID0	0x23
#define MCP_RXM1SIDH	0x24
#define MCP_RXM1SIDL	0x25
#define MCP_RXM1EID8	0x26
#define MCP_RXM1EID0	0x27
#define MCP_CNF3		0x28
#define MCP_CNF2		0x29
#define MCP_CNF1		0x2A
#define MCP_CANINTE		0x2B
#define MCP_CANINTF		0x2C
#define MCP_EFLG		0x2D
#define MCP_TXB0CTRL	0x30
#define MCP_TXB1CTRL	0x40
#define MCP_TXB2CTRL	0x50

#define MCP_TXB0SIDH	0x31
#define MCP_TXB0SIDL	0x32
#define MCP_TXB0DLC     0x35
#define MCP_TXB1DLC     0x45
#define MCP_TXB2DLC     0x55
#define MCP_TXB0_DATA0  0x36
#define MCP_TXB0_DATA1  0x37
#define MCP_TXB0_DATA2  0x38
#define MCP_TXB0_DATA3  0x39
#define MCP_TXB0_DATA4  0x3A
#define MCP_TXB0_DATA5  0x3B
#define MCP_TXB0_DATA6  0x3C
#define MCP_TXB0_DATA7  0x3D


#define MCP_TXB1SIDH	0x41
#define MCP_TXB1SIDL	0x42
#define MCP_TXB2SIDH	0x51
#define MCP_TXB2SIDL	0x52

#define MCP_RXB0CTRL	0x60
#define MCP_RXB0SIDH	0x61
#define MCP_RXB1CTRL	0x70
#define MCP_RXB1SIDH	0x71

#define MCP_RXB0_BASE 	0x66
#define MCP_RXB1_BASE	0x76

// Define SPI Instruction Set
#define MCP_WRITE		0x02
#define MCP_READ		0x03
#define MCP_BITMOD		0x05

#define MCP_LOAD_TX0	0x40
#define MCP_LOAD_TX1	0x42
#define MCP_LOAD_TX2	0x44

#define MCP_RTS_TX0		0x81
#define MCP_RTS_TX1		0x82
#define MCP_RTS_TX2		0x84
#define MCP_RTS_ALL		0x87

#define MCP_READ_RX0	0x90
#define MCP_READ_RX0D0  0x92
#define MCP_READ_RX1	0x94

#define MCP_READ_STATUS	0xA0
#define MCP_RX_STATUS	0xB0
#define MCP_RESET		0xC0


// CANCTRL Register Values
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP	0xE0
#define MODE_MASK		0xE0
#define ABORT_TX        0x10
#define MODE_ONESHOT	0x08
#define CLKOUT_ENABLE	0x04
#define CLKOUT_DISABLE	0x00
#define CLKOUT_PS1		0x00
#define CLKOUT_PS2		0x01
#define CLKOUT_PS4		0x02
#define CLKOUT_PS8		0x03

/*
** Bits in the TXBnCTRL registers.
*/
#define MCP_TXB_TXBUFE_M    0x80
#define MCP_TXB_ABTF_M      0x40
#define MCP_TXB_MLOA_M      0x20
#define MCP_TXB_TXERR_M     0x10
#define MCP_TXB_TXREQ_M     0x08
#define MCP_TXB_TXIE_M      0x04
#define MCP_TXB_TXP10_M     0x03

/*
** Bits in the CANINTF registers.
*/
#define RX0IE			    0x01
#define RX1IE			    0x02
#define TX0IE			    0x04
#define TX1IE			    0x08
#define TX2IE			    0x10
#define ERRIE			    0x20
#define WAKIE			    0x40
#define MERRE			    0x80
#define MCP_INT_MASK		(RX0IE|RX1IE|TX0IE|ERRIE)

/*
** Bits in the EFLG registers.
*/
#define RX1OV			0x80
#define RX0OV			0x40
#define TXBO				0x20
#define TXEP				0x10
#define RXEP				0x08
#define TXWAR			0x04
#define RXWAR			0x02
#define EWARN			0x01

#define	MCP_8M_125K_CF1		0x01
#define	MCP_8M_125K_CF2		0x9A
#define	MCP_8M_125K_CF3		0x07

#define	MCP_12M_125K_CF1	0x02
#define	MCP_12M_125K_CF2	0xBC
#define	MCP_12M_125K_CF3	0x01

/* Function declaration for SPI1 - CAN Bus communication*/
static void SPI1_Tx(uint8_t data);
static uint8_t SPI1_Rx(void);
void MCP2515_SPI1_ReadReg(uint8_t addr, uint8_t* buff, uint8_t size);
void MCP2515_SPI1_WriteReg(uint8_t addr, uint8_t* buff, uint8_t size);
void MCP2515_SPI1_Reset(void);
void MCP2515_SPI1_RateConfig(void);
void MCP2515_SPI1_AddrReset(void);
void MCP2515_SPI1_RegModify(uint8_t addr, uint8_t mask, uint8_t data);
void MCP2515_SPI1_CanInit(void);
void MCP2515_SPI1_GetRxStatus(uint16_t* buffer);
void MCP2515_SPI1_ReadStatus(uint8_t* buffer);
void MCP2515_SPI1_LoadTxBuffer(uint8_t instruction, uint8_t data);
void MCP2515_SPI1_RequestToSend(uint8_t instruction);

/* Function declaration for SPI2 - EMS communication*/
static void SPI2_Tx(uint8_t data);
static uint8_t SPI2_Rx(void);
void MCP2515_SPI2_ReadReg(uint8_t addr, uint8_t* buff, uint8_t size);
void MCP2515_SPI2_WriteReg(uint8_t addr, uint8_t* buff, uint8_t size);
void MCP2515_SPI2_Reset(void);
void MCP2515_SPI2_RateConfig(void);
void MCP2515_SPI2_AddrReset(void);
void MCP2515_SPI2_RegModify(uint8_t addr, uint8_t mask, uint8_t data);
void MCP2515_SPI2_CanInit(void);
void MCP2515_SPI2_GetRxStatus(uint16_t* buffer);
void MCP2515_SPI2_ReadStatus(uint8_t* buffer);
void MCP2515_SPI2_LoadTxBuffer(uint8_t instruction, uint8_t data);
void MCP2515_SPI2_RequestToSend(uint8_t instruction);

#endif
