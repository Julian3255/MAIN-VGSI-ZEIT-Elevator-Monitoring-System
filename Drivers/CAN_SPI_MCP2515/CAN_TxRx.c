#include "CAN_TxRx.h"
#include "main.h"
#include "mcp2515.h"

extern uint8 TxDataAdrr[8];
extern uint8 TxBufferData[8];

void Read_TXdata(int dlc_length) {
    for(int i = 0; i < sizeof(TxDataAdrr)/sizeof(uint8); i++) {
        MCP2515_ReadReg(TxDataAdrr[i], &TxBufferData[i], 1);
    }
    for (int i = dlc_length; i < 8; i++) {
    	TxBufferData[i] = 0x00;
    }
}

extern uint8 tx_sidl;
extern uint8 tx_sidh;


extern uint8 ID_200[8];
extern uint8 ID_400[8];
extern uint8 ID_490[8];
extern uint8 ID_501[8];
//extern uint8 ID_48C[8];
extern uint8 ID_68B[8];
extern uint8 rxLength;

void Read_RXdata(uint16* rx_id, uint8* base_adr) {
    
    switch(*rx_id) 
    {
        case 0x200:
            MCP2515_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              ID_200[i] = 0x00;
              MCP2515_ReadReg(*base_adr+i, &ID_200[i], 1);
			}
            
        break;
      
        case 0x400:
            MCP2515_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              ID_400[i] = 0x00;
              MCP2515_ReadReg(*base_adr+i, &ID_400[i], 1);
			}

        break;

        case 0x490:
            MCP2515_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              ID_490[i] = 0x00;
              MCP2515_ReadReg(*base_adr+i, &ID_490[i], 1);
			}

        break;

        case 0x501:
            MCP2515_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
               ID_501[i] = 0x00;
               MCP2515_ReadReg(*base_adr+i, &ID_501[i], 1);
       		}

        break;

//        case 0x48C:
//            MCP2515_ReadReg(0x65, &rxLength, 1);
//            for (int i = 0; i < rxLength; i++) {
//              ID_48C[i] = 0x00;
//              MCP2515_ReadReg(*base_adr+i, &ID_48C[i], 1);
//			}
//
//        break;

        case 0x68B:
            MCP2515_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              ID_68B[i] = 0x00;
              MCP2515_ReadReg(*base_adr+i, &ID_68B[i], 1);
			}

        break;

    }
}
