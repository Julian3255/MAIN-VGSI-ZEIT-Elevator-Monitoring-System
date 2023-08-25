#include "CAN_TxRx.h"
#include "main.h"
#include "mcp2515.h"

/* Define CAN ID for the slave EMS */
//#if(EMS_TYPE == SLAVE_EMS)
#define ID400_ADDR_SIDH          (0x60)
#define ID481_ADDR_SIDH          (0x62)
#define ID490_ADDR_SIDH          (0x64)
#define ID68B_ADDR_SIDH          (0x66)

#define SLAVE_ID_SIDL            SLAVE_1  
#define SLAVE_1                  (0x20)
#define SLAVE_2                  (0x40)
#define SLAVE_3                  (0x60)
//#endif

EMS_Buffer  Master;
EMS_Buffer  Slave_1;

extern uint8 rx_sidh;
extern uint8 rx_sidl;
extern uint16 rx_id;
extern uint8 rx_sidh2;
extern uint8 rx_sidl2;
extern uint16 rx_id2;
extern uint8 TxDataAdrr[8];
extern uint8 TxBufferData_SPI1[8];
extern uint8 TxBufferData_SPI2[8];
extern volatile uint8 rxLength;
extern volatile uint8 rxLength2;

void Read_CAN_ID(void) {
    MCP2515_SPI1_ReadReg(0x61, &rx_sidh, 1);
    MCP2515_SPI1_ReadReg(0x62, &rx_sidl, 1);
    rx_id = (rx_sidh << 3) | (rx_sidl >> 5);
}

void Read_CAN2_ID(void) {
    MCP2515_SPI2_ReadReg(0x61, &rx_sidh2, 1);
    MCP2515_SPI2_ReadReg(0x62, &rx_sidl2, 1);
    rx_id2 = (rx_sidh2 << 3) | (rx_sidl2 >> 5);
}

void Read_TXdata(int channel) {
    if(channel == SPI_CHANNEL_1) {
        for(int i = 0; i < sizeof(TxDataAdrr)/sizeof(uint8); i++) {
            MCP2515_SPI1_ReadReg(TxDataAdrr[i], &TxBufferData_SPI1[i], 1);
        }
        // This ensure when sending data bytes with different DLC length, the previous data in the buffers is erased
        for (int i = rxLength; i < 8; i++) {
            TxBufferData_SPI1[i] = 0x00;
        }
    }
    else {
			for(int i = 0; i < sizeof(TxDataAdrr)/sizeof(uint8); i++) {
            MCP2515_SPI2_ReadReg(TxDataAdrr[i], &TxBufferData_SPI2[i], 1);
        }
        // This ensure when sending data bytes with different DLC length, the previous data in the buffers is erased
        for (int i = rxLength; i < 8; i++) {
            TxBufferData_SPI2[i] = 0x00;
        }
    }
}

extern uint8 tx_sidl;
extern uint8 tx_sidh;

extern volatile int CAN_rx;

extern EMS_data Master_EMS; 
extern EMS_data Slave_EMS_1;
extern volatile uint16 send_rx;
extern volatile int ID_481_flag;
extern volatile int rx_flag;

void Read_RXdata(uint16* rx_id, uint8* base_adr) {
	 MCP2515_SPI1_ReadReg(0x65, &rxLength, 1);
    switch(*rx_id) 
    {
        /* 0x400 - Indicate current floor of elevator */
        case 0x400:
           // MCP2515_SPI1_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              Master.ID_400_buffer[i] = 0x00;
              MCP2515_SPI1_ReadReg(*base_adr+i, &Master.ID_400_buffer[i], 1);
            }
			if(Master.ID_400_buffer[0] == 0x40) {
				Master_EMS.curren_floor = Master.ID_400_buffer[1];
            }
        break;

        /* 0x481 - Indicate door status (opened/closed) of the elevator */
        case 0x481:
         //   MCP2515_SPI1_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
                MCP2515_SPI1_ReadReg(*base_adr+i, &Master.ID_481_buffer[i], 1);
            }
            for(int j = rxLength; j < 8; j++) {
                Master.ID_481_buffer[j] = 0x00;
            }
			if(Master.ID_481_buffer[1] == 0x02) {
				if(Master.ID_481_buffer[5] == 0x00) {
					Master_EMS.door_status = DOOR_OPENED;
				}
				else {
					Master_EMS.door_status = DOOR_CLOSED;
				}
			}
        break;

        /* 0x490 - Indicate the chosen floor from the panel or the open/close door button status (ON/OFF)*/
        case 0x490:
           // MCP2515_SPI1_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              MCP2515_SPI1_ReadReg(*base_adr+i, &Master.ID_490_buffer[i], 1);
            }
            for(int j = rxLength; j < 8; j++) {
                Master.ID_490_buffer[j] = 0x00;
            }
            switch (Master.ID_490_buffer[0])
                {
                case 0x05:
                    Master_EMS.chosen_floor = Master.ID_490_buffer[1];
                    break;
                case 0x0E:
                    if(Master.ID_490_buffer[1] == 0x09) {
                        if(Master.ID_490_buffer[5] == 0x00) {
                            Master_EMS.open_door_button_stat = BUTTON_OFF;
                        }
                        else {
                            Master_EMS.open_door_button_stat = BUTTON_ON;
                        }
                    }
                    else if (Master.ID_490_buffer[1] == 0x0A){
                        if(Master.ID_490_buffer[5] == 0x00) {
                            Master_EMS.close_door_button_stat = BUTTON_OFF;
                        }
                        else {
                            Master_EMS.close_door_button_stat = BUTTON_ON;
                        }
                    }
                    break;
            }        
        break;

        /* 0x68B - Indicate the time of the elevator */
        case 0x68B:
           // MCP2515_SPI1_ReadReg(0x65, &rxLength, 1);
            for (int i = 0; i < rxLength; i++) {
              Master.ID_68B_buffer[i] = 0x00;
              MCP2515_SPI1_ReadReg(*base_adr+i, &Master.ID_68B_buffer[i], 1);
            }
            getEMStime_Master();
        break;

        default:
        break;
    }  
} 

void Read_Slave1_RXdata(uint16* rx_id, uint8* base_adr) {
	MCP2515_SPI2_ReadReg(0x65, &rxLength2, 1);

    switch(*rx_id)
    {
        /* 0x400 - Indicate current floor of elevator */
        case 0x301:
            for (int i = 0; i < rxLength2; i++) {
              MCP2515_SPI2_ReadReg(*base_adr+i, &Slave_1.ID_400_buffer[i], 1);
            }
            for(int j = rxLength2; j < 8; j++) {
                Slave_1.ID_400_buffer[j] = 0x00;
            }
            if(Slave_1.ID_400_buffer[0] == 0x40) {
                    Slave_EMS_1.curren_floor = Slave_1.ID_400_buffer[1];
            }
            CAN_rx++;
        break;

        case 0x311:
            for (int i = 0; i < rxLength2; i++) {
                MCP2515_SPI2_ReadReg(*base_adr+i, &Slave_1.ID_481_buffer[i], 1);
            }
            for(int j = rxLength2; j < 8; j++) {
                Slave_1.ID_481_buffer[j] = 0x00;
            }
            if(Slave_1.ID_481_buffer[1] == 0x02) {
                if(Slave_1.ID_481_buffer[5] == 0x00) {
                    Slave_EMS_1.door_status = DOOR_OPENED;
                }
                else {
                    Slave_EMS_1.door_status = DOOR_CLOSED;
                }
            }
            CAN_rx++;
        break;  
        case 0x321:
            for (int i = 0; i < rxLength2; i++) {
              MCP2515_SPI2_ReadReg(*base_adr+i, &Slave_1.ID_490_buffer[i], 1);
            }
            for(int j = rxLength2; j < 8; j++) {
                Slave_1.ID_490_buffer[j] = 0x00;
            }
            switch (Slave_1.ID_490_buffer[0])
            {
                case 0x05:
                    Slave_EMS_1.chosen_floor = Slave_1.ID_490_buffer[1];
                    break;
                case 0x0E:
                    if(Slave_1.ID_490_buffer[1] == 0x09) {
                        if(Slave_1.ID_490_buffer[5] == 0x00) {
                            Slave_EMS_1.open_door_button_stat = BUTTON_OFF;
                        }
                        else {
                            Slave_EMS_1.open_door_button_stat = BUTTON_ON;
                        }
                    }
                    else if (Slave_1.ID_490_buffer[1] == 0x0A){
                        if(Slave_1.ID_490_buffer[5] == 0x00) {
                            Slave_EMS_1.close_door_button_stat = BUTTON_OFF;
                        }
                        else {
                            Slave_EMS_1.close_door_button_stat = BUTTON_ON;
                        }
                    }
                    CAN_rx++;
            }
                break;

        case 0x331:
            for (int i = 0; i < rxLength2; i++) {
              MCP2515_SPI2_ReadReg(*base_adr+i, &Slave_1.ID_68B_buffer[i], 1);
            }
            getEMStime_Slave1();
            CAN_rx++;
        break;
        
        default:
            break;       
    }
}

void getEMStime_Master(void) {
    Master_EMS.EMS_time.second = Master.ID_68B_buffer[0];
    Master_EMS.EMS_time.minute = Master.ID_68B_buffer[1];
    Master_EMS.EMS_time.hour = Master.ID_68B_buffer[2];
    Master_EMS.EMS_time.day = Master.ID_68B_buffer[3];
    Master_EMS.EMS_time.weekday = Master.ID_68B_buffer[4] & (0x07);  // Weekday is the first 3 bits of 5th byte
    Master_EMS.EMS_time.year = Master.ID_68B_buffer[5];
}

void getEMStime_Slave1(void) {
    Slave_EMS_1.EMS_time.second = Slave_1.ID_68B_buffer[0];
    Slave_EMS_1.EMS_time.minute = Slave_1.ID_68B_buffer[1];
    Slave_EMS_1.EMS_time.hour = Slave_1.ID_68B_buffer[2];
    Slave_EMS_1.EMS_time.day = Slave_1.ID_68B_buffer[3];
    Slave_EMS_1.EMS_time.weekday = (Slave_1.ID_68B_buffer[4] & (0x07));  // Weekday is the first 3 bits of 5th byte
    Slave_EMS_1.EMS_time.year = Slave_1.ID_68B_buffer[5];
}

void Send_RXdata(uint16* rx_id) {
    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, MCP_TXB_ABTF_M | MCP_TXB_MLOA_M \
    | MCP_TXB_TXERR_M | MCP_TXB_TXREQ_M, 0x00);
    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, 0x03, 0xFF);
    switch(*rx_id) {
        case 0x400:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID400_ADDR_SIDH, 1);  // 0x200 = 0010 0000 0000 -> 0100 0000    = 0x40
    	    MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   // 000 = 0x00
    	    MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1);

            for(int i = 0; i < rxLength; i++) {
              MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_400_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
        break;

        case 0x481:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID481_ADDR_SIDH, 1);  // 0x200 = 0010 0000 0000 -> 0100 0000    = 0x40
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   // 000 = 0x00
            MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1); // change to corespoind byte length
            for(int i = 0; i < rxLength; i++) {
                MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_481_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
        break;
        
        case 0x490:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID490_ADDR_SIDH, 1);  // 0x200 = 0010 0000 0000 -> 0100 0000    = 0x40
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   // 000 = 0x00
            MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1); // change to corespoind byte length
            for(int i = 0; i < rxLength; i++) {
                MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_490_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
        break;
        
        case 0x68B:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID68B_ADDR_SIDH, 1);  // 0x200 = 0010 0000 0000 -> 0100 0000    = 0x40
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   // 000 = 0x00
            MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1); // change to corespoind byte length
            for(int i = 0; i < rxLength; i++) {
                MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_68B_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
         break;
    }

}

uint8 close_cmd_data[2][8] = 
{
    {CLOSE_CMD_BYTE0,CLOSE_CMD_BYTE1,CLOSE_CMD_BYTE2, CLOSE_CMD_BYTE3, CLOSE_CMD_BYTE4, CLOSE_CMD_BYTE5, CLOSE_CMD_BYTE6,OPEN_CMD_BYTE7},
    {MCP_TXB0_DATA0, MCP_TXB0_DATA1, MCP_TXB0_DATA2, MCP_TXB0_DATA3, MCP_TXB0_DATA4, MCP_TXB0_DATA5, MCP_TXB0_DATA6, MCP_TXB0_DATA7}
};


uint8 open_cmd_data[2][8] = 
{
    {OPEN_CMD_BYTE0,OPEN_CMD_BYTE1,OPEN_CMD_BYTE2, OPEN_CMD_BYTE3, OPEN_CMD_BYTE4, OPEN_CMD_BYTE5, OPEN_CMD_BYTE6,OPEN_CMD_BYTE7},
    {MCP_TXB0_DATA0, MCP_TXB0_DATA1, MCP_TXB0_DATA2, MCP_TXB0_DATA3, MCP_TXB0_DATA4, MCP_TXB0_DATA5, MCP_TXB0_DATA6, MCP_TXB0_DATA7}
};


void Send_CloseDoor(void) {
    for(int i = 0; i < CLOSE_CMD_DLC; i++) {
        MCP2515_SPI2_WriteReg(close_cmd_data[1][i], close_cmd_data[0][i], ONE_BYTE);
    }
}

void Send_OpenDoor(void) {
    for(int i = 0; i < OPEN_CMD_DLC; i++) {
        MCP2515_SPI2_WriteReg(open_cmd_data[1][i], open_cmd_data[0][i], ONE_BYTE);
    }
}

