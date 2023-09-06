#include "CAN_TxRx.h"
#include "main.h"
#include "mcp2515.h"

/* Define CAN ID for the slave EMS since each EMS has unique address*/
// Define the high 8-bit address 
#define ID400_ADDR_SIDH          (0x60)
#define ID481_ADDR_SIDH          (0x62)
#define ID490_ADDR_SIDH          (0x64)
#define ID68B_ADDR_SIDH          (0x66)

// Define the lowest 4-bit address 
#define SLAVE_ID_SIDL            SLAVE_1   // Change this to change the slave ID
#define SLAVE_1                  (0x20)
#define SLAVE_2                  (0x40)
#define SLAVE_3                  (0x60)

// Create the buffers to store the data for the Master and Slave 1 EMS boards
EMS_Buffer  Master;
EMS_Buffer  Slave_1;

// Take the variables declared from main.c 
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

// Read the CAN ID from the CAN bus for SPI1 (apply for Master and Slave)
void Read_CAN_ID(void) {
    MCP2515_SPI1_ReadReg(0x61, &rx_sidh, 1);
    MCP2515_SPI1_ReadReg(0x62, &rx_sidl, 1);
    rx_id = (rx_sidh << 3) | (rx_sidl >> 5);
}

// Read the CAN ID from the CAN bus for SPI2 (apply for Slave only)
void Read_CAN2_ID(void) {
    MCP2515_SPI2_ReadReg(0x61, &rx_sidh2, 1);
    MCP2515_SPI2_ReadReg(0x62, &rx_sidl2, 1);
    rx_id2 = (rx_sidh2 << 3) | (rx_sidl2 >> 5);
}

// Read the data that was just sent to the CAN bus through SPI1 and SPI2 (apply for Master and Slave)
void Read_TXdata(int channel) {
    // Extract the data sent through SPI1
    if(channel == SPI_CHANNEL_1) {
        // Loop through the address of each data buffer from MCP2515
        for(int i = 0; i < sizeof(TxDataAdrr)/sizeof(uint8); i++) {
            MCP2515_SPI1_ReadReg(TxDataAdrr[i], &TxBufferData_SPI1[i], 1);
        }
        // This ensure when sending data bytes with different DLC length, the previous data in the buffers is erased
        for (int i = rxLength; i < 8; i++) {
            TxBufferData_SPI1[i] = 0x00;
        }
    }
    // Extract the data sent through SPI2
    else {
        // Loop through the address of each data buffer from MCP2515
        for(int i = 0; i < sizeof(TxDataAdrr)/sizeof(uint8); i++) {
        MCP2515_SPI2_ReadReg(TxDataAdrr[i], &TxBufferData_SPI2[i], 1);
        }
        // This ensure when sending data bytes with different DLC length, the previous data in the buffers is erased
        for (int i = rxLength; i < 8; i++) {
            TxBufferData_SPI2[i] = 0x00;
        }
    }
}

// Take the variables declared from main.c 
extern uint8 tx_sidl;
extern uint8 tx_sidh;
extern volatile int CAN_rx;
extern EMS_data Master_EMS; 
extern EMS_data Slave_EMS_1;
extern volatile uint16 send_rx;

// Read the received data from the CAN bus through SPI1 (apply for Master and Slave)
void Read_RXdata(uint16* rx_id, uint8* base_adr) {
	 MCP2515_SPI1_ReadReg(0x65, &rxLength, 1);  // Extract the DLC length of the message frame
    switch(*rx_id) 
    {
        /* 0x400 - Indicate current floor of elevator */
        case 0x400:
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
            for (int i = 0; i < rxLength; i++) {
                MCP2515_SPI1_ReadReg(*base_adr+i, &Master.ID_481_buffer[i], 1);
            }
            for(int j = rxLength; j < 8; j++) {
                Master.ID_481_buffer[j] = 0x00;
            }
			if(Master.ID_481_buffer[0] == 0x0E) {
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

// Read the CAN data from Slave EMS using SPI2 (apply for Master only)
void Read_Slave1_RXdata(uint16* rx_id, uint8* base_adr) {
	MCP2515_SPI2_ReadReg(0x65, &rxLength2, 1);  // Extract the DLC length of the messsage frame received from the Slaves 
    switch(*rx_id)
    {
        /* If original data from slave is x400 - Indicate current floor of elevator 
           -> Send the same data to the Master under CAN ID 0x30z 
           z = 1: Slave ID 1
           z = 2: Slave ID 2
           z = 3: Slave ID 3
        */
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
        break;

        /* If original data from slave is x481 - Indicate door status (closed or opeend)
           -> Send the same data to the Master under CAN ID 0x31z 
           z = 1: Slave ID 1
           z = 2: Slave ID 2
           z = 3: Slave ID 3
        */
        case 0x311:
            for (int i = 0; i < rxLength2; i++) {
                MCP2515_SPI2_ReadReg(*base_adr+i, &Slave_1.ID_481_buffer[i], 1);
            }
            for(int j = rxLength2; j < 8; j++) {
                Slave_1.ID_481_buffer[j] = 0x00;
            }
            if(Slave_1.ID_481_buffer[0] == 0x0E) {
                if(Slave_1.ID_481_buffer[5] == 0x00) {
                    Slave_EMS_1.door_status = DOOR_OPENED;
                }
                else {
                    Slave_EMS_1.door_status = DOOR_CLOSED;
                }
            }
        break;  

        /* If original data from slave is x490 - Indicate open and close door status (pressed or unpressed) 
           -> Send the same data to the Master under CAN ID 0x32z 
           z = 1: Slave ID 1
           z = 2: Slave ID 2
           z = 3: Slave ID 3
        */
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
            }
                break;

        /* If original data from slave is 0x68B - Indicate the time of elevator
           -> Send the same data to the Master under CAN ID 0x33z 
           z = 1: Slave ID 1
           z = 2: Slave ID 2
           z = 3: Slave ID 3
        */
        case 0x331:
            for (int i = 0; i < rxLength2; i++) {
              MCP2515_SPI2_ReadReg(*base_adr+i, &Slave_1.ID_68B_buffer[i], 1);
            }
            getEMStime_Slave1(); // Extract the time of the EMS slave - 1
        break;
        
        default:
            break;       
    }
}

// Extract the time of the EMS Master
void getEMStime_Master(void) {
    Master_EMS.EMS_time.second = Master.ID_68B_buffer[0];
    Master_EMS.EMS_time.minute = Master.ID_68B_buffer[1];
    Master_EMS.EMS_time.hour = Master.ID_68B_buffer[2];
    Master_EMS.EMS_time.day = Master.ID_68B_buffer[3];
    Master_EMS.EMS_time.weekday = Master.ID_68B_buffer[4] & (0x07);  // Weekday is the first 3 bits of 5th byte
    Master_EMS.EMS_time.year = Master.ID_68B_buffer[5];
}

// Extract the time of the EMS slave - 1
void getEMStime_Slave1(void) {
    Slave_EMS_1.EMS_time.second = Slave_1.ID_68B_buffer[0];
    Slave_EMS_1.EMS_time.minute = Slave_1.ID_68B_buffer[1];
    Slave_EMS_1.EMS_time.hour = Slave_1.ID_68B_buffer[2];
    Slave_EMS_1.EMS_time.day = Slave_1.ID_68B_buffer[3];
    Slave_EMS_1.EMS_time.weekday = (Slave_1.ID_68B_buffer[4] & (0x07));  // Weekday is the first 3 bits of 5th byte
    Slave_EMS_1.EMS_time.year = Slave_1.ID_68B_buffer[5];
}

// Send back the data received from CAN bus (SPI1) through SPI2 (apply for Slave only) 
void Send_RXdata(uint16* rx_id) {
    // Setup the registers
    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, MCP_TXB_ABTF_M | MCP_TXB_MLOA_M \
    | MCP_TXB_TXERR_M | MCP_TXB_TXREQ_M, 0x00);
    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, 0x03, 0xFF);

    // Check the received ID from SPI1 and send back to SPI2 under 
    switch(*rx_id) {
        /* 
            If the received CAN ID is 0x400 -> Send back under ID 0x30z 
            z = 1: Slave 1 ID
            z = 2: Slave 2 ID
            z = 3: Slave 3 ID
        */
        case 0x400:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID400_ADDR_SIDH, 1);  
    	    MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   
    	    MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1);

            for(int i = 0; i < rxLength; i++) {
              MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_400_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
            break;

        /* 
            If the received CAN ID is 0x481 -> Send back under ID 0x31z 
            z = 1: Slave 1 ID
            z = 2: Slave 2 ID
            z = 3: Slave 3 ID
        */
        case 0x481:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID481_ADDR_SIDH, 1);  
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   
            MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1); // change to corespoind byte length
            for(int i = 0; i < rxLength; i++) {
                MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_481_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
            break;

        /* 
            If the received CAN ID is 0x490 -> Send back under ID 0x32z 
            z = 1: Slave 1 ID
            z = 2: Slave 2 ID
            z = 3: Slave 3 ID
        */
        case 0x490:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID490_ADDR_SIDH, 1);  
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   
            MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1); // change to corespoind byte length
            for(int i = 0; i < rxLength; i++) {
                MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_490_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
            break;

        /* 
            If the received CAN ID is 0x68B -> Send back under ID 0x33z 
            z = 1: Slave 1 ID
            z = 2: Slave 2 ID
            z = 3: Slave 3 ID
        */
        case 0x68B:
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, ID68B_ADDR_SIDH, 1);  
            MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SLAVE_ID_SIDL, 1);   
            MCP2515_SPI2_WriteReg(MCP_TXB0DLC, rxLength, 1); // change to corespoind byte length
            for(int i = 0; i < rxLength; i++) {
                MCP2515_SPI2_WriteReg(TxDataAdrr[i], Master.ID_68B_buffer[i], ONE_BYTE);
            }
            MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
            break;
        
        default:
            break;
    }

}

void SendData_SPI1(uint8 SIDH, uint8 SIDL, uint8 LENGTH) {
    // Setup the registers
    MCP2515_SPI1_RegModify(MCP_TXB0CTRL, MCP_TXB_ABTF_M | MCP_TXB_MLOA_M \
    | MCP_TXB_TXERR_M | MCP_TXB_TXREQ_M, 0x00);
    MCP2515_SPI1_RegModify(MCP_TXB0CTRL, 0x03, 0xFF);

    /******************************************************************/
    /* User will define the data and CAN ID later */

    // MCP2515_SPI1_WriteReg(MCP_TXB0SIDH, SIDH, 1);  
    // MCP2515_SPI1_WriteReg(MCP_TXB0SIDL, SIDL, 1);   
    // MCP2515_SPI1_WriteReg(MCP_TXB0DLC, LENGTH, 1);

    // for(int i = 0; i < LENGTH; i++) {
    //     MCP2515_SPI1_WriteReg(TxDataAdrr[i], DATA_ARRAY, ONE_BYTE);
    // }
    
    /*****************************************************************/

    MCP2515_SPI1_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
} 

void SendData_SPI2(uint8 SIDH, uint8 SIDL, uint8 LENGTH) {
    // Setup the registers
    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, MCP_TXB_ABTF_M | MCP_TXB_MLOA_M \
    | MCP_TXB_TXERR_M | MCP_TXB_TXREQ_M, 0x00);
    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, 0x03, 0xFF);

    /******************************************************************/
    /* User will define the data and CAN ID later */

    // MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, SIDH, 1);  
    // MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, SIDL, 1);   
    // MCP2515_SPI2_WriteReg(MCP_TXB0DLC, LENGTH, 1);

    // for(int i = 0; i < LENGTH; i++) {
    //     MCP2515_SPI2_WriteReg(TxDataAdrr[i], DATA_ARRAY, ONE_BYTE);
    // }

    /*****************************************************************/
    MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
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

