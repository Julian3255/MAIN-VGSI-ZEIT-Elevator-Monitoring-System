#include "main.h"
#ifndef CAN_TXRX
#define CAN_TXRX

/* Number of byte to be transmit / receive */
#define ONE_BYTE            (1u)

typedef struct 
{
    uint8 ID_400_buffer[8];
    uint8 ID_481_buffer[8];
    uint8 ID_490_buffer[8];
    uint8 ID_68B_buffer[8];
} EMS_Buffer;

/* Define CAN frame for the CLOSE door command*/
#define CLOSE_CMD_SIDH      (0x40)
#define CLOSE_CMD_SIDL      (0x00)
#define CLOSE_CMD_DLC       (0x04)
#define CLOSE_CMD_BYTE0     (0x00)
#define CLOSE_CMD_BYTE1     (0x00)
#define CLOSE_CMD_BYTE2     (0x00)
#define CLOSE_CMD_BYTE3     (0xF0)
#define CLOSE_CMD_BYTE4     (0x00)
#define CLOSE_CMD_BYTE5     (0x00)
#define CLOSE_CMD_BYTE6     (0x00)
#define CLOSE_CMD_BYTE7     (0x00)


/* Define CAN frame for the OPEN door command*/
#define OPEN_CMD_SIDH       (0x40)
#define OPEN_CMD_SIDL       (0x00)
#define OPEN_CMD_DLC        (0x04)
#define OPEN_CMD_BYTE0      (0x00)
#define OPEN_CMD_BYTE1      (0x30)
#define OPEN_CMD_BYTE2      (0x00)
#define OPEN_CMD_BYTE3      (0xF0)
#define OPEN_CMD_BYTE4      (0x00)
#define OPEN_CMD_BYTE5      (0x00)
#define OPEN_CMD_BYTE6      (0x00)
#define OPEN_CMD_BYTE7      (0x00)

/* Function declaration */
void Read_CAN_ID(void);
void Read_TXdata(int channel);
void Read_RXdata(uint16* rx_id, uint8* base_adr);
void Read_Slave1_RXdata(uint16* rx_id, uint8* base_adr);
void getEMStime_Master(void);
void getEMStime_Slave1(void);
void Send_RXdata(uint16* rx_id);
void Send_CloseDoor(void);
void Send_OpenDoor(void);

#endif



