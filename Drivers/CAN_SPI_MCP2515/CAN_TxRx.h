#include "main.h"
#ifndef CAN_TXRX
#define CAN_TXRX

void Read_TXdata(int dlc_length);
void Read_RXdata(uint16* rx_id, uint8* base_adr);
#endif
