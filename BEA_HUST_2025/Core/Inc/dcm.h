/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_H
#define _DCM_H

#include "main.h"
#include "stm32f4xx_it.h"

// UDS Service Response Codes
#define INVALID_LENGTH_RESPONSE_CODE 0x13
#define INVALID_DID_RESPONSE_CODE 0x31
#define READ_SID 0x22
#define WRITE_SID 0x2E
#define SESSION_SID 0x27
#define NRC 0x7F
#define ACCESS_DENIED_CODE 0x33
#define WRONG_KEY_RESPONSE_CODE 0x35
#define GENERAL_REJECT 0x10

// DCM function prototypes
void DCM_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame);

// External variables (defined in main.c)
extern uint16_t AVAILABLE_SERVICE;
extern volatile uint8_t SecurityUnlocked;
extern uint8_t SeedProvided;
extern uint32_t newStdID;
extern CAN_TxHeaderTypeDef CAN1_pHeader;
extern uint16_t iso_tp_rx_len;

#endif
