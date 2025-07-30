/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _ISO_TP_H
#define _ISO_TP_H

#include "main.h"

// ISO-TP Protocol Control Information (PCI) defines
#define ISO_TP_PCI_SF 0x00   // Single Frame
#define ISO_TP_PCI_FF 0x10   // First Frame
#define ISO_TP_PCI_CF 0x20   // Consecutive Frame
#define ISO_TP_PCI_FC 0x30   // Flow Control

// Flow Control Status
#define FC_CTS 0x00          // Continue To Send
#define FC_WAIT 0x01         // Wait
#define FC_OVFLW 0x02        // Overflow

// ISO-TP timing parameters (in ms)
#define ISO_TP_TIMEOUT_FC 10000   // 10 seconds timeout for Flow Control
#define ISO_TP_TIMEOUT_CF 10000
#define ISO_TP_ST_MIN 0x10      // Separation time minimum (16ms)

// ISO-TP state enumeration
typedef enum {
    ISO_TP_STATE_IDLE,
    ISO_TP_STATE_WAIT_FC,
    ISO_TP_STATE_SENDING_CF,
    ISO_TP_STATE_RECEIVING_FF,
    ISO_TP_STATE_RECEIVING_CF
} iso_tp_state_t;

// ISO-TP function prototypes
void iso_tp_init(void);
void iso_tp_process_rx(uint8_t *can_data);
void iso_tp_send_fc(uint8_t status);
void iso_tp_send_sf(uint8_t *data, uint8_t len);
void iso_tp_send_ff(uint8_t *data, uint16_t len);
void iso_tp_send_cf(void);
void iso_tp_handle_timeout(void);
uint8_t iso_tp_get_data_length(uint8_t *buffer, uint16_t *total_len);
void iso_tp_timer_update(void);
void iso_tp_start_timer(uint32_t timeout_ms);

// External variables (defined in main.c)
extern uint8_t CAN2_DATA_TX[8];
extern CAN_TxHeaderTypeDef CAN2_pHeader;
extern uint32_t CAN2_pTxMailbox;
extern CAN_HandleTypeDef hcan2;
extern uint8_t REQ_BUFFER[4096];
extern uint16_t iso_tp_rx_len;

#endif
