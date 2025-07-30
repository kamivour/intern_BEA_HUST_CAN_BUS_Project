/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "iso_tp.h"
#include "dcm.h"
#include <string.h>

// ISO-TP variables
uint8_t iso_tp_rx_buffer[4096];     // Buffer for multi-frame reception
uint8_t iso_tp_tx_buffer[4096];     // Buffer for multi-frame transmission
uint16_t iso_tp_rx_index;           // Current index in RX buffer
uint16_t iso_tp_tx_len;             // Total length for TX
uint16_t iso_tp_tx_index;           // Current index in TX buffer
uint8_t iso_tp_sn;                  // Sequence number for CF
iso_tp_state_t iso_tp_state;        // Current ISO-TP state
uint32_t iso_tp_timer_start;        // Timer start timestamp
uint32_t iso_tp_timeout_value;      // Current timeout value in ms

// External functions from main.c
extern void delay(uint16_t delay);
extern void USART3_SendString(uint8_t *ch);
extern void PrintCANLog(uint16_t CANID, uint8_t *CAN_Frame);
extern void Error_Handler(void);

// External DCM processing function
extern void DCM_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame);

void iso_tp_init(void) {
    iso_tp_state = ISO_TP_STATE_IDLE;
    iso_tp_rx_len = 0;
    iso_tp_rx_index = 0;
    iso_tp_tx_len = 0;
    iso_tp_tx_index = 0;
    iso_tp_sn = 1;
    iso_tp_timer_start = 0;
    iso_tp_timeout_value = 0;
    memset(iso_tp_rx_buffer, 0, sizeof(iso_tp_rx_buffer));
    memset(iso_tp_tx_buffer, 0, sizeof(iso_tp_tx_buffer));
}

void iso_tp_timer_update(void) {
    if (iso_tp_timeout_value > 0) {
        uint32_t current_time = HAL_GetTick();
        if ((current_time - iso_tp_timer_start) >= iso_tp_timeout_value) {
            iso_tp_handle_timeout();
        }
    }
}

void iso_tp_start_timer(uint32_t timeout_ms) {
    iso_tp_timer_start = HAL_GetTick();
    iso_tp_timeout_value = timeout_ms;
}

void iso_tp_handle_timeout(void) {
    if (iso_tp_state == ISO_TP_STATE_WAIT_FC) {
        // N_As timeout: Flow Control not received within timeout
    } else if (iso_tp_state == ISO_TP_STATE_RECEIVING_CF) {
        // N_Cr timeout: Consecutive Frame not received within timeout
    }
    
    // Always abort and reset to IDLE state on timeout
    iso_tp_init();
}

uint8_t iso_tp_get_data_length(uint8_t *buffer, uint16_t *total_len) {
    uint8_t pci = buffer[0] & 0xF0;
    uint8_t sf_len = buffer[0] & 0x0F;
    
    if (pci == ISO_TP_PCI_SF) {
        *total_len = sf_len;
        return 1; // Data starts at index 1
    } else if (pci == ISO_TP_PCI_FF) {
        *total_len = ((uint16_t)(buffer[0] & 0x0F) << 8) | buffer[1];
        return 2; // Data starts at index 2
    }
    return 0;
}

void iso_tp_process_rx(uint8_t *can_data) {
    uint8_t pci = can_data[0] & 0xF0;
    uint16_t total_len;
    uint8_t data_start_idx;
    uint8_t data_len;
    
    switch (pci) {
        case ISO_TP_PCI_SF: // Single Frame
            data_start_idx = iso_tp_get_data_length(can_data, &total_len);
            if (total_len > 0 && total_len <= 7) {
                // Copy single frame data directly to REQ_BUFFER for UDS processing
                memcpy(REQ_BUFFER, &can_data[data_start_idx], total_len);
                
                // Process UDS service through DCM
                DCM_ProcessRequest(can_data, 1); // 1 = single frame
            }
            break;
            
        case ISO_TP_PCI_FF: // First Frame
            data_start_idx = iso_tp_get_data_length(can_data, &iso_tp_rx_len);
            
            if (iso_tp_rx_len > 7 && iso_tp_rx_len < 4096) {
                iso_tp_state = ISO_TP_STATE_RECEIVING_FF;
                iso_tp_rx_index = 0;
                iso_tp_sn = 1;
                
                // Copy first frame data
                data_len = 8 - data_start_idx;
                memcpy(&iso_tp_rx_buffer[iso_tp_rx_index], &can_data[data_start_idx], data_len);
                iso_tp_rx_index += data_len;
                
                // Send Flow Control - Continue to Send
                iso_tp_send_fc(FC_CTS);
                iso_tp_start_timer(ISO_TP_TIMEOUT_CF);
            }
            break;
            
        case ISO_TP_PCI_CF: // Consecutive Frame
            if (iso_tp_state == ISO_TP_STATE_RECEIVING_FF || iso_tp_state == ISO_TP_STATE_RECEIVING_CF) {
                uint8_t expected_sn = iso_tp_sn & 0x0F;
                uint8_t received_sn = can_data[0] & 0x0F;
                
                if (received_sn == expected_sn) {
                    // Copy consecutive frame data
                    data_len = (iso_tp_rx_len - iso_tp_rx_index > 7) ? 7 : (iso_tp_rx_len - iso_tp_rx_index);
                    memcpy(&iso_tp_rx_buffer[iso_tp_rx_index], &can_data[1], data_len);
                    iso_tp_rx_index += data_len;
                    iso_tp_sn++;
                    
                    if (iso_tp_rx_index >= iso_tp_rx_len) {
                        // Multi-frame reception complete
                        // Copy to REQ_BUFFER for UDS processing
                        memcpy(REQ_BUFFER, iso_tp_rx_buffer, iso_tp_rx_len);
                        
                        // Process UDS service through DCM
                        DCM_ProcessRequest(iso_tp_rx_buffer, 0); // 0 = multi-frame
                        
                        iso_tp_init(); // Reset state
                    } else {
                        iso_tp_state = ISO_TP_STATE_RECEIVING_CF;
                        iso_tp_start_timer(ISO_TP_TIMEOUT_CF);
                    }
                } else {
                    // Sequence number error
                    iso_tp_init();
                }
            }
            break;
            
        case ISO_TP_PCI_FC: // Flow Control
            if (iso_tp_state == ISO_TP_STATE_WAIT_FC) {
                uint8_t fc_status = can_data[0] & 0x0F;
                
                if (fc_status == FC_CTS) {
                    // Continue sending consecutive frames
                    iso_tp_state = ISO_TP_STATE_SENDING_CF;
                    iso_tp_send_cf();
                } else if (fc_status == FC_WAIT) {
                    // Wait for next FC
                    iso_tp_start_timer(ISO_TP_TIMEOUT_FC);
                } else {
                    // Overflow or error
                    iso_tp_init();
                }
            }
            break;
    }
}

void iso_tp_send_fc(uint8_t status) {
    CAN2_DATA_TX[0] = ISO_TP_PCI_FC | status;
    CAN2_DATA_TX[1] = 0; // Block size (0 = no limit)
    CAN2_DATA_TX[2] = ISO_TP_ST_MIN; // Separation time
    for (int i = 3; i < 8; i++) {
        CAN2_DATA_TX[i] = 0x55; // Padding
    }
    
    if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK) {
        Error_Handler();
    }
    delay(100);
}

void iso_tp_send_sf(uint8_t *data, uint8_t len) {
    if (len <= 7) {
        CAN2_DATA_TX[0] = ISO_TP_PCI_SF | len;
        memcpy(&CAN2_DATA_TX[1], data, len);
        for (int i = len + 1; i < 8; i++) {
            CAN2_DATA_TX[i] = 0x55; // Padding
        }
        
        if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK) {
            Error_Handler();
        }
        delay(100);
    }
}

void iso_tp_send_ff(uint8_t *data, uint16_t len) {
    if (len > 7 && len < 4096) {
        // Store data for consecutive frames
        memcpy(iso_tp_tx_buffer, data, len);
        iso_tp_tx_len = len;
        iso_tp_tx_index = 6; // First 6 bytes sent in FF
        iso_tp_sn = 1;
        
        // Send First Frame
        CAN2_DATA_TX[0] = ISO_TP_PCI_FF | ((len >> 8) & 0x0F);
        CAN2_DATA_TX[1] = len & 0xFF;
        memcpy(&CAN2_DATA_TX[2], data, 6);
        
        // Print the first frame being sent
        PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
        
        if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK) {
            Error_Handler();
        }
        
        iso_tp_state = ISO_TP_STATE_WAIT_FC;
        iso_tp_start_timer(ISO_TP_TIMEOUT_FC);
        delay(100);
    }
}

void iso_tp_send_cf(void) {
    while (iso_tp_tx_index < iso_tp_tx_len && iso_tp_state == ISO_TP_STATE_SENDING_CF) {
        CAN2_DATA_TX[0] = ISO_TP_PCI_CF | (iso_tp_sn & 0x0F);
        
        uint8_t remaining = iso_tp_tx_len - iso_tp_tx_index;
        uint8_t to_send = (remaining > 7) ? 7 : remaining;
        
        memcpy(&CAN2_DATA_TX[1], &iso_tp_tx_buffer[iso_tp_tx_index], to_send);
        iso_tp_tx_index += to_send;
        
        for (int i = to_send + 1; i < 8; i++) {
            CAN2_DATA_TX[i] = 0x55; // Padding
        }
        
        // Print the consecutive frame being sent
        PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
        
        if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK) {
            Error_Handler();
        }
        
        iso_tp_sn++;
        delay(ISO_TP_ST_MIN + 10); // Add small delay between frames
        
        if (iso_tp_tx_index >= iso_tp_tx_len) {
            // Transmission complete
            iso_tp_init();
            break;
        }
    }
}

// Special function for ECU responses - sends multi-frame without waiting for FC
void iso_tp_send_response_auto(uint8_t *data, uint16_t len) {
    if (len <= 7) {
        // Single Frame
        iso_tp_send_sf(data, len);
    } else {
        // Multi-Frame - send FF + CF automatically without waiting for FC
        
        // Send First Frame
        CAN2_DATA_TX[0] = ISO_TP_PCI_FF | ((len >> 8) & 0x0F);
        CAN2_DATA_TX[1] = len & 0xFF;
        memcpy(&CAN2_DATA_TX[2], data, 6);
        
        PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
        
        if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK) {
            Error_Handler();
        }
        delay(50);
        
        // Send Consecutive Frames automatically
        uint16_t sent_bytes = 6;
        uint8_t sequence_number = 1;
        
        while (sent_bytes < len) {
            uint8_t remaining = len - sent_bytes;
            uint8_t to_send = (remaining > 7) ? 7 : remaining;
            
            CAN2_DATA_TX[0] = ISO_TP_PCI_CF | (sequence_number & 0x0F);
            memcpy(&CAN2_DATA_TX[1], &data[sent_bytes], to_send);
            
            // Pad remaining bytes
            for (int i = to_send + 1; i < 8; i++) {
                CAN2_DATA_TX[i] = 0x55;
            }
            
            PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
            
            if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox) != HAL_OK) {
                Error_Handler();
            }
            
            sent_bytes += to_send;
            sequence_number++;
            if (sequence_number > 15) sequence_number = 0;
            
            delay(10); // Small delay between frames
        }
    }
}
