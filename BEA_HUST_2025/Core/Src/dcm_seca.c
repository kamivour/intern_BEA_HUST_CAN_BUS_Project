/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_seca.h"
#include "iso_tp.h"

// External functions and variables from main.c
extern void delay(uint16_t delay);
extern void USART3_SendString(uint8_t *ch);
extern void calculate_key(uint8_t *input, uint8_t *output);
extern uint8_t compare_key(uint8_t *array1, uint8_t *array2, uint8_t length);
extern uint8_t SEED[6];
extern uint8_t KEY[6];
extern TIM_HandleTypeDef htim3;

void DCM_SECA_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame) {
    delay(100);
    uint8_t response_data[8];
    uint8_t response_len;
    
    // Determine if this is single-frame (with PCI) or multi-frame (pure UDS data)
    uint8_t pci = rx_buffer[0] & 0xF0;
    uint8_t *uds_data;
    uint8_t data_len;
    
    if (is_single_frame && (pci == ISO_TP_PCI_SF)) {
        // Single frame - extract UDS data after PCI
        data_len = rx_buffer[0] & 0x0F;
        uds_data = &rx_buffer[1];
    } else {
        // Multi-frame or pure UDS data - use the data directly
        // For multi-frame, rx_buffer already contains pure UDS data
        uds_data = rx_buffer;
        data_len = iso_tp_rx_len; // Use the total length from ISO-TP
    }
    
    switch (uds_data[1]) { // Sub-function
    case 0x01: // Request Seed
        if (data_len != 2) {
            response_data[0] = NRC;
            response_data[1] = SESSION_SID;
            response_data[2] = INVALID_LENGTH_RESPONSE_CODE;
            response_len = 3;
        } else {
            response_data[0] = uds_data[0] + 0x40; // Positive response
            response_data[1] = 0x01;
            for (int i = 0; i < 6; i++) {
                response_data[i + 2] = SEED[i];
            }
            response_len = 8;
            calculate_key(SEED, KEY);
            SeedProvided = 1;
        }
        break;
        
    case 0x02: // Send Key
        if (data_len != 8) {
            response_data[0] = NRC;
            response_data[1] = SESSION_SID;
            response_data[2] = INVALID_LENGTH_RESPONSE_CODE;
            response_len = 3;
        } else if (!compare_key(KEY, &uds_data[2], 6)) {
            response_data[0] = NRC;
            response_data[1] = SESSION_SID;
            response_data[2] = WRONG_KEY_RESPONSE_CODE;
            response_len = 3;
        } else if (SeedProvided && !SecurityUnlocked) {
            HAL_TIM_Base_Start_IT(&htim3);
            SecurityUnlocked = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

            USART3_SendString((uint8_t*) "Session Unlocked\n");

            response_data[0] = uds_data[0] + 0x40; // Positive response
            response_data[1] = 0x02;
            response_len = 2;
            SeedProvided = 0;
        } else {
            return; // Already unlocked or no seed provided
        }
        break;
        
    default:
        response_data[0] = NRC;
        response_data[1] = SESSION_SID;
        response_data[2] = INVALID_LENGTH_RESPONSE_CODE;
        response_len = 3;
        break;
    }
    
    // Send response using ISO-TP
    if (response_len <= 7) {
        iso_tp_send_sf(response_data, response_len);
    } else {
        iso_tp_send_ff(response_data, response_len);
    }
}

