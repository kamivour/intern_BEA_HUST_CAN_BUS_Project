
/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_rdbi.h"
#include "iso_tp.h"

// External functions from main.c
extern void delay(uint16_t delay);

void DCM_RDBI_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame) {
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
        uds_data = rx_buffer;
        data_len = iso_tp_rx_len; // Use the total length from ISO-TP
    }
    
    if (data_len != 3) // Invalid length
    {
        response_data[0] = NRC;
        response_data[1] = READ_SID;
        response_data[2] = INVALID_LENGTH_RESPONSE_CODE;
        response_len = 3;
    } else if (((uds_data[1] << 8) | uds_data[2]) != AVAILABLE_SERVICE) // INVALID DID
    {
        response_data[0] = NRC;
        response_data[1] = READ_SID;
        response_data[2] = INVALID_DID_RESPONSE_CODE;
        response_len = 3;
    } else // Correct message
    {
        response_data[0] = uds_data[0] + 0x40; // Positive response
        response_data[1] = uds_data[1];
        response_data[2] = uds_data[2];
        response_data[3] = (CAN1_pHeader.StdId) >> 8;
        response_data[4] = CAN1_pHeader.StdId & 0xFF;
        response_len = 5;
    }
    
    // Send response using ISO-TP
    if (response_len <= 7) {
        iso_tp_send_sf(response_data, response_len);
    } else {
        iso_tp_send_ff(response_data, response_len);
    }
}


