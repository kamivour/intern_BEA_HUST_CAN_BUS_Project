
/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_wdbi.h"
#include "iso_tp.h"

// External functions from main.c
extern void delay(uint16_t delay);

void DCM_WDBI_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame) {
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
    
    if (!SecurityUnlocked) {
        response_data[0] = NRC;
        response_data[1] = WRITE_SID;
        response_data[2] = ACCESS_DENIED_CODE;
        response_len = 3;
    } else if (data_len < 5) {
        response_data[0] = NRC;
        response_data[1] = WRITE_SID;
        response_data[2] = INVALID_LENGTH_RESPONSE_CODE;
        response_len = 3;
    } else if (((uds_data[1] << 8) | uds_data[2]) != AVAILABLE_SERVICE) // INVALID DID
    {
        response_data[0] = NRC;
        response_data[1] = WRITE_SID;
        response_data[2] = INVALID_DID_RESPONSE_CODE;
        response_len = 3;
    } else {
        // Update the Data Identifier with the new written data
        AVAILABLE_SERVICE = ((((uint16_t) uds_data[3]) << 8) | (uds_data[4])) & 0xFFFF;
        newStdID = ((((uint32_t) uds_data[3]) << 8) | (uds_data[4])) & 0x7FF;
        response_data[0] = uds_data[0] + 0x40; // Positive response (0x6E)
        response_data[1] = uds_data[3];        // Echo written data high byte
        response_data[2] = uds_data[4];        // Echo written data low byte
        response_len = 3;
    }
    
    // Send response using ISO-TP
    if (response_len <= 7) {
        iso_tp_send_sf(response_data, response_len);
    } else {
        iso_tp_send_ff(response_data, response_len);
    }
}


