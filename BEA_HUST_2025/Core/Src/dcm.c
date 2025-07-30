/*************void DCM_ProcessRequest(uint8_t *rx_buffer, uint8_t *tx_buffer) {
    // Determine if this is single-frame (with PCI) or multi-frame (pure UDS data)
    uint8_t pci = rx_buffer[0] & 0xF0;
    uint8_t *uds_data;
    
    if (pci == ISO_TP_PCI_SF) {
        // Single frame - extract UDS data after PCI
        uds_data = &rx_buffer[1];
    } else {
        // Multi-frame or pure UDS data - use the data directly
        uds_data = rx_buffer;
    }*****************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm.h"
/*for further services please add service header here*/
#include "dcm_rdbi.h"
#include "dcm_wdbi.h"
#include "dcm_seca.h"
#include "iso_tp.h"

// External functions from main.c
extern void USART3_SendString(uint8_t *ch);

void DCM_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame) {
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
    
    // Process UDS service based on Service ID
    switch (uds_data[0]) {
        case 0x22: // Read Data by Identifier
            DCM_RDBI_ProcessRequest(rx_buffer, is_single_frame);
            break;
        case 0x27: // Security Access
            DCM_SECA_ProcessRequest(rx_buffer, is_single_frame);
            break;
        case 0x2E: // Write Data by Identifier
            DCM_WDBI_ProcessRequest(rx_buffer, is_single_frame);
            break;
        default:
            USART3_SendString((uint8_t*) "Service not supported\n");
            break;
    }
}
