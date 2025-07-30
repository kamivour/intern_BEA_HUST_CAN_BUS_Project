/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_WDBI_H
#define _DCM_WDBI_H

#include "dcm.h"

// Service 0x2E - Write Data by Identifier function prototypes
void DCM_WDBI_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame);

#endif
