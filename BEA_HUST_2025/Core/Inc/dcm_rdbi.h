/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_RDBI_H
#define _DCM_RDBI_H

#include "dcm.h"

// Service 0x22 - Read Data by Identifier function prototypes
void DCM_RDBI_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame);

#endif
