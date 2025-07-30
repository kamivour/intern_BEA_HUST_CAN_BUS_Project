/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_SECA_H
#define _DCM_SECA_H

#include "dcm.h"

// Service 0x27 - Security Access function prototypes
void DCM_SECA_ProcessRequest(uint8_t *rx_buffer, uint8_t is_single_frame);

#endif
