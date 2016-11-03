/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#if !defined(REGISTERHANDLERS_H)
#define REGISTERHANDLERS_H
#include <stdint.h> 

//Prototypes
void processMasterRegChanges( void );
void processSlaveRegChanges( void );
void processRegChanges( void );
void setStatusBit( uint8_t bitMask );
void clearStatusBit( uint8_t bitMask );
    

#endif
/* [] END OF FILE */
