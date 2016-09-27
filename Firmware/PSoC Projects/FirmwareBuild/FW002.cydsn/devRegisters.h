#if !defined(DEV_REGISTERS_H)
#define DEV_REGISTERS_H
#include "stdint.h"
    
void initDevRegisters( void );
uint8_t readDevRegister( uint8_t );
void writeDevRegister( uint8_t regNumberIn, uint8_t dataToWrite );
void incrementDevRegister( uint8_t );
uint8_t getChangedStatus( uint8_t regNumberIn );
void clearChangedStatus( uint8_t regNumberIn );

#endif