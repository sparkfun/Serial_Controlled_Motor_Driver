#include "devRegisters.h"
#include "stdint.h"

#define REGISTER_TABLE_LENGTH 128

uint8_t registerTable[REGISTER_TABLE_LENGTH];
uint8_t registerChangedTable[REGISTER_TABLE_LENGTH];

void initDevRegisters( void )
{
    int i = 0;
    for( i = 0; i < REGISTER_TABLE_LENGTH; i++ )
    {
        registerTable[i] = 0x00;
        registerChangedTable[i] = 0x00;
    }
}

uint8_t readDevRegister( uint8_t regNumberIn )
{
    return registerTable[regNumberIn];
}

void writeDevRegister( uint8_t regNumberIn, uint8_t dataToWrite )
{
    registerTable[regNumberIn] = dataToWrite;
    registerChangedTable[regNumberIn] = 1;
}

void incrementDevRegister( uint8_t regNumberIn )
{
    if( registerTable[regNumberIn] < 0xFF )
    {
        registerTable[regNumberIn]++;
        registerChangedTable[regNumberIn] = 1;
    }
}

uint8_t getChangedStatus( uint8_t regNumberIn )
{
    return registerChangedTable[regNumberIn];
}

void clearChangedStatus( uint8_t regNumberIn )
{
    registerChangedTable[regNumberIn] = 0;
}
