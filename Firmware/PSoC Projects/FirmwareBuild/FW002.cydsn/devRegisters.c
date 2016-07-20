#include "devRegisters.h"
#include "stdint.h"

#define REGISTER_TABLE_LENGTH 128
#define ID_WORD 0xA9

uint8_t registerTable[REGISTER_TABLE_LENGTH];

void initDevRegisters( void )
{
    int i = 0;
    for( i = 0; i < REGISTER_TABLE_LENGTH; i++ )
    {
        registerTable[i] = 0x00;
    }
    registerTable[0] = 0x1A;
    registerTable[1] = ID_WORD;
}

uint8_t readDevRegister( uint8_t regNumberIn )
{
    return registerTable[regNumberIn];
}

void writeDevRegister( uint8_t regNumberIn, uint8_t dataToWrite )
{
    registerTable[regNumberIn] = dataToWrite;
}

