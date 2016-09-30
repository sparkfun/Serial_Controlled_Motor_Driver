#include <project.h>
#include "devRegisters.h"
#include "stdint.h"

// Configuration constants from main
extern const uint8_t ID_WORD; //Device ID to be programmed into memory for reads
extern const uint8_t START_SLAVE_ADDR;//Start address of slaves
extern const uint8_t MAX_SLAVE_ADDR;//Max address of slaves

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
    writeDevRegister(SCMD_STATUS, 0x1A);
    writeDevRegister(SCMD_ID, ID_WORD);
    writeDevRegister(SCMD_CONFIG_BITS, CONFIG_BITS_REG_Read() ^ 0x0F);    // Read HW config bits
    writeDevRegister(SCMD_FSAFE_TIME, 0 );
    writeDevRegister(SCMD_MA_DRIVE, 0x80);
    writeDevRegister(SCMD_MB_DRIVE, 0x80);
    writeDevRegister(SCMD_REM_OFFSET, 0x01);
    writeDevRegister(SCMD_REM_ADDR, 0x4A);
    writeDevRegister(SCMD_SLV_POLL_CNT, 0);
    writeDevRegister(SCMD_SLAVE_ADDR, 0x10); // No one should ever ask for data on 0x10
    writeDevRegister(SCMD_BAUD_RATE, 0x07);
    writeDevRegister(SCMD_S1A_DRIVE, 0x80);
    writeDevRegister(SCMD_S1B_DRIVE, 0x80);
    writeDevRegister(SCMD_S2A_DRIVE, 0x80);
    writeDevRegister(SCMD_S2B_DRIVE, 0x80);
    writeDevRegister(SCMD_S3A_DRIVE, 0x80);
    writeDevRegister(SCMD_S3B_DRIVE, 0x80);
    writeDevRegister(SCMD_S4A_DRIVE, 0x80);
    writeDevRegister(SCMD_S4B_DRIVE, 0x80);
    writeDevRegister(SCMD_S5A_DRIVE, 0x80);
    writeDevRegister(SCMD_S5B_DRIVE, 0x80);
    writeDevRegister(SCMD_S6A_DRIVE, 0x80);
    writeDevRegister(SCMD_S6B_DRIVE, 0x80);
    writeDevRegister(SCMD_S7A_DRIVE, 0x80);
    writeDevRegister(SCMD_S7B_DRIVE, 0x80);
    writeDevRegister(SCMD_S8A_DRIVE, 0x80);
    writeDevRegister(SCMD_S8B_DRIVE, 0x80);
    writeDevRegister(SCMD_S9A_DRIVE, 0x80);
    writeDevRegister(SCMD_S9B_DRIVE, 0x80);
    writeDevRegister(SCMD_S10A_DRIVE, 0x80);
    writeDevRegister(SCMD_S10B_DRIVE, 0x80);
    writeDevRegister(SCMD_S11A_DRIVE, 0x80);
    writeDevRegister(SCMD_S11B_DRIVE, 0x80);
    writeDevRegister(SCMD_S12A_DRIVE, 0x80);
    writeDevRegister(SCMD_S12B_DRIVE, 0x80);
    writeDevRegister(SCMD_S13A_DRIVE, 0x80);
    writeDevRegister(SCMD_S13B_DRIVE, 0x80);
    writeDevRegister(SCMD_S14A_DRIVE, 0x80);
    writeDevRegister(SCMD_S14B_DRIVE, 0x80);
    writeDevRegister(SCMD_S15A_DRIVE, 0x80);
    writeDevRegister(SCMD_S15B_DRIVE, 0x80);
    writeDevRegister(SCMD_S16A_DRIVE, 0x80);
    writeDevRegister(SCMD_S16B_DRIVE, 0x80);

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
