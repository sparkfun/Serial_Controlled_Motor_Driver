/******************************************************************************
devRegisters.c
Serial controlled motor driver firmware
marshall.taylor@sparkfun.com
7-8-2016
https://github.com/sparkfun/Serial_Controlled_Motor_Driver/tree/RevisionRepaired

See github readme for mor information.

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include "devRegisters.h"
#include "SCMD_config.h"

//Set accessable table size here:
#define REGISTER_TABLE_LENGTH 128

static uint8_t registerTable[REGISTER_TABLE_LENGTH];
static bool registerChangedTable[REGISTER_TABLE_LENGTH];
//This counts accesses beyond REGISTER_TABLE_LENGTH.  Using #defined names, getOutOfRangeCount(); should always return 0;
static uint16_t outOfRangeCount = 0;

void initDevRegisters( void )
{
    //Explicitly set all values in the table
    int i = 0;
    for( i = 0; i < REGISTER_TABLE_LENGTH; i++ )
    {
        registerTable[i] = 0x00;
        registerChangedTable[i] = 0x00;
    }
    
    //Set hardcoded initial values --
    //ID_WORD, START_SLAVE_ADDR, MAX_SLAVE_ADDR are defined within the included files
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
    writeDevRegister(SCMD_UPDATE_RATE, 0x0A);

}

uint8_t readDevRegister( uint8_t regNumberIn )
{
    if( regNumberIn >= REGISTER_TABLE_LENGTH )
    {
        if(outOfRangeCount < 0xFFFF)outOfRangeCount++;
        return 0;
    }
    else
    {
        return registerTable[regNumberIn];
    }
}

void writeDevRegister( uint8_t regNumberIn, uint8_t dataToWrite )
{
    if( regNumberIn >= REGISTER_TABLE_LENGTH )
    {
        if(outOfRangeCount < 0xFFFF)outOfRangeCount++;
    }
    else
    {
        registerTable[regNumberIn] = dataToWrite;
        registerChangedTable[regNumberIn] = 1;
    }
}

void incrementDevRegister( uint8_t regNumberIn )
{
    if( regNumberIn >= REGISTER_TABLE_LENGTH )
    {
        if(outOfRangeCount < 0xFFFF)outOfRangeCount++;
    }
    else
    {
        if( registerTable[regNumberIn] < 0xFF )
        {
            registerTable[regNumberIn]++;
            registerChangedTable[regNumberIn] = true;
        }
    }

}

bool getChangedStatus( uint8_t regNumberIn )
{
    if( regNumberIn >= REGISTER_TABLE_LENGTH )
    {
        if(outOfRangeCount < 0xFFFF)outOfRangeCount++;
        return false;
    }
    else
    {
        return registerChangedTable[regNumberIn];
    }
}

void clearChangedStatus( uint8_t regNumberIn )
{
    if( regNumberIn >= REGISTER_TABLE_LENGTH )
    {
        if(outOfRangeCount < 0xFFFF)outOfRangeCount++;
    }
    else
    {
        registerChangedTable[regNumberIn] = false;
    }
}

uint16_t getOutOfRangeCount( void )
{
    return outOfRangeCount;
}