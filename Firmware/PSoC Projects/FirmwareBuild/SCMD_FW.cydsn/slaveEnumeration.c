/******************************************************************************
slaveEnumeration.c
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
#include <stdint.h>
#include <project.h>
#include "devRegisters.h"
#include "charMath.h"
#include "SCMD_config.h"
#include "serial.h"
#include "diagLEDS.h"

//Variables and associated #defines use in functions
static uint8_t slaveAddrEnumerator;

#define SCMDSlaveIdle 0
#define SCMDSlaveWaitForSync 1
#define SCMDSlaveWaitForAddr 2
#define SCMDSlaveDone 3

static uint8_t slaveState = SCMDSlaveIdle;

#define SCMDMasterIdle 0
#define SCMDMasterPollDefault 1
#define SCMDMasterSendAddr 2
#define SCMDMasterWait 3
#define SCMDMasterSendData 4

uint8_t masterState = SCMDMasterIdle;

extern volatile uint16_t masterSendCounter;
extern volatile bool masterSendCounterReset; //set this to 1 to reset counter.... self clearing

//Functions

void tickMasterSM( void )
{
    int slaveAddri;
    //Do master state machine
    uint8_t masterNextState = masterState;
    switch( masterState )
    {
    case SCMDMasterIdle:
        //Set initial variable value
        slaveAddrEnumerator = START_SLAVE_ADDR;
        CONFIG_OUT_Write(1);
        writeDevRegister(SCMD_SLV_POLL_CNT, 0);
        //Target 0x4A, point to address slot, clear local data
        writeDevRegister(SCMD_REM_ADDR, 0x4A);
        writeDevRegister(SCMD_REM_OFFSET, SCMD_SLAVE_ADDR);
        writeDevRegister(SCMD_REM_DATA_RD, 0);
        //Flag read op
        writeDevRegister(SCMD_REM_READ, 1);
        masterNextState = SCMDMasterPollDefault;
        break;
    case SCMDMasterPollDefault:
        //Poll address 0x4A
        incrementDevRegister(SCMD_SLV_POLL_CNT);
        writeDevRegister(SCMD_REM_ADDR, 0x4A);  //Target device 0x4A
        writeDevRegister(SCMD_REM_OFFSET, SCMD_SLAVE_ADDR);  //Get contents of self address register       
        writeDevRegister(SCMD_REM_DATA_RD, 0); //Clear the returned data slot
        //Flag read op
        writeDevRegister(SCMD_REM_READ, 1);
        
        masterNextState = SCMDMasterSendAddr;
        break;
    case SCMDMasterSendAddr:
        if( readDevRegister(SCMD_REM_DATA_RD) == 0x4A )//if the address came back reflected
        {
            writeDevRegister(SCMD_REM_DATA_WR, slaveAddrEnumerator);
            writeDevRegister(SCMD_REM_WRITE, 1);//flag the transfer
            writeDevRegister(SCMD_SLV_TOP_ADDR, slaveAddrEnumerator);//Save to local reg
            
            writeDevRegister(SCMD_SLV_POLL_CNT, 0);
            if(slaveAddrEnumerator < MAX_SLAVE_ADDR)
            {
                slaveAddrEnumerator++;
                masterNextState = SCMDMasterPollDefault; //go look for another
            }
            else
            {
                masterNextState = SCMDMasterSendData; //go back to this state
            }
        }
        else
        {
            masterNextState = SCMDMasterPollDefault; //poll again
        }
        if( readDevRegister(SCMD_SLV_POLL_CNT) > 200 )
        {
            //Must be done
            writeDevRegister( SCMD_STATUS_1, readDevRegister( SCMD_STATUS_1 ) | SCMD_ENUMERATION_B0); //Write "i'm done" bit
            writeDevRegister(SCMD_LOCAL_MASTER_LOCK, 0x00);  //Lock up Read-Only registers -- we're done configuring the slaves!
            masterNextState = SCMDMasterSendData;
        }

        CyDelay(10);
        break;
    case SCMDMasterWait:
        //Wait while tick counts
        if( readDevRegister(SCMD_UPDATE_RATE) != 0 )
        {
            //Do this if packet rate not set to 0 (force mode)
            if(masterSendCounter < readDevRegister(SCMD_UPDATE_RATE))
            {
            }
            else
            {
                masterNextState = SCMDMasterSendData;
            }
        }
        else
        {
            //force mode
            if(readDevRegister(SCMD_FORCE_UPDATE))
            {
                //clear force reg
                writeDevRegister( SCMD_FORCE_UPDATE, 0 );
                masterNextState = SCMDMasterSendData;
            }
        }
        break;
    case SCMDMasterSendData:
        //Set output drive levels for master
        PWM_1_WriteCompare(readDevRegister(SCMD_MA_DRIVE));
        PWM_2_WriteCompare(readDevRegister(SCMD_MB_DRIVE)); 
        for (slaveAddri = START_SLAVE_ADDR; slaveAddri <= readDevRegister(SCMD_SLV_TOP_ADDR); slaveAddri++)
        {
            //Write drive states out to slaves
            //  OLD WAY:
            //WriteSlaveData( slaveAddri, SCMD_MA_DRIVE, readDevRegister(SCMD_S1A_DRIVE + ((slaveAddri - START_SLAVE_ADDR) << 1 )) );
            //CyDelayUs(100);
            //WriteSlaveData( slaveAddri, SCMD_MB_DRIVE, readDevRegister(SCMD_S1B_DRIVE + ((slaveAddri - START_SLAVE_ADDR) << 1 )) );
            //CyDelayUs(100);
            //  NEW WAY:
            WriteSlave2Data( slaveAddri, SCMD_MA_DRIVE, readDevRegister(SCMD_S1A_DRIVE + ((slaveAddri - START_SLAVE_ADDR) << 1 )), readDevRegister(SCMD_S1B_DRIVE + ((slaveAddri - START_SLAVE_ADDR) << 1 )) );
            CyDelayUs(100);
            
        }
        masterSendCounterReset = 1; //maybe should be protected
        while(masterSendCounter > 0);
        masterNextState = SCMDMasterWait;
        break;
    default:
        break;
    }
    masterState = masterNextState;
    //Get data from next slave

}

void tickSlaveSM( void )
{
    //Now do the states.
    uint8_t slaveNextState = slaveState;
    switch( slaveState )
    {
    case SCMDSlaveIdle:
        slaveNextState = SCMDSlaveWaitForSync;
        break;
    case SCMDSlaveWaitForSync:
        if( CONFIG_IN_Read() == 1 )
        {
            //Become 4A
            writeDevRegister(SCMD_SLAVE_ADDR, 0x4A);
            slaveNextState = SCMDSlaveWaitForAddr;
        }
        break;
    case SCMDSlaveWaitForAddr:
        if( readDevRegister(SCMD_SLAVE_ADDR) != 0x4A )
        {
            //New address has been programmed
            CONFIG_OUT_Write(1);
            writeDevRegister(SCMD_LOCAL_MASTER_LOCK, 0x00);  //Lock up Read-Only registers - to preserve address
            slaveNextState = SCMDSlaveDone;
        }
        break;
    case SCMDSlaveDone:
        if(CONFIG_IN_Read() == 0)
        {
            //CONFIG_IN went low (it shouldn't).  wait a bit and reboot
            CyDelay(100);
            CySoftwareReset();
        }
        else
        {
            //Apply latest drives
            PWM_1_WriteCompare(readDevRegister(SCMD_MA_DRIVE));
            PWM_2_WriteCompare(readDevRegister(SCMD_MB_DRIVE)); 
        }
        break;
    default:
        break;
    }
    slaveState = slaveNextState;

}

void resetMasterSM( void )
{
    masterState = SCMDMasterIdle;
}

bool masterSMDone( void )
{
    bool returnVar = false;
    if(( masterState == SCMDMasterSendData ) || ( masterState == SCMDMasterWait ))
    {
        returnVar = true;
    }
    return returnVar;
}