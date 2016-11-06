/******************************************************************************
slaveEnumeration.c
Serial controlled motor driver firmware
marshall.taylor@sparkfun.com
7-8-2016
https://github.com/sparkfun/Serial_Controlled_Motor_Driver/

See github readme for mor information.

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#include <stdint.h>
#include <project.h>
#include "devRegisters.h"
#include "registerHandlers.h"
#include "charMath.h"
#include "SCMD_config.h"
#include "serial.h"
#include "diagLEDS.h"
#include "slaveEnumeration.h"

//Variables and associated #defines use in functions
static uint8_t slaveAddrEnumerator;
static uint8_t slaveData = 0;
static volatile bool slaveResetRequested = false;

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
        //Get full access
        writeDevRegister( SCMD_LOCAL_USER_LOCK, USER_LOCK_KEY );
        writeDevRegister( SCMD_LOCAL_MASTER_LOCK, MASTER_LOCK_KEY );
		
        slaveAddrEnumerator = START_SLAVE_ADDR;
		CONFIG_OUT_Write(1);
        writeDevRegister( SCMD_SLV_POLL_CNT, 0 );

        masterNextState = SCMDMasterPollDefault;
        break;
    case SCMDMasterPollDefault:
        incrementDevRegister( SCMD_SLV_POLL_CNT );
        slaveData =	ReadSlaveData( POLL_ADDRESS, SCMD_SLAVE_ADDR );

        masterNextState = SCMDMasterSendAddr;
        break;
    case SCMDMasterSendAddr:
        if( slaveData == POLL_ADDRESS )//if the address came back reflected
        {
			WriteSlaveData( POLL_ADDRESS, SCMD_SLAVE_ADDR, slaveAddrEnumerator );
            writeDevRegister( SCMD_SLV_TOP_ADDR, slaveAddrEnumerator );//Save to local reg
            writeDevRegister( SCMD_SLV_POLL_CNT, 0 ); //reset the polling counter.
            if(slaveAddrEnumerator < MAX_SLAVE_ADDR)  //We got a response and there are more addresses to explore
            {
                slaveAddrEnumerator++;
                masterNextState = SCMDMasterPollDefault; //go look for another
            }
            else  //We got a response but there are no more addresses to explore
            {
                //force the poll count to expire (this will cause state changes below)
                writeDevRegister( SCMD_SLV_POLL_CNT, MAX_POLL_LIMIT + 1 );
            }
        }
        else
        {
            masterNextState = SCMDMasterPollDefault; //poll again
        }
        if( readDevRegister( SCMD_SLV_POLL_CNT ) > MAX_POLL_LIMIT )
        {
            //Must be done
			setStatusBit( SCMD_ENUMERATION_BIT );  //Write "i'm done" bit
            writeDevRegister( SCMD_LOCAL_MASTER_LOCK, 0x00 );  //Lock up Read-Only registers -- we're done configuring the slaves!
			uint8_t configTemp = readDevRegister( SCMD_CONFIG_BITS );
			if((configTemp == 0) || (configTemp == 0x0D) || (configTemp == 0x0E)) //UART
			{
				//Display a splash screen
				USER_PORT_UartPutString("\r\nSparkFun Serial Controlled Motor Driver (SCMD)\r\n");
				USER_PORT_UartPutString("Enter 'H' for help.\r\n>");
			}
            masterNextState = SCMDMasterSendData;
        }
        CyDelay(10);
        break;
    case SCMDMasterWait:
        //Wait while tick counts
        if( readDevRegister( SCMD_UPDATE_RATE ) != 0 )
        {
            //Do this if packet rate not set to 0 (force mode)
            if(masterSendCounter < readDevRegister( SCMD_UPDATE_RATE ))
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
            if(readDevRegister( SCMD_FORCE_UPDATE ))
            {
                //clear force reg
                writeDevRegister( SCMD_FORCE_UPDATE, 0 );
                masterNextState = SCMDMasterSendData;
            }
        }
        break;
    case SCMDMasterSendData:
		setStatusBit( SCMD_BUSY_BIT );
        //Set output drive levels for master
        PWM_1_WriteCompare( readDevRegister( SCMD_MA_DRIVE ) );
        PWM_2_WriteCompare( readDevRegister( SCMD_MB_DRIVE ) ); 
        for (slaveAddri = START_SLAVE_ADDR; slaveAddri <= readDevRegister(SCMD_SLV_TOP_ADDR); slaveAddri++)
        {
            //Write drive states out to slaves
            WriteSlave2Data( slaveAddri, SCMD_MA_DRIVE, readDevRegister(SCMD_S1A_DRIVE + ((slaveAddri - START_SLAVE_ADDR) << 1 )), readDevRegister(SCMD_S1B_DRIVE + ((slaveAddri - START_SLAVE_ADDR) << 1 )) );
            CyDelayUs(100);
            
        }
		clearStatusBit( SCMD_BUSY_BIT );
        masterSendCounterReset = 1; //Request the ISR to reset the counter
        while(masterSendCounter > 0); //Counter now = 0
        masterNextState = SCMDMasterWait;
        break;
    default:
        break;
    }
    masterState = masterNextState;

    //This next block deals with the config_in line and its behavior
    if((CONFIG_IN_Read() == 1)&&(slaveResetRequested == 0))
    {
        //Newly detected config_in rising edge
        slaveResetRequested = 1;
        //Send the reset request
        switch(readDevRegister( SCMD_MST_E_IN_FN ))
        {
            default:
            case 0:
                slaveResetRequested = 0; //belay that order
            break;
            case 1:
                hardReset();
            break;
            case 2:
                reEnumerate();
            break;
        }
    }
    //State machine is done but the request is still high (we came from soft re-enum)
    if(masterSMDone()&&(slaveResetRequested == 1))
	{
		slaveResetRequested = 0;
		//Send preserved settings (trigger xfers only)
		writeDevRegister( SCMD_INV_2_9, readDevRegister( SCMD_INV_2_9 ));
		writeDevRegister( SCMD_INV_10_17, readDevRegister( SCMD_INV_10_17 ));
		writeDevRegister( SCMD_INV_18_25, readDevRegister( SCMD_INV_18_25 ));
		writeDevRegister( SCMD_INV_26_33, readDevRegister( SCMD_INV_26_33 ));
		writeDevRegister( SCMD_BRIDGE_SLV_L, readDevRegister( SCMD_BRIDGE_SLV_L ));
		writeDevRegister( SCMD_BRIDGE_SLV_H, readDevRegister( SCMD_BRIDGE_SLV_H ));
		writeDevRegister( SCMD_DRIVER_ENABLE, readDevRegister( SCMD_DRIVER_ENABLE ));
		writeDevRegister( SCMD_UPDATE_RATE, readDevRegister( SCMD_UPDATE_RATE ));
		writeDevRegister( SCMD_FORCE_UPDATE, readDevRegister( SCMD_FORCE_UPDATE ));
		writeDevRegister( SCMD_FSAFE_TIME, readDevRegister( SCMD_FSAFE_TIME ));
	}
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
            writeDevRegister( SCMD_SLAVE_ADDR, POLL_ADDRESS );
            slaveNextState = SCMDSlaveWaitForAddr;
        }
        break;
    case SCMDSlaveWaitForAddr:
        if(CONFIG_IN_Read() == 0)
        {
            //CONFIG_IN went low (it shouldn't).  wait a bit and reboot
			CONFIG_OUT_Write(0);
            CyDelay(100);
            CySoftwareReset();
        }
        if( readDevRegister(SCMD_SLAVE_ADDR) != POLL_ADDRESS )
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
			CONFIG_OUT_Write(0);
            CyDelay(100);
            CySoftwareReset();
        }
        else
        {
            //Apply latest drives
            PWM_1_WriteCompare( readDevRegister( SCMD_MA_DRIVE ) );
            PWM_2_WriteCompare( readDevRegister( SCMD_MB_DRIVE ) ); 
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

void hardReset( void )
{
    CONFIG_OUT_Write(0);
    CySoftwareReset();
}

void reEnumerate( void )
{
    writeDevRegister( SCMD_LOCAL_USER_LOCK, USER_LOCK_KEY);
    writeDevRegister( SCMD_LOCAL_MASTER_LOCK, MASTER_LOCK_KEY);
    
	clearStatusBit( SCMD_ENUMERATION_BIT );  //Clear "i'm done" bit

    //set slaveResetRequested to cause config transfer after re-enumeration
    slaveResetRequested = true;
    
    CONFIG_OUT_Write(0);

    //insert keys
    CyDelay(1000u);
    resetMasterSM();
}