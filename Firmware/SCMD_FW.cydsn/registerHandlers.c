/******************************************************************************
registerHandlers.c
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
#include "project.h"
#include "registerHandlers.h"
#include "devRegisters.h"
#include "SCMD_config.h"
#include "serial.h"
#include "slaveEnumeration.h"

extern const uint16_t SCBCLK_UART_DIVIDER_TABLE[8];
extern const uint16_t SCBCLK_I2C_DIVIDER_TABLE[4];

//Variables to store user key setup
static uint8_t savedUserKey = 0;
static uint8_t savedMasterKey = 0;

void processMasterRegChanges( void )
{
	//Remote reads (window reads through interface)
	if(getChangedStatus(SCMD_REM_WRITE))
	{
        saveKeysFullAccess(); //allow writes to registers
        
		setStatusBit( SCMD_BUSY_BIT );
		WriteSlaveData( readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET), readDevRegister(SCMD_REM_DATA_WR) );
		writeDevRegister( SCMD_REM_WRITE, 0 );
		clearChangedStatus( SCMD_REM_WRITE );
		clearStatusBit( SCMD_BUSY_BIT );
        
        restoreKeys();//Replace previous keys
	}
	//Do writes before reads if both present
	if(getChangedStatus(SCMD_REM_READ))
	{
        saveKeysFullAccess(); //allow writes to registers
        
		setStatusBit( SCMD_BUSY_BIT );
		writeDevRegister( SCMD_REM_DATA_RD, ReadSlaveData(readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET)) );
		writeDevRegister( SCMD_REM_READ, 0 );
		clearChangedStatus(SCMD_REM_READ);
		clearStatusBit( SCMD_BUSY_BIT );
        
        restoreKeys();//Replace previous keys
	} 
	//Tell slaves to change their inversion/bridging if the master was written

	//Count number of motors on slaves (0 == no motors)
	int16_t motorTemp = readDevRegister( SCMD_SLV_TOP_ADDR );
	uint8_t motorAddrTemp = 0;
	uint8_t offsetTemp = 0;
	if(motorTemp == 0)
	{
		//no motors
	}
	else
	{
		motorTemp = (motorTemp - 0x50 + 1) * 2; //single slave at 0x50 results in 2, 16 slaves (0x5F) results in 32
	}
	
	if(getChangedStatus( SCMD_INV_2_9 ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		int i;
		for(i = 0; i < 8; i++)
		{
			//Does slave exist?
			if( i < motorTemp )
			{
				motorAddrTemp = 0x50 + (i / 2);
				//Send bit
				if( i % 2 )
				{
					//remainder exists
					offsetTemp = SCMD_MOTOR_B_INVERT;
				}
				else
				{
					offsetTemp = SCMD_MOTOR_A_INVERT;
				}
				WriteSlaveData( motorAddrTemp, offsetTemp, (readDevRegister( SCMD_INV_2_9 ) >> i) & 0x01 );
			}
		}
		clearChangedStatus( SCMD_INV_2_9 );
		clearStatusBit( SCMD_BUSY_BIT );
	} 
	if(getChangedStatus( SCMD_INV_10_17 ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		int i;
		for(i = 0; i < 8; i++)
		{
			//Does slave exist?
			if( i < motorTemp - 8 )
			{
				motorAddrTemp = 0x54 + (i / 2);
				//Send bit
				if( i % 2 )
				{
					//remainder exists
					offsetTemp = SCMD_MOTOR_B_INVERT;
				}
				else
				{
					offsetTemp = SCMD_MOTOR_A_INVERT;
				}
				WriteSlaveData( motorAddrTemp, offsetTemp, (readDevRegister( SCMD_INV_10_17 ) >> i) & 0x01 );
			}
		}
		clearChangedStatus( SCMD_INV_10_17 );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus( SCMD_INV_18_25 ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		int i;
		for(i = 0; i < 8; i++)
		{
			//Does slave exist?
			if( i < motorTemp - 16 )
			{
				motorAddrTemp = 0x58 + (i / 2);
				//Send bit
				if( i % 2 )
				{
					//remainder exists
					offsetTemp = SCMD_MOTOR_B_INVERT;
				}
				else
				{
					offsetTemp = SCMD_MOTOR_A_INVERT;
				}
				WriteSlaveData( motorAddrTemp, offsetTemp, (readDevRegister( SCMD_INV_18_25 ) >> i) & 0x01 );
			}
		}
		clearChangedStatus( SCMD_INV_18_25 );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus(SCMD_INV_26_33))
	{
		setStatusBit( SCMD_BUSY_BIT );
		int i;
		for(i = 0; i < 8; i++)
		{
			//Does slave exist?
			if( i < motorTemp - 24 )
			{
				motorAddrTemp = 0x5C + (i / 2);
				//Send bit
				if( i % 2 )
				{
					//remainder exists
					offsetTemp = SCMD_MOTOR_B_INVERT;
				}
				else
				{
					offsetTemp = SCMD_MOTOR_A_INVERT;
				}
				WriteSlaveData( motorAddrTemp, offsetTemp, (readDevRegister( SCMD_INV_26_33 ) >> i) & 0x01 );
			}
		}
		clearChangedStatus( SCMD_INV_26_33 );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus( SCMD_BRIDGE_SLV_L ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		int i;
		uint8_t motorAddrTemp = 0x50;
		if(readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)
		{
			//Slave exists in range -- send all bits
			for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister( SCMD_SLV_TOP_ADDR )); i++)
			{
				WriteSlaveData( motorAddrTemp, SCMD_BRIDGE, (readDevRegister( SCMD_BRIDGE_SLV_L ) >> i) & 0x01 );
				motorAddrTemp++;
			}
		}
		clearChangedStatus( SCMD_BRIDGE_SLV_L );
		clearStatusBit( SCMD_BUSY_BIT );
	} 
	if(getChangedStatus( SCMD_BRIDGE_SLV_H ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		int i;
		uint8_t motorAddrTemp = 0x58;
		if(readDevRegister( SCMD_SLV_TOP_ADDR ) >= 0x58)
		{
			//Slave exists in range -- send all bits
			for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister(SCMD_SLV_TOP_ADDR)); i++)
			{
				WriteSlaveData(motorAddrTemp, SCMD_BRIDGE, (readDevRegister( SCMD_BRIDGE_SLV_H ) >> i) & 0x01);
				motorAddrTemp++;
			}
		}
		clearChangedStatus( SCMD_BRIDGE_SLV_H );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus( SCMD_MASTER_LOCK ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		//Do local
		writeDevRegister( SCMD_LOCAL_MASTER_LOCK, readDevRegister( SCMD_MASTER_LOCK ) );
		//send out to slaves here
		int i;
		for( i = 0x50; i <= readDevRegister(SCMD_SLV_TOP_ADDR); i++)
		{
			WriteSlaveData(i, SCMD_LOCAL_MASTER_LOCK, readDevRegister( SCMD_MASTER_LOCK ) );
			CyDelayUs(100);
		}

		clearChangedStatus( SCMD_MASTER_LOCK );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus( SCMD_USER_LOCK ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		//Do local
		writeDevRegister( SCMD_LOCAL_USER_LOCK, readDevRegister(SCMD_USER_LOCK) );
		//send out to slaves here
		int i;
		for( i = 0x50; i <= readDevRegister(SCMD_SLV_TOP_ADDR); i++)
		{
			WriteSlaveData(i, SCMD_LOCAL_USER_LOCK, readDevRegister(SCMD_USER_LOCK) );
			CyDelayUs(100);
		}

		clearChangedStatus( SCMD_USER_LOCK );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus( SCMD_E_BUS_SPEED ))
	{
		saveKeysFullAccess(); //allow writes to registers
		
		//Do local
		writeDevRegister( SCMD_E_PORT_CLKDIV_U, 0 );
		writeDevRegister( SCMD_E_PORT_CLKDIV_L, SCBCLK_I2C_DIVIDER_TABLE[readDevRegister(SCMD_E_BUS_SPEED) & 0x03] );
		writeDevRegister( SCMD_E_PORT_CLKDIV_CTRL, 0 ); //Triggers clock change
		
		restoreKeys();//Replace previous keys
		
		clearChangedStatus( SCMD_E_BUS_SPEED );
	}
	if(getChangedStatus( SCMD_CONTROL_1 ))
	{
		saveKeysFullAccess(); //allow writes to registers

		setStatusBit( SCMD_BUSY_BIT );
		if( readDevRegister( SCMD_CONTROL_1 ) & SCMD_FULL_RESET_BIT )
		{
			hardReset();
			//Bit will be cleared by reset
		}
		if( readDevRegister( SCMD_CONTROL_1 ) & SCMD_RE_ENUMERATE_BIT )
		{
			reEnumerate();
			writeDevRegister( SCMD_CONTROL_1, readDevRegister( SCMD_CONTROL_1 ) & ~SCMD_RE_ENUMERATE_BIT ); //Clear bit
		}
		clearChangedStatus( SCMD_CONTROL_1 );
		clearStatusBit( SCMD_BUSY_BIT );
		
		restoreKeys();//Replace previous keys
		
	}
}

void processSlaveRegChanges( void )
{
	//Change our address in the I2C device if the register has changed
	if(getChangedStatus( SCMD_SLAVE_ADDR ))
	{
		EXPANSION_PORT_I2CSlaveSetAddress( readDevRegister( SCMD_SLAVE_ADDR ) );
		clearChangedStatus( SCMD_SLAVE_ADDR );
	} 

}

void processRegChanges( void )
{
	//  Check local invert and bridge regs
	if(getChangedStatus( SCMD_MOTOR_A_INVERT ))
	{
		if(readDevRegister( SCMD_BRIDGE ) == 1)
		{
			//write both A and B bits
			if(readDevRegister( SCMD_MOTOR_A_INVERT ) == 1)
			{
				OUTPUT_MUX_CTRL_Write( 0x07 );
			}
			else
			{
				OUTPUT_MUX_CTRL_Write( 0x04 );
			}
		}
		else
		{
			//Just config motor A
			if(readDevRegister( SCMD_MOTOR_A_INVERT ) == 1)
			{
				OUTPUT_MUX_CTRL_Write( OUTPUT_MUX_CTRL_Read() | 0x01 ); //set bit 0
			}
			else
			{
				OUTPUT_MUX_CTRL_Write( OUTPUT_MUX_CTRL_Read() & 0x06 ); //clear bit 0
			}
		}
		clearChangedStatus( SCMD_MOTOR_A_INVERT );
	} 
	if(getChangedStatus( SCMD_MOTOR_B_INVERT ))
	{
		if(readDevRegister( SCMD_BRIDGE ) == 1)
		{
			//do nothing
		}
		else
		{
			//Just config motor B
			if(readDevRegister( SCMD_MOTOR_B_INVERT ) == 1)
			{
				OUTPUT_MUX_CTRL_Write( OUTPUT_MUX_CTRL_Read() | 0x02 ); //set bit 1
			}
			else
			{
				OUTPUT_MUX_CTRL_Write( OUTPUT_MUX_CTRL_Read() & 0x05 ); //clear bit 1
			}
		}
		clearChangedStatus( SCMD_MOTOR_B_INVERT );
	} 
	if(getChangedStatus( SCMD_BRIDGE ))
	{
		if(readDevRegister( SCMD_BRIDGE ) == 1)
		{
			//Use A for inversion
			if(readDevRegister( SCMD_MOTOR_A_INVERT ) == 1)
			{
				OUTPUT_MUX_CTRL_Write( 0x07 );
			}
			else
			{
				OUTPUT_MUX_CTRL_Write( 0x04 );
			}
		}
		else
		{
			//set both A and B based on register
			OUTPUT_MUX_CTRL_Write( (readDevRegister( SCMD_MOTOR_A_INVERT ) & 0x01) | ((readDevRegister(SCMD_MOTOR_B_INVERT) & 0x01) << 1) );
		}
		
		clearChangedStatus(SCMD_BRIDGE);
	}
	//  Check for change of failsafe time/enable register SCMD_FSAFE_TIME
	if(getChangedStatus( SCMD_FSAFE_TIME ))
	{
		setStatusBit( SCMD_BUSY_BIT );
		uint8_t tempValue = readDevRegister( SCMD_FSAFE_TIME );
		if(( readDevRegister(SCMD_CONFIG_BITS) != 2 )&&(readDevRegister( SCMD_SLV_TOP_ADDR ) >= 0x50)) //if you are master, and there are slaves
		{
			//send out to slaves here
			int i;
			for( i = 0x50; i <= readDevRegister( SCMD_SLV_TOP_ADDR ); i++)
			{
				WriteSlaveData( i, SCMD_FSAFE_TIME, tempValue );
				CyDelayUs(100);
			}
		}
		if(tempValue)
		{
			//Set new time and restart
			FSAFE_TIMER_Stop();
			FSAFE_TIMER_WriteCounter( 0 );
			FSAFE_TIMER_WriteCompare( tempValue );
			FSAFE_TIMER_Start();
		}
		else
		{
			//stop timer
			FSAFE_TIMER_Stop();
		}
		clearChangedStatus( SCMD_FSAFE_TIME );
		clearStatusBit( SCMD_BUSY_BIT );
	}

	//Set enable state
	if(getChangedStatus(SCMD_DRIVER_ENABLE))
	{
		setStatusBit( SCMD_BUSY_BIT );
		A_EN_Write( readDevRegister( SCMD_DRIVER_ENABLE ) & 0x01 );
		B_EN_Write( readDevRegister( SCMD_DRIVER_ENABLE ) & 0x01 );
		if(( readDevRegister( SCMD_CONFIG_BITS ) != 2 )&&(readDevRegister( SCMD_SLV_TOP_ADDR ) >= 0x50)) //if you are master, and there are slaves
		{
			//send out to slaves here
			int i;
			for( i = 0x50; i <= readDevRegister( SCMD_SLV_TOP_ADDR ); i++)
			{
				WriteSlaveData( i, SCMD_DRIVER_ENABLE, readDevRegister( SCMD_DRIVER_ENABLE ) & 0x01 );
				CyDelayUs( 100 );
			}
		}
		
		clearChangedStatus( SCMD_DRIVER_ENABLE );
		clearStatusBit( SCMD_BUSY_BIT );
	}
	if(getChangedStatus( SCMD_E_PORT_CLKDIV_CTRL ))
	{
		//Do local only
		initExpansionSerial( readDevRegister( SCMD_CONFIG_BITS ) );

		clearChangedStatus( SCMD_E_PORT_CLKDIV_CTRL );
	}
	if(getChangedStatus( SCMD_U_PORT_CLKDIV_CTRL ))
	{
		//Do local only
		initUserSerial( readDevRegister( SCMD_CONFIG_BITS ) );

		clearChangedStatus( SCMD_U_PORT_CLKDIV_CTRL );
	}
	if(getChangedStatus( SCMD_GEN_TEST_WORD ))
	{
		//do actions based on input
		switch( readDevRegister( SCMD_GEN_TEST_WORD ) )
		{
		case 1:
			//Put config bit input on rem read reg
			writeDevRegister( SCMD_REM_DATA_RD, CONFIG_BITS_REG_Read() ^ 0x0F );
			break;
		default:
			break;
		}
		clearChangedStatus( SCMD_GEN_TEST_WORD );
	}

}

void setStatusBit( uint8_t bitMask )
{
	writeDevRegisterUnprotected( SCMD_STATUS_1, readDevRegister( SCMD_STATUS_1 ) | bitMask); //Write bit
}

void clearStatusBit( uint8_t bitMask )
{
	writeDevRegisterUnprotected( SCMD_STATUS_1, readDevRegister( SCMD_STATUS_1 ) & ~bitMask); //Clear bit
}

void saveKeysFullAccess( void )
{
	//Save current keys
	savedUserKey = readDevRegister( SCMD_LOCAL_USER_LOCK );
	savedMasterKey = readDevRegister( SCMD_LOCAL_MASTER_LOCK );
	//Allow writes
	writeDevRegister( SCMD_LOCAL_USER_LOCK, USER_LOCK_KEY );
	writeDevRegister( SCMD_LOCAL_MASTER_LOCK, MASTER_LOCK_KEY );
}

void restoreKeys( void )
{
	writeDevRegister( SCMD_LOCAL_MASTER_LOCK, savedMasterKey );
	writeDevRegister( SCMD_LOCAL_USER_LOCK, savedUserKey );	
}

/* [] END OF FILE */
