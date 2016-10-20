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
#include <stdint.h>
#include "project.h"
#include "registerHandlers.h"
#include "devRegisters.h"
#include "SCMD_config.h"
#include "serial.h"

extern const uint16_t SCBCLK_UART_DIVIDER_TABLE[8];
extern const uint16_t SCBCLK_I2C_DIVIDER_TABLE[4];

void processMasterRegChanges( void )
{
    //Remote reads (window reads through interface)
    if(getChangedStatus(SCMD_REM_WRITE))
    {
        WriteSlaveData( readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET), readDevRegister(SCMD_REM_DATA_WR) );
        writeDevRegister( SCMD_REM_WRITE, 0 );
        clearChangedStatus(SCMD_REM_WRITE);
    }
    //Do writes before reads if both present
    if(getChangedStatus(SCMD_REM_READ))
    {
        writeDevRegister(SCMD_REM_DATA_RD, ReadSlaveData(readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET)));
        writeDevRegister( SCMD_REM_READ, 0 );
        clearChangedStatus(SCMD_REM_READ);
    } 
    //Tell slaves to change their inversion/bridging if the master was written

    //Count number of motors on slaves (0 == no motors)
    int16_t motorTemp = readDevRegister(SCMD_SLV_TOP_ADDR);
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
    
    if(getChangedStatus(SCMD_INV_2_9))
    {
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
                WriteSlaveData(motorAddrTemp, offsetTemp, (readDevRegister(SCMD_INV_2_9) >> i) & 0x01);
            }
        }
        clearChangedStatus(SCMD_INV_2_9);
    } 
    if(getChangedStatus(SCMD_INV_10_17))
    {
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
                WriteSlaveData(motorAddrTemp, offsetTemp, (readDevRegister(SCMD_INV_10_17) >> i) & 0x01);
            }
        }
        clearChangedStatus(SCMD_INV_10_17);
    }
    if(getChangedStatus(SCMD_INV_18_25))
    {
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
                WriteSlaveData(motorAddrTemp, offsetTemp, (readDevRegister(SCMD_INV_18_25) >> i) & 0x01);
            }
        }
        clearChangedStatus(SCMD_INV_18_25);
    }
    if(getChangedStatus(SCMD_INV_26_33))
    {
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
                WriteSlaveData(motorAddrTemp, offsetTemp, (readDevRegister(SCMD_INV_26_33) >> i) & 0x01);
            }
        }
        clearChangedStatus(SCMD_INV_26_33);
    }
    if(getChangedStatus(SCMD_BRIDGE_SLV_L))
    {
        int i;
        uint8_t motorAddrTemp = 0x50;
        if((readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)&&(readDevRegister(SCMD_SLV_TOP_ADDR) <= 0x57))
        {
            //Slave exists in range -- send all bits
            for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister(SCMD_SLV_TOP_ADDR)); i++)
            {
                WriteSlaveData(motorAddrTemp, SCMD_BRIDGE, (readDevRegister(SCMD_BRIDGE_SLV_L) >> i) & 0x01);
                motorAddrTemp++;
            }
        }
        clearChangedStatus(SCMD_BRIDGE_SLV_L);
    } 
    if(getChangedStatus(SCMD_BRIDGE_SLV_H))
    {
        int i;
        uint8_t motorAddrTemp = 0x58;
        if((readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)&&(readDevRegister(SCMD_SLV_TOP_ADDR) <= 0x5F))
        {
            //Slave exists in range -- send all bits
            for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister(SCMD_SLV_TOP_ADDR)); i++)
            {
                WriteSlaveData(motorAddrTemp, SCMD_BRIDGE, (readDevRegister(SCMD_BRIDGE_SLV_H) >> i) & 0x01);
                motorAddrTemp++;
            }
        }
        clearChangedStatus(SCMD_BRIDGE_SLV_H);
    }
    if(getChangedStatus(SCMD_MASTER_LOCK))
    {
        //Do local
        writeDevRegister( SCMD_LOCAL_MASTER_LOCK, readDevRegister(SCMD_MASTER_LOCK) );
        //Do remote
        int i;
        uint8_t motorAddrTemp = 0x50;
        if((readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)&&(readDevRegister(SCMD_SLV_TOP_ADDR) <= 0x5F))
        {
            //Slave exists in range -- send all bits
            for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister(SCMD_SLV_TOP_ADDR)); i++)
            {
                WriteSlaveData(motorAddrTemp, SCMD_LOCAL_MASTER_LOCK, (readDevRegister(SCMD_MASTER_LOCK) >> i) & 0x01);
                motorAddrTemp++;
            }
        }
        clearChangedStatus(SCMD_MASTER_LOCK);
    }
    if(getChangedStatus(SCMD_USER_LOCK))
    {
        //Do local
        writeDevRegister( SCMD_LOCAL_USER_LOCK, readDevRegister(SCMD_USER_LOCK) );
        //Do remote
        int i;
        uint8_t motorAddrTemp = 0x50;
        if((readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)&&(readDevRegister(SCMD_SLV_TOP_ADDR) <= 0x5F))
        {
            //Slave exists in range -- send all bits
            for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister(SCMD_SLV_TOP_ADDR)); i++)
            {
                WriteSlaveData(motorAddrTemp, SCMD_LOCAL_USER_LOCK, (readDevRegister(SCMD_USER_LOCK) >> i) & 0x01);
                motorAddrTemp++;
            }
        }
        clearChangedStatus(SCMD_USER_LOCK);
    }
    if(getChangedStatus(SCMD_E_BUS_SPEED))
    {
        //Save current keys
        uint8_t tempUserKey = readDevRegister(SCMD_LOCAL_USER_LOCK);
        uint8_t tempMasterKey = readDevRegister(SCMD_LOCAL_MASTER_LOCK);
        //Allow writes
        writeDevRegister(SCMD_LOCAL_USER_LOCK, USER_LOCK_KEY);
        writeDevRegister(SCMD_LOCAL_MASTER_LOCK, MASTER_LOCK_KEY);
        
        //Do remote first -- configure slaves before reconfiguring the bus
        //int i;
        //uint8_t motorAddrTemp = 0x50;
        //if((readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)&&(readDevRegister(SCMD_SLV_TOP_ADDR) <= 0x5F))
        //{
        //    //Slave exists in range -- send all bits
        //    for(i = 0; (i <= 8) && (motorAddrTemp <= readDevRegister(SCMD_SLV_TOP_ADDR)); i++)
        //    {
        //        WriteSlaveData(motorAddrTemp, SCMD_LOCAL_USER_LOCK, (readDevRegister(SCMD_USER_LOCK) >> i) & 0x01);
        //        motorAddrTemp++;
        //    }
        //}
        
        //Do local
        writeDevRegister(SCMD_E_PORT_CLKDIV_U, 0);
        writeDevRegister(SCMD_E_PORT_CLKDIV_L, SCBCLK_I2C_DIVIDER_TABLE[readDevRegister(SCMD_E_BUS_SPEED) & 0x03]);
        writeDevRegister(SCMD_E_PORT_CLKDIV_CTRL, 0); //Triggers clock change
        
        //Replace keys
        writeDevRegister(SCMD_LOCAL_USER_LOCK, tempUserKey);
        writeDevRegister(SCMD_LOCAL_MASTER_LOCK, tempMasterKey);
        
        clearChangedStatus(SCMD_E_BUS_SPEED);
    }
}

void processSlaveRegChanges( void )
{
    //Change our address in the I2C device if the register has changed
    if(getChangedStatus(SCMD_SLAVE_ADDR))
    {
        EXPANSION_PORT_I2CSlaveSetAddress(readDevRegister(SCMD_SLAVE_ADDR));
        clearChangedStatus(SCMD_SLAVE_ADDR);
    } 

}

void processRegChanges( void )
{
    //  Check local invert and bridge regs
    if(getChangedStatus(SCMD_MOTOR_A_INVERT))
    {
        if(readDevRegister(SCMD_BRIDGE) == 1)
        {
            //write both A and B bits
            if(readDevRegister(SCMD_MOTOR_A_INVERT) == 1)
            {
                OUTPUT_MUX_CTRL_Write(0x07);
            }
            else
            {
                OUTPUT_MUX_CTRL_Write(0x04);
            }
        }
        else
        {
            //Just config motor A
            if(readDevRegister(SCMD_MOTOR_A_INVERT) == 1)
            {
                OUTPUT_MUX_CTRL_Write(OUTPUT_MUX_CTRL_Read() | 0x01); //set bit 0
            }
            else
            {
                OUTPUT_MUX_CTRL_Write(OUTPUT_MUX_CTRL_Read() & 0x06); //clear bit 0
            }
        }
        clearChangedStatus(SCMD_MOTOR_A_INVERT);
    } 
    if(getChangedStatus(SCMD_MOTOR_B_INVERT))
    {
        if(readDevRegister(SCMD_BRIDGE) == 1)
        {
            //do nothing
        }
        else
        {
            //Just config motor B
            if(readDevRegister(SCMD_MOTOR_B_INVERT) == 1)
            {
                OUTPUT_MUX_CTRL_Write(OUTPUT_MUX_CTRL_Read() | 0x02); //set bit 1
            }
            else
            {
                OUTPUT_MUX_CTRL_Write(OUTPUT_MUX_CTRL_Read() & 0x05); //clear bit 1
            }
        }
        clearChangedStatus(SCMD_MOTOR_B_INVERT);
    } 
    if(getChangedStatus(SCMD_BRIDGE))
    {
        if(readDevRegister(SCMD_BRIDGE) == 1)
        {
            //Use A for inversion
            if(readDevRegister(SCMD_MOTOR_A_INVERT) == 1)
            {
                OUTPUT_MUX_CTRL_Write(0x07);
            }
            else
            {
                OUTPUT_MUX_CTRL_Write(0x04);
            }
        }
        else
        {
            //set both A and B based on register
            OUTPUT_MUX_CTRL_Write((readDevRegister(SCMD_MOTOR_A_INVERT) & 0x01) | ((readDevRegister(SCMD_MOTOR_B_INVERT) & 0x01) << 1));
        }
        
        clearChangedStatus(SCMD_BRIDGE);
    }

    //  Check for change of failsafe time/enable register SCMD_FSAFE_TIME
    if(getChangedStatus(SCMD_FSAFE_TIME))
    {
        uint8_t tempValue = readDevRegister( SCMD_FSAFE_TIME );
        if(( readDevRegister(SCMD_CONFIG_BITS) != 2 )&&(readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)) //if you are master, and there are slaves
        {
            //send out to slaves here
            int i;
            for( i = 0x50; i <= readDevRegister(SCMD_SLV_TOP_ADDR); i++)
            {
                WriteSlaveData(i, SCMD_FSAFE_TIME, tempValue );
                CyDelayUs(100);
            }
        }

        if( tempValue )
        {
            //Set new time and restart
            FSAFE_TIMER_Stop();
            FSAFE_TIMER_WriteCounter(0);
            FSAFE_TIMER_WriteCompare( tempValue );
            FSAFE_TIMER_Start();
        }
        else
        {
            //stop timer
            FSAFE_TIMER_Stop();
        }
        clearChangedStatus(SCMD_FSAFE_TIME);
    }

    //Set enable state
    if(getChangedStatus(SCMD_DRIVER_ENABLE))
    {
        
        A_EN_Write(readDevRegister(SCMD_DRIVER_ENABLE) & 0x01);
        B_EN_Write(readDevRegister(SCMD_DRIVER_ENABLE) & 0x01);
        if(( readDevRegister(SCMD_CONFIG_BITS) != 2 )&&(readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)) //if you are master, and there are slaves
        {
            //send out to slaves here
            int i;
            for( i = 0x50; i <= readDevRegister(SCMD_SLV_TOP_ADDR); i++)
            {
                WriteSlaveData(i, SCMD_DRIVER_ENABLE, readDevRegister(SCMD_DRIVER_ENABLE) & 0x01 );
                CyDelayUs(100);
            }
        }
     
        clearChangedStatus(SCMD_DRIVER_ENABLE);
    }
    if(getChangedStatus(SCMD_E_PORT_CLKDIV_CTRL))
    {
        //Do local only
        initExpansionSerial(readDevRegister(SCMD_CONFIG_BITS));

        clearChangedStatus(SCMD_E_PORT_CLKDIV_CTRL);
    }
    if(getChangedStatus(SCMD_U_PORT_CLKDIV_CTRL))
    {
        //Do local only
        initUserSerial(readDevRegister(SCMD_CONFIG_BITS));

        clearChangedStatus(SCMD_U_PORT_CLKDIV_CTRL);
    }

}

/* [] END OF FILE */
