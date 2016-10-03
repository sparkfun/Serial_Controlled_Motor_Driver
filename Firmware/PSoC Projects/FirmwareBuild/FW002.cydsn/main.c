/******************************************************************************
main.c
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
#include "devRegisters.h"
#include "charMath.h"
#include "config.h"
#include "serial.h"
#include "diagLEDS.h"

extern uint16_t SCBCLK_UART_DIVIDER;

//To use software to define the config bits, uncomment this #define and set value below.
//#define USE_SW_CONFIG_BITS

//Otherwise, this global is used to get the bits from the hardware jumpers
volatile uint8_t CONFIG_BITS = 0x3;
    //0 -- UART
    //1 -- SPI
    //2 -- I2C slave on slave port
    //3 -- I2C @ 0x58

uint8_t slaveAddrEnumerator;

#define SCMDSlaveIdle 0
#define SCMDSlaveWaitForSync 1
#define SCMDSlaveWaitForAddr 2
#define SCMDSlaveDone 3

uint8_t slaveState = SCMDSlaveIdle;

#define SCMDMasterIdle 0
#define SCMDMasterPollDefault 1
#define SCMDMasterSendAddr 2
#define SCMDMasterDone 3

uint8_t masterState = SCMDMasterIdle;

/*******************************************************************************
* Define Interrupt service routine and allocate an vector to the Interrupt
********************************************************************************/
CY_ISR(FSAFE_TIMER_Interrupt)
{
    
	/* Clear TC Inerrupt */
   	FSAFE_TIMER_ClearInterrupt(FSAFE_TIMER_INTR_MASK_CC_MATCH);
    uint16_t tempValue = readDevRegister(SCMD_FSAFE_FAULTS);
    if( tempValue < 255)
    {
        writeDevRegister( SCMD_FSAFE_FAULTS, tempValue + 1 );
    }

    //Take remedial actions
    //  Stop the motors
    writeDevRegister( SCMD_MA_DRIVE, 0x80 );
    writeDevRegister( SCMD_MB_DRIVE, 0x80 );
    //  Reconfigure?  Try and recover the bus?
    
    //  Tell someone
    setDiagMessage(7,8);

    
    //reset the counter
    FSAFE_TIMER_Stop();
    FSAFE_TIMER_WriteCounter(0);
    FSAFE_TIMER_Start();
}

int main()
{
    initDevRegisters();  //Prep device registers, set initial values (triggers actions)
    slaveAddrEnumerator = START_SLAVE_ADDR;  
#ifndef USE_SW_CONFIG_BITS
    CONFIG_BITS = readDevRegister(SCMD_CONFIG_BITS); //Get the bits value
#endif

    DIAG_LED_CLK_Start();
    KHZ_CLK_Start();
    setDiagMessage(0, CONFIG_BITS);
    
    CONFIG_IN_Write(0); //Tell the slaves to start fresh
    //Do a boot-up delay
    CyDelay(1500u);
    if(CONFIG_BITS != 0x02) CyDelay(1000u); //Give the slaves extra time
    
    
    MODE_Write(1);
    
    //Debug timer
    DEBUG_CLK_Start();
    DEBUG_TIMER_Start();
    
    //Failsafe timer
    FSAFE_ISR_StartEx(FSAFE_TIMER_Interrupt);
    FSAFE_CLK_Start();
    FSAFE_TIMER_Start();
       
    CyDelay(50u);
    
    /* Start the components */
    PWM_1_Start();
    PWM_1_WriteCompare(128u);
    PWM_2_Start();
    PWM_2_WriteCompare(128u);        
    
    //Config USER_PORT
    if(CONFIG_BITS == 0) //UART
    {
        SetScbConfiguration(OP_MODE_UART);
    }
    if(CONFIG_BITS == 1) //SPI
    {
        SetScbConfiguration(OP_MODE_SPI);
        USER_PORT_SpiUartClearRxBuffer();
    }
    if(CONFIG_BITS == 2) //Slave
    {
        SetExpansionScbConfigurationSlave();
    }
    else
    {
        SetExpansionScbConfigurationMaster();
    }
    if((CONFIG_BITS >= 0x3)&&(CONFIG_BITS <= 0xE)) //I2C
    {
        SetScbConfiguration(OP_MODE_I2C);
    }
    
    Clock_1_Start();
        
    CyGlobalIntEnable; 
    
    uint8_t addressPointer = 0;
        
    //rxBuffer for serial input storage
    uint8_t rxBufferPtr = 0;
    char rxBuffer[20];

    setDiagMessage(1, 1);    
    while(1)
    {
        if(CONFIG_BITS == 0) //UART
        {
            //General:
            //Save data (echo it) until we get a delimiter
            //Parse data and take action
            char ch = USER_PORT_UartGetChar();
            if( ch )
            {
                //Show data on LED
                LED_PULSE_Write(1);
                //Save data
                if( rxBufferPtr < 19)
                {
                    rxBuffer[rxBufferPtr] = ch;
                    rxBufferPtr++;
                }
                else
                {
                    //Overwrite last
                    rxBuffer[rxBufferPtr - 1] = ch;
                    USER_PORT_UartPutString("ovf\r\n");
                }
                //Echo
                USER_PORT_UartPutChar(ch);
                LED_PULSE_Write(0);
            }
            //if((rxBuffer[rxBufferPtr - 1] == '\n')||(rxBuffer[rxBufferPtr - 1] == '\r'))//Delimiter found
            if(rxBuffer[rxBufferPtr - 1] == '\n')//Delimiter found, don't delimit \r
            {
                LED_PULSE_Write(1);
                uint8_t errorFlag = 0;
                uint8_t dirPtr = 2;
                uint8_t valLsd = 2;
                int16_t motorNum = 0;
                int16_t motorDir = 0;
                int16_t motorDrive = 0;
                int16_t slaveNum = 0;
                uint8_t addressTemp = 0;
                uint8_t dataTemp = 0;
                uint8_t maxMotorCount = 2;
                //Do some action
                //  Branch based of type of message
                switch(rxBuffer[0])
                {
                    case 'M':
                    //Find direction
                    if(( rxBuffer[dirPtr] != 'F' )&&( rxBuffer[dirPtr] != 'R' )&&( rxBuffer[dirPtr] != 'S' )) // not here
                    {
                        dirPtr++;
                    }
                    if(( rxBuffer[dirPtr] != 'F' )&&( rxBuffer[dirPtr] != 'R' )&&( rxBuffer[dirPtr] != 'S' )) // not here
                    {
                        //Bail, otherwise dirPtr now points to direction
                        errorFlag = 1;
                        break;
                    }
                    //Get motor number
                    //  Get the one to the left of direction
                    motorNum = char2hex(rxBuffer[dirPtr - 1]);
                    //  If direction in 3 spot, get 10s place
                    if( dirPtr == 3 )
                    {
                        motorNum += 10 * char2hex(rxBuffer[dirPtr - 2]);
                    }
                    //Parse value
                    // First seek number of places
                    for(valLsd = dirPtr + 1; ((( rxBuffer[valLsd + 1] != '\n' )&&( rxBuffer[valLsd + 1] != '\r' ))&&( valLsd != (dirPtr + 3))); valLsd++);
                    // Now valLsd should be the least significant digit of the 0-100 number
                    uint16_t driveMultiplier = 1;
                    while(dirPtr != valLsd)
                    {
                        motorDrive += char2hex(rxBuffer[valLsd]) * driveMultiplier;
                        valLsd--;
                        driveMultiplier *= 10;
                    }
                    //Check for polarity switching
                    if( rxBuffer[dirPtr] == 'S' )
                    {
                        //Check if motor is in range
                        maxMotorCount = (readDevRegister(SCMD_SLV_TOP_ADDR) - 0x50 + 1 ) * 2 + 2;
                        if( motorNum < maxMotorCount )
                        {
                        }
                        else
                        {
                            errorFlag = 2;
                            break;
                        }
                        //Now write to the correct place
                        if(motorNum == 0)
                        {
                            writeDevRegister(SCMD_MOTOR_A_INVERT, readDevRegister(SCMD_MOTOR_A_INVERT) ^ 0x01);
                        }
                        else if(motorNum == 1)
                        {
                            writeDevRegister(SCMD_MOTOR_B_INVERT, readDevRegister(SCMD_MOTOR_B_INVERT) ^ 0x01);
                        }
                        else if(motorNum < 10) //must be 2 through 9
                        {
                            dataTemp = readDevRegister(SCMD_INV_2_9);
                            dataTemp ^= 0x01 << (motorNum - 2);
                            writeDevRegister(SCMD_INV_2_9, dataTemp);
                        }
                        else if(motorNum < 18) //must be 10 through 17
                        {
                            dataTemp = readDevRegister(SCMD_INV_2_9);
                            dataTemp ^= 0x01 << (motorNum - 10);
                            writeDevRegister(SCMD_INV_2_9, dataTemp);
                        }
                        else if(motorNum < 26) //must be 18 through 25
                        {
                            dataTemp = readDevRegister(SCMD_INV_2_9);
                            dataTemp ^= 0x01 << (motorNum - 18);
                            writeDevRegister(SCMD_INV_2_9, dataTemp);
                        }
                        else if(motorNum < 34) //must be 26 through 33
                        {
                            dataTemp = readDevRegister(SCMD_INV_2_9);
                            dataTemp ^= 0x01 << (motorNum - 26);
                            writeDevRegister(SCMD_INV_2_9, dataTemp);
                        }
                    }
                    else
                    {
                        if( rxBuffer[dirPtr] == 'F' )
                        {
                            motorDir = 1;
                        }
                        else
                        {
                            motorDir = -1;
                        }
                        //Check if motor is in range
                        maxMotorCount = (readDevRegister(SCMD_SLV_TOP_ADDR) - 0x50 + 1 ) * 2 + 2;
                        if( motorNum < maxMotorCount )
                        {
                            //Perform remote write
                            writeDevRegister(SCMD_MA_DRIVE + motorNum, motorDir * ((motorDrive * 127) / 100) + 128);
                        }
                        else
                        {
                            errorFlag = 2;
                            break;
                        }
                    }
                    break;
                    case 'W':
                    //Check for hex
                    if( ishex(rxBuffer[1])&&ishex(rxBuffer[2])&&ishex(rxBuffer[3])&&ishex(rxBuffer[4]) )
                    {
                        addressTemp = char2hex(rxBuffer[1]) << 4 | char2hex(rxBuffer[2]);
                        dataTemp = char2hex(rxBuffer[3]) << 4 | char2hex(rxBuffer[4]);
                        writeDevRegister(addressTemp, dataTemp);
                        USER_PORT_UartPutString("\r\n");
                    }
                    else
                    {
                        //is not all hex
                        errorFlag = 1;
                    }                    
                    break;
                    case 'R':
                    //Check for hex
                    if( ishex(rxBuffer[1])&&ishex(rxBuffer[2]) )
                    {
                        addressTemp = char2hex(rxBuffer[1]) << 4 | char2hex(rxBuffer[2]);
                        dataTemp = readDevRegister(addressTemp);
                        USER_PORT_UartPutChar(hex2char((dataTemp&0xF0) >> 4));
                        USER_PORT_UartPutChar(hex2char(dataTemp&0x0F));
                        USER_PORT_UartPutString("\r\n");
                    }
                    else
                    {
                        //is not all hex
                        errorFlag = 1;
                    }
                    break;
                    //Change baud rate
                    case 'U':
                    //Check for valid next char
                    if( ishex(rxBuffer[1]) )
                    {
                        switch(rxBuffer[1])
                        {
                            case '1':
                            USER_PORT_UartPutString("2400\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 625;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '2':
                            USER_PORT_UartPutString("4800\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 312;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '3':
                            USER_PORT_UartPutString("9600\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 156;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '4':
                            USER_PORT_UartPutString("14400\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 102;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '5':
                            USER_PORT_UartPutString("19200\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 77;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '6':
                            USER_PORT_UartPutString("38400\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 38;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '7':
                            USER_PORT_UartPutString("57600\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 25;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            case '8':
                            USER_PORT_UartPutString("115200\r\n");
                            CyDelay(400);
                            SCBCLK_UART_DIVIDER = 12;
                            SetScbConfiguration(OP_MODE_UART);
                            break;
                            default:
                            break;
                        }
                    }
                    else
                    {
                        errorFlag = 1;
                        break;
                    }
                    break;
                    case 'E': //Handle enable
                    writeDevRegister(SCMD_DRIVER_ENABLE, readDevRegister(SCMD_DRIVER_ENABLE) ^ 0x01);
                    break;
                    case 'B': //Handle bridging 
                    if( ishex(rxBuffer[1])&&ishex(rxBuffer[2])) //2 digits
                    {
                        slaveNum = char2hex(rxBuffer[2]);
                        slaveNum += 10 * char2hex(rxBuffer[1]);
                    }
                    else if( ishex(rxBuffer[1]) ) //1 digit
                    {
                        slaveNum = char2hex(rxBuffer[1]);
                    }
                    else // no digits
                    {
                        errorFlag = 1;
                        break;
                    }
                    //Motor number established; set regs accordingly
                    if( slaveNum == 0 )
                    {
                        //Write local
                        writeDevRegister(SCMD_BRIDGE, readDevRegister(SCMD_BRIDGE) ^ 0x01);
                    }
                    if( slaveNum < 9 )
                    {
                        dataTemp = readDevRegister(SCMD_BRIDGE_SLV_L);
                        dataTemp ^= 0x01 << (slaveNum - 1);
                        writeDevRegister(SCMD_BRIDGE_SLV_L, dataTemp);
                    }
                    else if( slaveNum < 17 )
                    {
                        dataTemp = readDevRegister(SCMD_BRIDGE_SLV_H);
                        dataTemp ^= 0x01 << (slaveNum - 9);
                        writeDevRegister(SCMD_BRIDGE_SLV_H, dataTemp);
                    }
                    else
                    {
                        errorFlag = 1;
                        break;
                    }
                    break;
                    //In the special case of carriage controls in the first position, don't complain
                    case '\r':
                    break;
                    case '\n':
                    break;
                    default:
                    USER_PORT_UartPutString("inv\r\n");
                    break;
                }
                if( errorFlag == 1 )
                {
                    USER_PORT_UartPutString("fmt\r\n");
                }
                if( errorFlag == 2 )
                {
                    USER_PORT_UartPutString("nom\r\n");
                }
                //Reset the buffers
                rxBufferPtr = 0;
                //Clear that char (in case it is delimiter)
                rxBuffer[rxBufferPtr] = 0;
                LED_PULSE_Write(0);
            }

        }
        if(CONFIG_BITS == 1) //SPI
        {
            uint8_t rxTemp[10];
            uint8_t rxTempPtr = 0;
            //check for interrupt, if so, toggle LED
            if(USER_PORT_SpiUartGetRxBufferSize())
            {
                rxTempPtr = 0;
                while(USER_PORT_SpiUartGetRxBufferSize())  //This reads out all rx data
                {
                    rxTemp[rxTempPtr] = USER_PORT_SpiUartReadRxData();
                    if(rxTempPtr < 9) rxTempPtr++;
                }
                
                //check first bit
                if(rxTemp[0] & 0x80)
                {
                    //Command is READ, NOT WRITE
                    addressPointer = rxTemp[0] & 0x7F;
                    USER_PORT_SpiUartWriteTxData(readDevRegister(addressPointer)); //This will be available on next read, during command bits
                }
                else
                {
                    //Command is WRITE
                    //Ok, if packet has more than 1 byte
                    if(rxTempPtr > 1)
                    {
                        addressPointer = rxTemp[0] & 0x7F;
                        writeDevRegister(addressPointer, rxTemp[1]); //Write the next byte
                    }
                }
                
                
                //USER_PORT_SpiUartWriteTxData(0xA1);
                USER_PORT_SpiUartClearRxBuffer();
                LED_R_Write(LED_R_Read()^0x01);
            }
        }
        if((CONFIG_BITS >= 0x3)&&(CONFIG_BITS <= 0xE)) //I2C
        {
            DEBUG_TIMER_Stop();
            DEBUG_TIMER_WriteCounter(0);
            DEBUG_TIMER_Start();
            /* Write complete: parse command packet */
            if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_WR_CMPLT))
            {
                LED_PULSE_Write(1);
                if(readDevRegister(SCMD_FSAFE_TIME)) //Active, clear watchdog
                {
                    //reset the counter
                    FSAFE_TIMER_Stop();
                    FSAFE_TIMER_WriteCounter(0);
                    FSAFE_TIMER_Start();
                    //Clear error message
                    clearDiagMessage(7);
                }
                
                /* Check packet length */
                if (USER_PORT_I2CSlaveGetWriteBufSize() == 2)
                {
                    //we have a address and data to write
                    addressPointer = bufferRx[0];
                    writeDevRegister(addressPointer, bufferRx[1]);
                }
                if (USER_PORT_I2CSlaveGetWriteBufSize() == 1)
                {
                    //we have a address only, expose
                    //for now, limit address
                    addressPointer = bufferRx[0];
                }
                //Count errors while clearing the status
                if( USER_PORT_I2CSlaveClearWriteStatus() & USER_PORT_I2C_SSTAT_WR_ERR ) incrementDevRegister( SCMD_I2C_WR_ERR );
                
                USER_PORT_I2CSlaveClearWriteBuf();
                LED_PULSE_Write(0);
            }
            //always expose buffer?
            /*expose buffer to master */
            bufferTx[0] = readDevRegister(addressPointer);
            if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_RD_CMPLT))
            {
                /* Clear slave read buffer and status */
                USER_PORT_I2CSlaveClearReadBuf();
                //Count errors while clearing the status
                if( USER_PORT_I2CSlaveClearReadStatus() & USER_PORT_I2C_SSTAT_RD_ERR ) incrementDevRegister( SCMD_I2C_RD_ERR );
            }
            uint32_t tempValue = DEBUG_TIMER_ReadCounter();
            if(tempValue > 0xFF) tempValue = 0xFF;
            //do 'peak hold' on SCMD_UPORT_TIME -- write 0 to reset
            if(tempValue > readDevRegister(SCMD_UPORT_TIME)) writeDevRegister(SCMD_UPORT_TIME, tempValue);
        }
        if(CONFIG_BITS == 2) //Slave
        {
            //Do slave
            /* Write complete: parse command packet */
            if (0u != (EXPANSION_PORT_I2CSlaveStatus() & EXPANSION_PORT_I2C_SSTAT_WR_CMPLT))
            {
                LED_PULSE_Write(1);
                /* Check packet length */
                if (EXPANSION_PORT_I2CSlaveGetWriteBufSize() == 2)
                {
                    //we have a address and data to write
                    addressPointer = expansionBufferRx[0];
                    writeDevRegister(addressPointer, expansionBufferRx[1]);
                }
                if (EXPANSION_PORT_I2CSlaveGetWriteBufSize() == 1)
                {
                    //we have a address only, expose
                    //for now, limit address
                    addressPointer = expansionBufferRx[0];
                }
                EXPANSION_PORT_I2CSlaveClearWriteStatus();
                EXPANSION_PORT_I2CSlaveClearWriteBuf();
                LED_PULSE_Write(0);
            }
            //always expose buffer?
            /*expose buffer to master */
            expansionBufferTx[0] = readDevRegister(addressPointer);
    
            if (0u != (EXPANSION_PORT_I2CSlaveStatus() & EXPANSION_PORT_I2C_SSTAT_RD_CMPLT))
            {
                LED_R_Write(LED_R_Read()^0x01);
                /* Clear slave read buffer and status */
                EXPANSION_PORT_I2CSlaveClearReadBuf();
                (void) EXPANSION_PORT_I2CSlaveClearReadStatus();
            }
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
                    slaveNextState = SCMDSlaveDone;
                }
                break;
            case SCMDSlaveDone:
                if(CONFIG_IN_Read() == 0)
                {
                    //CONFIG_IN went low (it shouldn't).  wait a sec and reboot
                    CyDelay(1000);
                    CySoftwareReset();
                }
                break;
            default:
                break;
            }
            slaveState = slaveNextState;
        }
        else //Do master operations
        {
            //Do master state machine
            uint8_t masterNextState = masterState;
            switch( masterState )
            {
            case SCMDMasterIdle:
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
                        masterNextState = SCMDMasterDone; //go back to this state
                    }
                }
                else
                {
                    masterNextState = SCMDMasterPollDefault; //poll again
                }
                if( readDevRegister(SCMD_SLV_POLL_CNT) > 200 )
                {
                    //Must be done
                    masterNextState = SCMDMasterDone;
                }

                CyDelay(5);
                break;
            case SCMDMasterDone:
                break;
            default:
                break;
            }
            masterState = masterNextState;
            //Get data from next slave
            
            if( masterState != SCMDMasterDone )//Needs to get data for state machine
            {
                writeDevRegister(SCMD_SLAVE_ID, ReadSlaveData(readDevRegister(SCMD_REM_ADDR), SCMD_ID));
                //writeDevRegister(SCMD_REM_DATA_RD, ReadSlaveData(readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET)));
            }
            if( masterState == SCMDMasterDone )//If the init is complete
            {
                int slaveAddri;
                for (slaveAddri = 0x50; slaveAddri <= readDevRegister(SCMD_SLV_TOP_ADDR); slaveAddri++)
                {
                    //Write drive states out to slaves
                    WriteSlaveData( slaveAddri, SCMD_MA_DRIVE, readDevRegister(SCMD_S1A_DRIVE + ((slaveAddri - 0x50) << 1 )) );
                    //WriteSlaveData( slaveAddri, SCMD_MA_DRIVE, 0x8A );
                    CyDelayUs(100);
                    WriteSlaveData( slaveAddri, SCMD_MB_DRIVE, readDevRegister(SCMD_S1B_DRIVE + ((slaveAddri - 0x50) << 1 )) );
                    CyDelayUs(100);
                }
            }
            CyDelay(1);
        }

        //Get changes -- Here, check if any registers have a changed flag that need to be serviced
        //  Check for change of failsafe time/enable register SCMD_FSAFE_TIME
        if(getChangedStatus(SCMD_FSAFE_TIME))
        {
            uint8_t tempValue = readDevRegister( SCMD_FSAFE_TIME );
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
        if(getChangedStatus(SCMD_REM_WRITE))
        {
            if( CONFIG_BITS != 0x02 )
            {
                WriteSlaveData( readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET), readDevRegister(SCMD_REM_DATA_WR) );
            }
            writeDevRegister( SCMD_REM_WRITE, 0 );
            clearChangedStatus(SCMD_REM_WRITE);
        }
        //Do writes before reads if both present
        if(getChangedStatus(SCMD_REM_READ))
        {
            if( CONFIG_BITS != 0x02 )
            {
                writeDevRegister(SCMD_REM_DATA_RD, ReadSlaveData(readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET)));
            }
            writeDevRegister( SCMD_REM_READ, 0 );
            clearChangedStatus(SCMD_REM_READ);
        } 
        if(getChangedStatus(SCMD_SLAVE_ADDR))
        {
            if( CONFIG_BITS == 0x02 ) //We are actually a slave
            {
                EXPANSION_PORT_I2CSlaveSetAddress(readDevRegister(SCMD_SLAVE_ADDR));
            }
            clearChangedStatus(SCMD_SLAVE_ADDR);
        } 
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
        //Do slave inversion and bridging
        //Don't do this stuff if you are a slave!
        if( CONFIG_BITS != 0x02 )
        {
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
        }
        //Do this one partially if you are a slave
        if(getChangedStatus(SCMD_DRIVER_ENABLE))
        {
            
            A_EN_Write(readDevRegister(SCMD_DRIVER_ENABLE) & 0x01);
            B_EN_Write(readDevRegister(SCMD_DRIVER_ENABLE) & 0x01);
            {
                if(( CONFIG_BITS != 2 )&&(readDevRegister(SCMD_SLV_TOP_ADDR) >= 0x50)) //if you are master, and there are slaves
                {
                    //send out to slaves here
                    int i;
                    for( i = 0x50; i <= readDevRegister(SCMD_SLV_TOP_ADDR); i++)
                    {
                        WriteSlaveData(i, SCMD_DRIVER_ENABLE, readDevRegister(SCMD_DRIVER_ENABLE) & 0x01 );
                    }
                }
            }
            clearChangedStatus(SCMD_DRIVER_ENABLE);
        }
        //Set outputs (always, especially if you are slave)
        PWM_1_WriteCompare(readDevRegister(SCMD_MA_DRIVE));
        PWM_2_WriteCompare(readDevRegister(SCMD_MB_DRIVE));  
    }//End of while loop
}
/* [] END OF FILE */
