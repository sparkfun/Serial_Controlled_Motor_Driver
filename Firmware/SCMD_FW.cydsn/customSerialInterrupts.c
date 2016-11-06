/******************************************************************************
customSerialInterrupts.c
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
//#include "project.h"
#include "SCMD_config.h"
#include "devRegisters.h"
#include "customSerialInterrupts.h"
#include "USER_PORT_PVT.h"
#include "USER_PORT_SPI_UART_PVT.h"
#include "serial.h"
#include "diagLEDS.h"
#include "charMath.h"
#include "registerHandlers.h"

//Variables for SPI
uint8_t rxTemp[10];
uint8_t rxTempPtr = 0;
uint8_t masterAddressPointer = 0;

//Variables for I2C
uint8_t addressPointer = 0;
uint8_t expansionAddressPointer = 0;
extern const uint16_t SCBCLK_I2C_DIVIDER_TABLE[4];

//Variables for UART
static uint8_t rxBufferPtr = 0;
static char rxBuffer[20];
extern const uint16_t SCBCLK_UART_DIVIDER_TABLE[8];

//****************************************************************************//
//
//  User Port in SPI mode -- Altered auto-generated ISR plus data handler
//
//****************************************************************************//
CY_ISR(custom_USER_PORT_SPI_UART_ISR)
{
#if(USER_PORT_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
    uint32 dataRx;
#endif /* (USER_PORT_INTERNAL_RX_SW_BUFFER_CONST) */

#if(USER_PORT_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (USER_PORT_INTERNAL_TX_SW_BUFFER_CONST) */

    if(USER_PORT_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        USER_PORT_ClearSpiExtClkInterruptSource(USER_PORT_INTR_SPI_EC_WAKE_UP);
    }

    if(USER_PORT_CHECK_RX_SW_BUFFER)
    {
        if(USER_PORT_CHECK_INTR_RX_MASKED(USER_PORT_INTR_RX_NOT_EMPTY))
        {
            while(0u != USER_PORT_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = USER_PORT_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (USER_PORT_rxBufferHead + 1u);

                /* Adjust local head index */
                if(USER_PORT_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == USER_PORT_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    USER_PORT_rxBufferOverflow = (uint8) USER_PORT_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    USER_PORT_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    USER_PORT_rxBufferHead = locHead;
                    
                }
                parseSPI();
            }

            USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_RX_NOT_EMPTY);
            
        }
    }

    if(USER_PORT_CHECK_TX_SW_BUFFER)
    {
        if(USER_PORT_CHECK_INTR_TX_MASKED(USER_PORT_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(USER_PORT_SPI_UART_FIFO_SIZE != USER_PORT_GET_TX_FIFO_ENTRIES)
            {
                /* Check for room in TX software buffer */
                if(USER_PORT_txBufferHead != USER_PORT_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (USER_PORT_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(USER_PORT_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    USER_PORT_TX_FIFO_WR_REG = USER_PORT_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    USER_PORT_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    USER_PORT_DISABLE_INTR_TX(USER_PORT_INTR_TX_NOT_FULL);
                    break;
                }
            }

            USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_TX_NOT_FULL);
        }
    }
}

void parseSPI( void )
{
    if(USER_PORT_SpiUartGetRxBufferSize())
    {
        LED_PULSE_Write(1);
        if(readDevRegister( SCMD_FSAFE_TIME )) //Active, clear watchdog
        {
            //reset the counter
            FSAFE_TIMER_Stop();
            FSAFE_TIMER_WriteCounter(0);
            FSAFE_TIMER_Start();
            //Clear error message
            clearDiagMessage(7);
        }
        //Data exists to be read.
        while(USER_PORT_SpiUartGetRxBufferSize())  //This reads out all rx data
        {
            rxTemp[rxTempPtr] = USER_PORT_SpiUartReadRxData();
            if(rxTempPtr < 9) rxTempPtr++;
        }
        //Now the temp buffer contains whatever was in it, plus all new bytes

        //check first bit
        if(rxTemp[0] & 0x80)
        {
            //Command is READ, NOT WRITE
            masterAddressPointer = rxTemp[0] & 0x7F;
            USER_PORT_SpiUartClearTxBuffer();
            volatile uint8_t reg = readDevRegister( masterAddressPointer );
            USER_PORT_SpiUartWriteTxData(reg); //This will be available on next read, during command bits
            //We should check that the buffer doesn't have unused data and count!
            //But we won't...
            USER_PORT_SpiUartClearRxBuffer();
            rxTempPtr = 0;
            
        }
        else
        {
            //Command is WRITE
            switch( rxTempPtr )
            {
                case 2: //Buffer has command and data
                    masterAddressPointer = rxTemp[0] & 0x7F;
                    writeDevRegister( masterAddressPointer, rxTemp[1] ); //Write the next byte
                    USER_PORT_SpiUartClearRxBuffer();
                    rxTempPtr = 0;
                break;
                case 1:
                    //Not enough data, do nothing and wait for next loop.
                break;
                case 0:
                default:
                    //Too much data.  Dump data, reset, and keep a count
                    incrementDevRegister( SCMD_U_BUF_DUMPED );
                    USER_PORT_SpiUartClearRxBuffer();
                    rxTempPtr = 0;                    
                break;
            }
        }
        
        LED_PULSE_Write(0);

    }

}

//****************************************************************************//
//
//  User Port in UART mode
//
//****************************************************************************//

void parseUART( void )
{
    //General:
    //Save data (echo it) until we get a delimiter
    //Parse data and take action
    char ch = USER_PORT_UartGetChar();
    if(ch)
    {
        //Show data on LED
        LED_PULSE_Write(1);
        if(readDevRegister( SCMD_FSAFE_TIME )) //Active, clear watchdog
        {
            //reset the counter
            FSAFE_TIMER_Stop();
            FSAFE_TIMER_WriteCounter(0);
            FSAFE_TIMER_Start();
            //Clear error message
            clearDiagMessage(7);
        }
        //Save data
        if(rxBufferPtr < 19)
        {
			if(((rxBufferPtr == 0)&&(ch != '\n'))||(rxBufferPtr > 0))
			{
				rxBuffer[rxBufferPtr] = ch;
				rxBufferPtr++;
			}
        }
        else
        {
            //Overwrite last
            rxBuffer[rxBufferPtr - 1] = ch;
            USER_PORT_UartPutString( "\r\novf" );
        }
        //Echo all but nl and cr
        if((ch != '\n')&&(ch != '\r')) USER_PORT_UartPutChar(ch);
        //USER_PORT_UartPutChar(ch);
        LED_PULSE_Write(0);
    }
    if((rxBuffer[rxBufferPtr - 1] == '\n')||(rxBuffer[rxBufferPtr - 1] == '\r'))//Delimiter found
    //if(rxBuffer[rxBufferPtr - 1] == '\n')//Delimiter found, don't delimit \r
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
            if(( rxBuffer[dirPtr] != 'F' )&&( rxBuffer[dirPtr] != 'R' )&&( rxBuffer[dirPtr] != 'I' )&&( rxBuffer[dirPtr] != 'C' )) // not here
            {
                dirPtr++;
            }
            if(( rxBuffer[dirPtr] != 'F' )&&( rxBuffer[dirPtr] != 'R' )&&( rxBuffer[dirPtr] != 'I' )&&( rxBuffer[dirPtr] != 'C' )) // not here
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
            //Check for polarity switching on
            if( rxBuffer[dirPtr] == 'I' )
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
                    writeDevRegister(SCMD_MOTOR_A_INVERT, 0x01);
                }
                else if(motorNum == 1)
                {
                    writeDevRegister(SCMD_MOTOR_B_INVERT, 0x01);
                }
                else if(motorNum < 10) //must be 2 through 9
                {
                    dataTemp = readDevRegister(SCMD_INV_2_9);
                    dataTemp |= 0x01 << (motorNum - 2);
                    writeDevRegister(SCMD_INV_2_9, dataTemp);
                }
                else if(motorNum < 18) //must be 10 through 17
                {
                    dataTemp = readDevRegister(SCMD_INV_10_17);
                    dataTemp |= 0x01 << (motorNum - 10);
                    writeDevRegister(SCMD_INV_10_17, dataTemp);
                }
                else if(motorNum < 26) //must be 18 through 25
                {
                    dataTemp = readDevRegister(SCMD_INV_18_25);
                    dataTemp |= 0x01 << (motorNum - 18);
                    writeDevRegister(SCMD_INV_18_25, dataTemp);
                }
                else if(motorNum < 34) //must be 26 through 33
                {
                    dataTemp = readDevRegister(SCMD_INV_26_33);
                    dataTemp |= 0x01 << (motorNum - 26);
                    writeDevRegister(SCMD_INV_26_33, dataTemp);
                }
            }
            //Check for polarity switching
            else if( rxBuffer[dirPtr] == 'C' )
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
                    writeDevRegister(SCMD_MOTOR_A_INVERT, 0x00);
                }
                else if(motorNum == 1)
                {
                    writeDevRegister(SCMD_MOTOR_B_INVERT, 0x00);
                }
                else if(motorNum < 10) //must be 2 through 9
                {
                    dataTemp = readDevRegister(SCMD_INV_2_9);
                    dataTemp &= ~(0x01 << (motorNum - 2));
                    writeDevRegister(SCMD_INV_2_9, dataTemp);
                }
                else if(motorNum < 18) //must be 10 through 17
                {
                    dataTemp = readDevRegister(SCMD_INV_10_17);
                    dataTemp &= ~(0x01 << (motorNum - 10));
                    writeDevRegister(SCMD_INV_10_17, dataTemp);
                }
                else if(motorNum < 26) //must be 18 through 25
                {
                    dataTemp = readDevRegister(SCMD_INV_18_25);
                    dataTemp &= ~(0x01 << (motorNum - 18));
                    writeDevRegister(SCMD_INV_18_25, dataTemp);
                }
                else if(motorNum < 34) //must be 26 through 33
                {
                    dataTemp = readDevRegister(SCMD_INV_26_33);
                    dataTemp &= ~(0x01 << (motorNum - 26));
                    writeDevRegister(SCMD_INV_26_33, dataTemp);
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
                maxMotorCount = (readDevRegister( SCMD_SLV_TOP_ADDR ) - 0x50 + 1 ) * 2 + 2;
                if( motorNum < maxMotorCount )
                {
                    //Perform remote write
                    writeDevRegister( SCMD_MA_DRIVE + motorNum, motorDir * ((motorDrive * 127) / 100) + 128 );
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
				saveKeysFullAccess(); //allow writes to registers

                switch(rxBuffer[1])
                {
                    case '0':
                    USER_PORT_UartPutString("2400\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 0);
                    break;
                    case '1':
                    USER_PORT_UartPutString("4800\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 1);
                    break;
                    case '2':
                    USER_PORT_UartPutString("9600\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 2);
                    break;
                    case '3':
                    USER_PORT_UartPutString("14400\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 3);
                    break;
                    case '4':
                    USER_PORT_UartPutString("19200\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 4);
                    break;
                    case '5':
                    USER_PORT_UartPutString("38400\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 5);
                    break;
                    case '6':
                    USER_PORT_UartPutString("57600\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 6);
                    break;
                    case '7':
                    USER_PORT_UartPutString("115200\r\n");
                    writeDevRegister(SCMD_U_BUS_UART_BAUD, 7);
                    break;
                    default:
                    break;
                }
				
				restoreKeys();//Replace previous keys
				
                //Cause the update, but give time to print the new speed at the old speed:
                CyDelay(400);
                calcUserDivider(readDevRegister(SCMD_CONFIG_BITS));
                SetScbConfiguration(OP_MODE_UART);
            }
            else
            {
                errorFlag = 1;
                break;
            }
            break;
            case 'E': //Handle enable
            writeDevRegister(SCMD_DRIVER_ENABLE, 0x01);
            break;
            case 'D': //Handle disable
            writeDevRegister(SCMD_DRIVER_ENABLE, 0x00);
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
                writeDevRegister(SCMD_BRIDGE, 0x01);
            }
            if( slaveNum < 9 )
            {
                dataTemp = readDevRegister(SCMD_BRIDGE_SLV_L);
                dataTemp |= 0x01 << (slaveNum - 1);
                writeDevRegister(SCMD_BRIDGE_SLV_L, dataTemp);
            }
            else if( slaveNum < 17 )
            {
                dataTemp = readDevRegister(SCMD_BRIDGE_SLV_H);
                dataTemp |= 0x01 << (slaveNum - 9);
                writeDevRegister(SCMD_BRIDGE_SLV_H, dataTemp);
            }
            else
            {
                errorFlag = 1;
                break;
            }
            break;
            case 'N': //Handle unbridging 
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
                writeDevRegister(SCMD_BRIDGE, 0x00);
            }
            if( slaveNum < 9 )
            {
                dataTemp = readDevRegister(SCMD_BRIDGE_SLV_L);
                dataTemp &= ~(0x01 << (slaveNum - 1));
                writeDevRegister(SCMD_BRIDGE_SLV_L, dataTemp);
            }
            else if( slaveNum < 17 )
            {
                dataTemp = readDevRegister(SCMD_BRIDGE_SLV_H);
                dataTemp &= ~(0x01 << (slaveNum - 9));
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
            case '\n':
            //writeDevRegister(0x6F, 0x10);
            // USER_PORT_UartPutString("\r\n");
            break;
            default:
            USER_PORT_UartPutString("\r\ninv");
            break;
        }
        if( errorFlag == 1 )
        {
            USER_PORT_UartPutString("\r\nfmt");
        }
        if( errorFlag == 2 )
        {
            USER_PORT_UartPutString("\r\nnom");
        }
        //Reset the buffers
        rxBufferPtr = 0;
        //Clear that char (in case it is delimiter)
        rxBuffer[rxBufferPtr] = 0;
        LED_PULSE_Write(0);
		USER_PORT_UartPutString("\r\n");
		USER_PORT_UartPutString(">");
    }

}

//****************************************************************************//
//
//  User Port in I2C mode pseudo-ISR
//
//  This is called from an interrupt, and periodically by the main
//
//****************************************************************************//

void parseI2C( void )
{
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
        else if (USER_PORT_I2CSlaveGetWriteBufSize() == 1)
        {
            //we have a address only, expose
            //for now, limit address
            addressPointer = bufferRx[0];
        }
        else if (USER_PORT_I2CSlaveGetWriteBufSize() > 2)
        {
            //Something was wrong with the ammount of data in the buffer.  It will be dumped, but keep a count
            incrementDevRegister( SCMD_U_BUF_DUMPED );
        }
        //Count errors while clearing the status
        if( USER_PORT_I2CSlaveClearWriteStatus() & USER_PORT_I2C_SSTAT_WR_ERR ) incrementDevRegister( SCMD_U_I2C_WR_ERR );
        
        USER_PORT_I2CSlaveClearWriteBuf();
        bufferTx[0] = readDevRegister(addressPointer);
        LED_PULSE_Write(0);
    }
    //expose data
    if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_RD_CMPLT))
    {
        /* Clear slave read buffer and status */
        USER_PORT_I2CSlaveClearReadBuf();
        //Count errors while clearing the status
        if( USER_PORT_I2CSlaveClearReadStatus() & USER_PORT_I2C_SSTAT_RD_ERR ) incrementDevRegister( SCMD_U_I2C_RD_ERR );
    }

}

//****************************************************************************//
//
//  Expansion Port pseudo-ISR
//
//  This is called from an interrupt, and periodically by the main
//
//****************************************************************************//

void parseSlaveI2C( void )
{
    //DEBUG_TIMER_Stop();
    //DEBUG_TIMER_WriteCounter(0);
    //DEBUG_TIMER_Start(); 
    /* Write complete: parse command packet */
    if ((0u != (EXPANSION_PORT_I2CSlaveStatus() & EXPANSION_PORT_I2C_SSTAT_WR_CMPLT))&&(0u == (EXPANSION_PORT_I2CSlaveStatus() & EXPANSION_PORT_I2C_SSTAT_WR_BUSY)))
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
        switch( EXPANSION_PORT_I2CSlaveGetWriteBufSize() )
        {
            case 3:
                //we have a address and 2 data to write
                expansionAddressPointer = expansionBufferRx[0];
                writeDevRegister(expansionAddressPointer, expansionBufferRx[1]);
                writeDevRegister(expansionAddressPointer + 1, expansionBufferRx[2]);
            break;
            case 2:
                //we have a address and data to write
                expansionAddressPointer = expansionBufferRx[0];
                writeDevRegister(expansionAddressPointer, expansionBufferRx[1]);
            break;
            case 1:
                //we have a address only, expose
                //for now, limit address
                expansionAddressPointer = expansionBufferRx[0];
            break;
            default:
            break;
        }
        //Count errors while clearing the status
        if( EXPANSION_PORT_I2CSlaveClearWriteStatus() & EXPANSION_PORT_I2C_SSTAT_WR_ERR ) incrementDevRegister( SCMD_E_I2C_WR_ERR );
        EXPANSION_PORT_I2CSlaveClearWriteBuf();
        LED_PULSE_Write(0);

    }
    //always expose buffer?
    /*expose buffer to master */
    expansionBufferTx[0] = readDevRegister(expansionAddressPointer);

    if (0u != (EXPANSION_PORT_I2CSlaveStatus() & EXPANSION_PORT_I2C_SSTAT_RD_CMPLT))
    {
        /* Clear slave read buffer and status */
        EXPANSION_PORT_I2CSlaveClearReadBuf();
        //Count errors while clearing the status
        if( EXPANSION_PORT_I2CSlaveClearReadStatus() & EXPANSION_PORT_I2C_SSTAT_RD_ERR ) incrementDevRegister( SCMD_E_I2C_RD_ERR );
    }
}


/* [] END OF FILE */