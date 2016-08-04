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
#include <project.h>
#include "devRegisters.h"


//To use software to define the config bits, uncomment this #define and set value below.
//#define USE_SW_CONFIG_BITS

volatile uint8_t CONFIG_BITS = 0x3;
    //0 -- UART
    //1 -- SPI
    //2 -- I2C slave on slave port
    //3 -- I2C @ 0x58

#define ID_WORD 0xA9 //Device ID to be programmed into memory for reads

#define SCMD_STATUS          0x00
#define SCMD_ID              0x01
#define SCMD_SLAVE_ADDR      0x02
#define SCMD_CONFIG_BITS     0x03
#define SCMD_I2C_FAULTS      0x04
#define SCMD_I2C_RD_ERR      0x05
#define SCMD_I2C_WR_ERR      0x06
#define SCMD_SPI_FAULTS      0x07
#define SCMD_UART_FAULTS     0x08
#define SCMD_UPORT_TIME	     0x09
#define SCMD_SLV_POLL_CNT    0x0A

#define SCMD_SLAVE_ID        0x10
#define SCMD_REM_ADDR        0x11
#define SCMD_REM_OFFSET      0x12
#define SCMD_REM_DATA_WR     0x13
#define SCMD_REM_DATA_RD     0x14
#define SCMD_REM_WRITE       0x15

#define SCMD_FSAFE_TIME      0x18
#define SCMD_FSAFE_FAULTS    0x19

#define SCMD_MA_DRIVE        0x20
#define SCMD_MB_DRIVE        0x21
#define SCMD_S1A_DRIVE       0x22
#define SCMD_S1B_DRIVE       0x23
#define SCMD_S2A_DRIVE       0x24
#define SCMD_S2B_DRIVE       0x25



#define USER_PORT_BUFFER_SIZE (10u)


/* Common buffers or I2C and UART */
uint8 bufferRx[USER_PORT_BUFFER_SIZE + 1u];/* RX software buffer requires one extra entry for correct operation in UART mode */
uint8 bufferTx[USER_PORT_BUFFER_SIZE]; /* TX software buffer */

const USER_PORT_I2C_INIT_STRUCT configI2C =
{
    USER_PORT_I2C_MODE_SLAVE, /* mode: slave */
    0u, /* oversampleLow: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* oversampleHigh: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* enableMedianFilter: N/A since SCB v2.0 */
    0x58u, /* slaveAddr: slave address */
    //0x2Cu,
    0xFEu, /* slaveAddrMask: signle slave address */
    0u, /* acceptAddr: disable */
    0u, /* enableWake: disable */
    0u, /* enableByteMode: disable */
    100u /* dataRate: 100 kbps */
};

const USER_PORT_UART_INIT_STRUCT configUart =
{
    USER_PORT_UART_MODE_STD, /* mode: Standard */
    USER_PORT_UART_TX_RX, /* direction: RX + TX */
    8u, /* dataBits: 8 bits */
    USER_PORT_UART_PARITY_NONE, /* parity: None */
    USER_PORT_UART_STOP_BITS_1, /* stopBits: 1 bit */
    16u, /* oversample: 16u */
    0u, /* enableIrdaLowPower: disable */
    1u, /* enableMedianFilter: enable */
    0u, /* enableRetryNack: disable */
    0u, /* enableInvertedRx: disable */
    0u, /* dropOnParityErr: disable */
    0u, /* dropOnFrameErr: disable */
    0u, /* enableWake: disable */
    USER_PORT_BUFFER_SIZE, /* rxBufferSize: software buffer 10 bytes */
    bufferRx, /* rxBuffer: RX software buffer enable */
    USER_PORT_BUFFER_SIZE, /* txBufferSize: software buffer 10 bytes */
    bufferTx, /* txBuffer: TX software buffer enable */
    0u, /* enableMultiproc: disable */
    0u, /* multiprocAcceptAddr: disable */
    0u, /* multiprocAddr: N/A for this configuration */
    0u, /* multiprocAddrMask: N/A for this configuration */
    1u, /* enableInterrupt: enable to process software buffer */
    USER_PORT_INTR_RX_NOT_EMPTY, /* rxInterruptMask: enable NOT_EMPTY for RX software buffer operations */
    0u, /* rxTriggerLevel: N/A for this configuration */
    0u, /* txInterruptMask: NOT_FULL is enabled when there is data to transmit */
    0u, /* txTriggerLevel: N/A for this configuration */
    0u, /* enableByteMode: N/A for this configuration */
    0u, /* enableCts: N/A for this configuration */
    0u, /* ctsPolarity: N/A for this configuration */
    0u, /* rtsRxTriggerLevel: N/A for this configuration */
    0u /* rtsPolarity: N/A for this configuration */
};

const USER_PORT_SPI_INIT_STRUCT configSPI =
{
    USER_PORT_SPI_SLAVE, //USER_PORT_SPI_MODE,
    USER_PORT_SPI_MODE_MOTOROLA, //USER_PORT_SPI_SUB_MODE,
    USER_PORT_SPI_SCLK_CPHA0_CPOL0, //USER_PORT_SPI_CLOCK_MODE,
    0u, //USER_PORT_SPI_OVS_FACTOR, -- Ignored for Slave Mode
    0u, //USER_PORT_SPI_MEDIAN_FILTER_ENABLE,
    0u, //USER_PORT_SPI_LATE_MISO_SAMPLE_ENABLE, -- Ignored for Slave Mode
    0u, //USER_PORT_SPI_WAKE_ENABLE, -- Ignored for Slave Mode
    8u, //USER_PORT_SPI_RX_DATA_BITS_NUM,
    8u, //USER_PORT_SPI_TX_DATA_BITS_NUM,
    USER_PORT_BITS_ORDER_MSB_FIRST, //USER_PORT_SPI_BITS_ORDER,
    USER_PORT_SPI_TRANSFER_CONTINUOUS, //USER_PORT_SPI_TRANSFER_SEPARATION, -- Ignored for Slave Mode
    USER_PORT_BUFFER_SIZE, /* rxBufferSize: software buffer 10 bytes */
    bufferRx, /* rxBuffer: RX software buffer enable */
    USER_PORT_BUFFER_SIZE, /* txBufferSize: software buffer 10 bytes */
    bufferTx, /* txBuffer: TX software buffer enable */
    1u, //(uint32) USER_PORT_SCB_IRQ_INTERNAL,
    USER_PORT_INTR_RX_NOT_EMPTY, //USER_PORT_SPI_INTR_RX_MASK,
    0u, //USER_PORT_SPI_RX_TRIGGER_LEVEL,
    0u, //USER_PORT_SPI_INTR_TX_MASK,
    0u, //USER_PORT_SPI_TX_TRIGGER_LEVEL,
    0u, //(uint8) USER_PORT_SPI_BYTE_MODE_ENABLE, -- Ignore for non- BLE
    0u, //(uint8) USER_PORT_SPI_FREE_RUN_SCLK_ENABLE, -- Ignore for non- BLE
    USER_PORT_SPI_SS_ACTIVE_LOW//(uint8) USER_PORT_SPI_SS_POLARITY -- Ignore for non- BLE
    
};

/* Buffers for expansion port */
uint8 expansionBufferRx[USER_PORT_BUFFER_SIZE + 1u];/* RX software buffer requires one extra entry for correct operation in UART mode */
uint8 expansionBufferTx[USER_PORT_BUFFER_SIZE]; /* TX software buffer */

//Config for slave
const EXPANSION_PORT_I2C_INIT_STRUCT expansionConfigI2CSlave =
{
    USER_PORT_I2C_MODE_SLAVE, /* mode: slave */
    0u, /* oversampleLow: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* oversampleHigh: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* enableMedianFilter: N/A since SCB v2.0 */
    0x4Au, /* slaveAddr: slave address */
    //0x2Cu,
    0xFEu, /* slaveAddrMask: signle slave address */
    0u, /* acceptAddr: disable */
    0u, /* enableWake: disable */
    0u, /* enableByteMode: disable */
    100u /* dataRate: 100 kbps */
};

//Config for master
const EXPANSION_PORT_I2C_INIT_STRUCT expansionConfigI2CMaster =
{
    USER_PORT_I2C_MODE_MASTER, /* mode: master */
    0u, /* oversampleLow: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* oversampleHigh: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* enableMedianFilter: N/A since SCB v2.0 */
    0x4Au, /* slaveAddr: slave address */
    //0x2Cu,
    0xFEu, /* slaveAddrMask: signle slave address */
    0u, /* acceptAddr: disable */
    0u, /* enableWake: disable */
    0u, /* enableByteMode: disable */
    100u /* dataRate: 100 kbps */
};


/* The clock divider value written into the register has to be one less from calculated */
#define SCBCLK_I2C_DIVIDER (14u) /* I2C Slave: 100 kbps Required SCBCLK = 1.6 MHz, Div = 15 */
#define SCBCLK_UART_DIVIDER (12u) /* UART: 115200 kbps with OVS = 16. Required SCBCLK = 1.846 MHz, Div = 13 */
#define SCBCLK_SPI_DIVIDER (1u) //Max for now -- Does this even matter?

/* Operation mode: I2C slave or UART */
#define OP_MODE_UART (1u)
#define OP_MODE_I2C (2u)
#define OP_MODE_SPI (3u)

cystatus SetScbConfiguration(uint32 opMode)
{
    cystatus status = CYRET_SUCCESS;
    if (OP_MODE_I2C == opMode)
    {
        USER_PORT_Stop(); /* Disable component before configuration change */
        /* Change clock divider */
        SCBCLK_Stop();
        SCBCLK_SetFractionalDividerRegister(SCBCLK_I2C_DIVIDER, 0u);
        SCBCLK_Start();
        /* Configure to I2C slave operation */
        USER_PORT_I2CSlaveInitReadBuf (bufferTx, USER_PORT_BUFFER_SIZE);
        USER_PORT_I2CSlaveInitWriteBuf(bufferRx, USER_PORT_BUFFER_SIZE);
        USER_PORT_I2CInit(&configI2C);
        USER_PORT_I2CSlaveSetAddress(0x58 + CONFIG_BITS - 0x3);
        USER_PORT_Start(); /* Enable component after configuration change */
    }
    else if (OP_MODE_UART == opMode)
    {
        USER_PORT_Stop(); /* Disable component before configuration change */
        /* Change clock divider */
        SCBCLK_Stop();
        SCBCLK_SetFractionalDividerRegister(SCBCLK_UART_DIVIDER, 0u);
        SCBCLK_Start();
        /* Configure to UART operation */
        USER_PORT_UartInit(&configUart);
        USER_PORT_Start(); /* Enable component after configuration change */
    }
    else if (OP_MODE_SPI == opMode)
    {
        USER_PORT_Stop(); /* Disable component before configuration change */
        /* Change clock divider */
        SCBCLK_Stop();
        SCBCLK_SetFractionalDividerRegister(SCBCLK_SPI_DIVIDER, 0u);
        SCBCLK_Start();
        
        /* Configure to SPI operation */
        USER_PORT_SpiInit(&configSPI);
        
        USER_PORT_Start(); /* Enable component after configuration change */
        USER_PORT_EnableInt();
        CyGlobalIntEnable; 
        
    }
    else
    {
        status = CYRET_BAD_PARAM; /* Uknowns operation mode - no actions */
    }
    return (status);
}

// ***Known good config for slave*** //
cystatus SetExpansionScbConfigurationSlave(void)
{
    cystatus status = CYRET_SUCCESS;
    EXPANSION_PORT_Stop(); /* Disable component before configuration change */
    /* Change clock divider */
    EXPANSION_SCBCLK_Stop();
    EXPANSION_SCBCLK_SetFractionalDividerRegister(SCBCLK_I2C_DIVIDER, 0u);
    EXPANSION_SCBCLK_Start();
    /* Configure to I2C slave operation */
    EXPANSION_PORT_I2CSlaveInitReadBuf (expansionBufferTx, USER_PORT_BUFFER_SIZE);
    EXPANSION_PORT_I2CSlaveInitWriteBuf(expansionBufferRx, USER_PORT_BUFFER_SIZE);
    EXPANSION_PORT_I2CInit(&expansionConfigI2CSlave);
    USER_PORT_I2CSlaveSetAddress(0x4A);
    writeDevRegister(SCMD_SLAVE_ADDR, 0x4A);
    EXPANSION_PORT_Start(); /* Enable component after configuration change */
    return (status);
}

//Expansion port config
cystatus SetExpansionScbConfigurationMaster(void)
{
    cystatus status = CYRET_SUCCESS;
    EXPANSION_PORT_Stop(); /* Disable component before configuration change */
    /* Change clock divider */
    EXPANSION_SCBCLK_Stop();
    EXPANSION_SCBCLK_SetFractionalDividerRegister(SCBCLK_I2C_DIVIDER, 0u);
    EXPANSION_SCBCLK_Start();
    /* Configure to I2C slave operation */
    EXPANSION_PORT_I2CSlaveInitReadBuf (expansionBufferTx, USER_PORT_BUFFER_SIZE);
    EXPANSION_PORT_I2CSlaveInitWriteBuf(expansionBufferRx, USER_PORT_BUFFER_SIZE);
    EXPANSION_PORT_I2CInit(&expansionConfigI2CMaster);
    EXPANSION_PORT_Start(); /* Enable component after configuration change */
    return (status);
}


uint8_t lastDiagLeds = 0;
void setDiagLeds(uint8_t input)
{
    if(input == lastDiagLeds) return;
    lastDiagLeds = input;
    int i;
    uint32_t outputPattern = 0;
    for(i = 0; i < input; i++)
    {
        outputPattern = outputPattern << 2;
        outputPattern |= 1;
    }
    ShiftReg_1_Stop();
    ShiftReg_1_WriteRegValue(outputPattern);
    ShiftReg_1_Start();

}

uint8_t errorLevelMemory[8] = {0,0,0,0,0,0,0,0}; //put flash codes within

void sendDiagMessage( void ) //send error level
{
    int i;
    for(i = 7; i >= 0; i--)
    {
        if(errorLevelMemory[i])//message persists
        {
            setDiagLeds(errorLevelMemory[i]);
            return;
        }
    }
}

void setDiagMessage( uint8_t errorLevel, uint8_t message ) //send error level
{
    errorLevelMemory[errorLevel] = message;
    sendDiagMessage();
}

void clearDiagMessage( uint8_t errorLevel ) //send error level
{
    errorLevelMemory[errorLevel] = 0;
    sendDiagMessage();
}

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

uint8_t slaveAddrEnumerator = 0x50;
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

uint8 ReadSlaveData( uint8_t address, uint8_t offset )
{
    uint8  buffer[10];
    uint8_t offsetPointer[1];
    offsetPointer[0] = offset;
    uint8_t returnVar = 0;
    
    //Write an offset
    (void) EXPANSION_PORT_I2CMasterWriteBuf(address, offsetPointer, 1, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER);

    /* Waits until master completes write transfer */
    while (0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_WR_CMPLT))
    {
    }

    //Get a byte
    (void) EXPANSION_PORT_I2CMasterReadBuf(address, buffer, 1, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER);

    /* Waits until master complete read transfer */
    while (0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_RD_CMPLT))
    {
    }

    /* Displays transfer status */
    if (0u == (EXPANSION_PORT_I2C_MSTAT_ERR_XFER & EXPANSION_PORT_I2CMasterStatus()))
    {
        /* Check packet structure */
        if ((EXPANSION_PORT_I2CMasterGetReadBufSize() >= 1 ))
        {
            returnVar = buffer[0];
        }
    }

    EXPANSION_PORT_I2CMasterClearStatus();
    EXPANSION_PORT_I2CMasterClearReadBuf();
    EXPANSION_PORT_I2CMasterClearWriteBuf();

    return returnVar;
}

uint8 WriteSlaveData( uint8_t address, uint8_t offset, uint8_t data )
{
    uint8  buffer[10];
    buffer[0] = offset;
    buffer[1] = data;
    uint8_t returnVar = 0;
    
    //Write an offset
    (void) EXPANSION_PORT_I2CMasterWriteBuf(address, buffer, 2, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER);

    /* Waits until master completes write transfer */
    while (0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_WR_CMPLT))
    {
    }

    EXPANSION_PORT_I2CMasterClearStatus();
    EXPANSION_PORT_I2CMasterClearReadBuf();
    EXPANSION_PORT_I2CMasterClearWriteBuf();

    return returnVar;
}

int main()
{
    initDevRegisters();  //Prep device registers
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
    writeDevRegister(SCMD_S1A_DRIVE, 0x81);
    writeDevRegister(SCMD_S1B_DRIVE, 0x82);
    writeDevRegister(SCMD_S2A_DRIVE, 0x83);
    writeDevRegister(SCMD_S2B_DRIVE, 0x84);
#ifndef USE_SW_CONFIG_BITS
    CONFIG_BITS = readDevRegister(SCMD_CONFIG_BITS); //Get the bits value
#endif

    DIAG_LED_CLK_Start();
    KHZ_CLK_Start();
    setDiagMessage(0, CONFIG_BITS);
    CyDelay(1500u);
    if(CONFIG_BITS != 0x02) CyDelay(1000u); //Give the slaves extra time
    
    
    MODE_Write(1);
    A_EN_Write(1);
    B_EN_Write(1);
    
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

//    isr_1_Start();      /* Initializing the ISR */
//    USER_PORT_Start();     /* Enabling the UART */

    
    uint8_t addressPointer = 0;
        
    char ch;
    uint8_t rxBufferPtr = 0;

    char rxBuffer[20];

    setDiagMessage(1, 1);    
    while(1)
    {
        if(CONFIG_BITS == 0) //UART
        {
            int8_t motorChannel = -1;
            uint8_t motorDrive = 0;
            ch = USER_PORT_UartGetChar();
            
            if(ch == 'M') //new command
            {
                rxBufferPtr = 0;

                USER_PORT_UartPutChar('M');
            }
            else if( ((ch >= '0')&&(ch <= '9'))||(ch == ' '))//valid char
            {
                rxBuffer[rxBufferPtr] = ch;
                if(rxBufferPtr < 20)//contain it
                {
                    rxBufferPtr++;
                }
                USER_PORT_UartPutChar(ch);
            }
            else if((ch == '\n')||(ch == '\r'))//end of command
            {
                rxBufferPtr--;
                USER_PORT_UartPutChar('E');
                motorChannel = rxBuffer[0] - 48;
                uint8_t mult = 1;
                while(rxBuffer[rxBufferPtr] != ' ')
                {
                    motorDrive += mult*(rxBuffer[rxBufferPtr] - 48);
                    mult = mult * 10;
                    rxBufferPtr--;
                    if(rxBufferPtr < 2)
                    {
                        rxBuffer[rxBufferPtr] = ' ';
                    }
                }
                USER_PORT_UartPutChar(motorDrive);
                switch(motorChannel)
                {
                    case 0:
                    PWM_1_WriteCompare(motorDrive * 100);
                    break;
                    case 1:
                    PWM_2_WriteCompare(motorDrive * 100);
                    break;
                    default:
                    break;
                }
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
                break;
            default:
                break;
            }
            slaveState = slaveNextState;
        }
        else
        {
            //Not slave, do master operations
            //Now do the states.
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
                masterNextState = SCMDMasterPollDefault;
                break;
            case SCMDMasterPollDefault:
                //Poll address 0x4A
                incrementDevRegister(SCMD_SLV_POLL_CNT);
                if( readDevRegister(SCMD_REM_DATA_RD) == 0x4A )//if the address came back reflected
                {
                    writeDevRegister(SCMD_REM_DATA_WR, slaveAddrEnumerator);
                    writeDevRegister(SCMD_REM_WRITE, 1);//flag the transfer
                    slaveAddrEnumerator++;
                    writeDevRegister(SCMD_SLV_POLL_CNT, 0);
                    masterNextState = SCMDMasterPollDefault; //go back to this state
                }
                if( readDevRegister(SCMD_SLV_POLL_CNT) > 200 )
                {
                    //Must be done
                    masterNextState = SCMDMasterDone;
                }
                CyDelay(10);
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
                writeDevRegister(SCMD_REM_DATA_RD, ReadSlaveData(readDevRegister(SCMD_REM_ADDR), readDevRegister(SCMD_REM_OFFSET)));
            }
            if( masterState == SCMDMasterDone )//If the init is complete
            {
                int slaveAddri;
                for (slaveAddri = 0x50; slaveAddri < (slaveAddrEnumerator - 1); slaveAddri++)
                {
                    //Write drive states out to slaves
                    WriteSlaveData( slaveAddri, SCMD_MA_DRIVE, readDevRegister(SCMD_S1A_DRIVE + ((slaveAddri - 0x50) << 1 )) );
                    //WriteSlaveData( slaveAddri, SCMD_MA_DRIVE, 0x8A );
                    CyDelay(1);
                    WriteSlaveData( slaveAddri, SCMD_MB_DRIVE, readDevRegister(SCMD_S1B_DRIVE + ((slaveAddri - 0x50) << 1 )) );
                    CyDelay(1);
                }
            }
            CyDelay(10);
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
        if(getChangedStatus(SCMD_SLAVE_ADDR))
        {
            if( CONFIG_BITS == 0x02 ) //We are actually a slave
            {
                EXPANSION_PORT_I2CSlaveSetAddress(readDevRegister(SCMD_SLAVE_ADDR));
            }
            clearChangedStatus(SCMD_SLAVE_ADDR);
        } 
        //Set outputs
        
        PWM_1_WriteCompare(readDevRegister(SCMD_MA_DRIVE));
        PWM_2_WriteCompare(readDevRegister(SCMD_MB_DRIVE));  
        
    }//End of while loop

}






/* [] END OF FILE */
