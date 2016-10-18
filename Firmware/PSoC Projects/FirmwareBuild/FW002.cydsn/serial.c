/******************************************************************************
serial.c
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
#include "SCMD_config.h"
#include "serial.h"

extern volatile uint8_t CONFIG_BITS;
uint16_t SCBCLK_UART_DIVIDER = 12; // UART: 115200 kbps with OVS = 16. Required SCBCLK = 1.846 MHz, Div = 13

#define TIMEOUTCOUNTLIMIT 50000 //roughly 100ms

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

cystatus ResetExpansionScbConfigurationSlave(void)
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
    EXPANSION_PORT_I2CSlaveClearReadBuf();
    EXPANSION_PORT_I2CSlaveClearWriteBuf();
    EXPANSION_PORT_I2CSlaveClearReadStatus();
    EXPANSION_PORT_I2CSlaveClearWriteStatus();
    EXPANSION_PORT_I2CSlaveSetAddress(readDevRegister(SCMD_SLAVE_ADDR));
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

uint8 ReadSlaveData( uint8_t address, uint8_t offset )
{
    uint8  buffer[10];
    uint8_t offsetPointer[1];
    offsetPointer[0] = offset;
    uint8_t returnVar = 0;
    
    //Write an offset
    (void) EXPANSION_PORT_I2CMasterWriteBuf(address, offsetPointer, 1, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER);

    /* Waits until master completes write transfer */
    uint16_t timeoutCount = 0;
    while ((0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_WR_CMPLT)) && ( timeoutCount < TIMEOUTCOUNTLIMIT ))
    {
        timeoutCount++;
    }
    if( timeoutCount >= TIMEOUTCOUNTLIMIT )
    {
        //EXPANSION_PORT_slStatus = 0u;
        incrementDevRegister( SCMD_MST_E_ERR );
        //EXPANSION_PORT_I2C_MASTER_CLEAR_START;
        //EXPANSION_PORT_I2CMasterSendStop();
        SetExpansionScbConfigurationMaster();
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
    uint16_t timeoutCount = 0;
    while ((0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_WR_CMPLT)) && ( timeoutCount < TIMEOUTCOUNTLIMIT ))
    {
        timeoutCount++;
    }
    if( timeoutCount >= TIMEOUTCOUNTLIMIT )
    {
        //EXPANSION_PORT_slStatus = 0u;
        incrementDevRegister( SCMD_MST_E_ERR );
        //EXPANSION_PORT_I2C_MASTER_CLEAR_START;
        //EXPANSION_PORT_I2CMasterSendStop();
        SetExpansionScbConfigurationMaster();
    }

    if( EXPANSION_PORT_I2CMasterClearStatus() & EXPANSION_PORT_I2C_MSTAT_ERR_XFER ) incrementDevRegister( SCMD_MST_E_ERR );
    EXPANSION_PORT_I2CMasterClearReadBuf();
    EXPANSION_PORT_I2CMasterClearWriteBuf();

    return returnVar;
}

uint8 WriteSlave2Data( uint8_t address, uint8_t offset, uint8_t data0, uint8_t data1 )
{
    uint8  buffer[10];
    buffer[0] = offset;
    buffer[1] = data0;
    buffer[2] = data1;
    uint8_t returnVar = 0;
    
    //Write an offset
    writeDevRegister( SCMD_MST_E_STATUS, EXPANSION_PORT_I2CMasterWriteBuf(address, buffer, 3, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER));
    
    /* Waits until master completes write transfer */
    uint16_t timeoutCount = 0;
    while ((0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_WR_CMPLT)) && ( timeoutCount < TIMEOUTCOUNTLIMIT ))
    {
        timeoutCount++;
    }
    if( timeoutCount >= TIMEOUTCOUNTLIMIT )
    {
        //EXPANSION_PORT_slStatus = 0u;
        incrementDevRegister( SCMD_MST_E_ERR );
        //EXPANSION_PORT_I2C_MASTER_CLEAR_START;
        //EXPANSION_PORT_I2CMasterSendStop();
        SetExpansionScbConfigurationMaster();
    }

    if( EXPANSION_PORT_I2CMasterClearStatus() & EXPANSION_PORT_I2C_MSTAT_ERR_XFER ) incrementDevRegister( SCMD_MST_E_ERR );
    EXPANSION_PORT_I2CMasterClearReadBuf();
    EXPANSION_PORT_I2CMasterClearWriteBuf();

    return returnVar;
}