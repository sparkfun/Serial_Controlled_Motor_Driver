/******************************************************************************
serial.c
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
#include <project.h>
#include "devRegisters.h"
#include "SCMD_config.h"
#include "serial.h"
#include "registerHandlers.h"

extern volatile uint8_t CONFIG_BITS;

/***** The clock divider value written into the register has to be one less from calculated *****/

//SCBCLK constants for USER UART
// UART: 115200 kbps with OVS = 16. Required SCBCLK = 1.846 MHz, Div = 25
const uint16_t SCBCLK_UART_DIVIDER_TABLE[8] = {
625, //2400
312, //4800
156, //9600
102, //14400
77, //19200
38, //38400
25, //57600
12 //115200
};

//SCBCLK constants for EXPANSION I2C
// source rate / divider / oversampling h+l = bit rate
const uint16_t SCBCLK_I2C_DIVIDER_TABLE[4] = {
29, //50 kHz
14, //100 kHz
2, //400 kHz
2, //400 kHz
};

//Timeout for writes and reads to the slaves
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
    8u, /* oversampleLow: N/A for slave, SCBCLK determines maximum data rate*/
    8u, /* oversampleHigh: N/A for slave, SCBCLK determines maximum data rate*/
    0u, /* enableMedianFilter: N/A since SCB v2.0 */
    0x4Au, /* slaveAddr: slave address */
    //0x2Cu,
    0xFEu, /* slaveAddrMask: signle slave address */
    0u, /* acceptAddr: disable */
    0u, /* enableWake: disable */
    0u, /* enableByteMode: disable */
    400u /* dataRate: 100 kbps */
};

cystatus SetScbConfiguration(uint32 opMode)
{
    //Do I need to enable interrupt for all cases????????????????????????????????????????
    
    cystatus status = CYRET_SUCCESS;
    if(OP_MODE_I2C == opMode)
    {
        USER_PORT_Stop(); /* Disable component before configuration change */
        USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_RX_ALL);
        USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_TX_ALL);
        USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_ALL);
        USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_ALL);
        
        /* Change clock divider */
        SCBCLK_Stop();
        SCBCLK_SetFractionalDividerRegister( (readDevRegister( SCMD_U_PORT_CLKDIV_U ) << 8) | readDevRegister( SCMD_U_PORT_CLKDIV_L ), SCMD_U_PORT_CLKDIV_CTRL );
        SCBCLK_Start();
        /* Configure to I2C slave operation */
        USER_PORT_I2CSlaveInitReadBuf ( bufferTx, USER_PORT_BUFFER_SIZE );
        USER_PORT_I2CSlaveInitWriteBuf( bufferRx, USER_PORT_BUFFER_SIZE );
        USER_PORT_I2CInit( &configI2C );
        USER_PORT_I2CSlaveSetAddress( 0x58 + CONFIG_BITS - 0x3 );
        USER_PORT_Start(); /* Enable component after configuration change */
    }
    else if(OP_MODE_UART == opMode)
    {
        USER_PORT_Stop(); /* Disable component before configuration change */
        USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_RX_ALL);
        USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_TX_ALL);
        USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_ALL);
        USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_ALL);
        
        /* Change clock divider */
        SCBCLK_Stop();
        SCBCLK_SetFractionalDividerRegister( (readDevRegister( SCMD_U_PORT_CLKDIV_U ) << 8) | readDevRegister( SCMD_U_PORT_CLKDIV_L ), SCMD_U_PORT_CLKDIV_CTRL );
        SCBCLK_Start();
        /* Configure to UART operation */
        USER_PORT_UartInit( &configUart );
        USER_PORT_Start(); /* Enable component after configuration change */
    }
    else if(OP_MODE_SPI == opMode)
    {
        USER_PORT_Stop(); /* Disable component before configuration change */
        USER_PORT_SetRxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_SetTxInterruptMode(USER_PORT_NO_INTR_SOURCES);
        USER_PORT_ClearTxInterruptSource(USER_PORT_INTR_RX_ALL);
        USER_PORT_ClearRxInterruptSource(USER_PORT_INTR_TX_ALL);
        USER_PORT_ClearSlaveInterruptSource(USER_PORT_INTR_SLAVE_ALL);
        USER_PORT_ClearMasterInterruptSource(USER_PORT_INTR_MASTER_ALL);
        
        /* Change clock divider */
        SCBCLK_Stop();
        SCBCLK_SetFractionalDividerRegister( (readDevRegister( SCMD_U_PORT_CLKDIV_U ) << 8) | readDevRegister( SCMD_U_PORT_CLKDIV_L ), SCMD_U_PORT_CLKDIV_CTRL );
        SCBCLK_Start();
        
        /* Configure to SPI operation */
        USER_PORT_SpiInit( &configSPI );
        
        USER_PORT_Start(); /* Enable component after configuration change */
        USER_PORT_EnableInt();
        CyGlobalIntEnable; 
        
    }
    else
    {
        status = CYRET_BAD_PARAM; /* Uknowns operation mode - no actions */
    }
    return(status);
}

// ***Known good config for slave*** //
cystatus SetExpansionScbConfigurationSlave(void)
{
    cystatus status = CYRET_SUCCESS;
    EXPANSION_PORT_Stop(); /* Disable component before configuration change */
    EXPANSION_PORT_SetRxInterruptMode(EXPANSION_PORT_NO_INTR_SOURCES);
    EXPANSION_PORT_SetTxInterruptMode(EXPANSION_PORT_NO_INTR_SOURCES);
    EXPANSION_PORT_ClearTxInterruptSource(EXPANSION_PORT_INTR_RX_ALL);
    EXPANSION_PORT_ClearRxInterruptSource(EXPANSION_PORT_INTR_TX_ALL);
    EXPANSION_PORT_ClearSlaveInterruptSource(EXPANSION_PORT_INTR_SLAVE_ALL);
    EXPANSION_PORT_ClearMasterInterruptSource(EXPANSION_PORT_INTR_MASTER_ALL);
    
    /* Change clock divider */
    EXPANSION_SCBCLK_Stop();
    EXPANSION_SCBCLK_SetFractionalDividerRegister( (readDevRegister( SCMD_E_PORT_CLKDIV_U ) << 8) | readDevRegister( SCMD_E_PORT_CLKDIV_L ), SCMD_E_PORT_CLKDIV_CTRL );
    EXPANSION_SCBCLK_Start();
    /* Configure to I2C slave operation */
    EXPANSION_PORT_I2CSlaveInitReadBuf ( expansionBufferTx, USER_PORT_BUFFER_SIZE );
    EXPANSION_PORT_I2CSlaveInitWriteBuf( expansionBufferRx, USER_PORT_BUFFER_SIZE );
    EXPANSION_PORT_I2CInit( &expansionConfigI2CSlave );
    EXPANSION_PORT_I2CSlaveSetAddress( readDevRegister( SCMD_SLAVE_ADDR ) );
    //writeDevRegister(SCMD_SLAVE_ADDR, 0x10);
    EXPANSION_PORT_I2CSlaveClearReadBuf();
    EXPANSION_PORT_I2CSlaveClearWriteBuf();
    EXPANSION_PORT_I2CSlaveClearReadStatus();
    EXPANSION_PORT_I2CSlaveClearWriteStatus();
    EXPANSION_PORT_Start(); /* Enable component after configuration change */
    return(status);
}

//Expansion port config
cystatus SetExpansionScbConfigurationMaster(void)
{
    cystatus status = CYRET_SUCCESS;
    /* Clear interrupt sources as they are not automatically cleared after SCB is disabled */
    EXPANSION_PORT_Stop(); /* Disable component before configuration change */
    EXPANSION_PORT_SetRxInterruptMode(EXPANSION_PORT_NO_INTR_SOURCES);
    EXPANSION_PORT_SetTxInterruptMode(EXPANSION_PORT_NO_INTR_SOURCES);
    EXPANSION_PORT_ClearTxInterruptSource(EXPANSION_PORT_INTR_RX_ALL);
    EXPANSION_PORT_ClearRxInterruptSource(EXPANSION_PORT_INTR_TX_ALL);
    EXPANSION_PORT_ClearSlaveInterruptSource(EXPANSION_PORT_INTR_SLAVE_ALL);
    EXPANSION_PORT_ClearMasterInterruptSource(EXPANSION_PORT_INTR_MASTER_ALL);
    
    /* Change clock divider */
    EXPANSION_SCBCLK_Stop();
    EXPANSION_SCBCLK_SetFractionalDividerRegister( (readDevRegister( SCMD_E_PORT_CLKDIV_U ) << 8) | readDevRegister( SCMD_E_PORT_CLKDIV_L ), SCMD_E_PORT_CLKDIV_CTRL );
    EXPANSION_SCBCLK_Start();
    /* Configure to I2C slave operation */
    EXPANSION_PORT_I2CSlaveInitReadBuf ( expansionBufferTx, USER_PORT_BUFFER_SIZE );
    EXPANSION_PORT_I2CSlaveInitWriteBuf( expansionBufferRx, USER_PORT_BUFFER_SIZE );
    EXPANSION_PORT_I2CInit( &expansionConfigI2CMaster );
    EXPANSION_PORT_I2CMasterClearReadBuf();
    EXPANSION_PORT_I2CMasterClearWriteBuf();
    EXPANSION_PORT_I2CMasterClearStatus();
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
    EXPANSION_PORT_I2CMasterWriteBuf( address, offsetPointer, 1, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER );

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
        //SetExpansionScbConfigurationMaster();
        //initExpansionSerial(CONFIG_BITS); 
    }

    //Get a byte
    EXPANSION_PORT_I2CMasterReadBuf(address, buffer, 1, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER);

    /* Waits until master complete read transfer */
    while ((0u == (EXPANSION_PORT_I2CMasterStatus() & EXPANSION_PORT_I2C_MSTAT_RD_CMPLT)) && ( timeoutCount < TIMEOUTCOUNTLIMIT ))
    {
        timeoutCount++;
    }
    if( timeoutCount >= TIMEOUTCOUNTLIMIT )
    {
        //EXPANSION_PORT_slStatus = 0u;
        incrementDevRegister( SCMD_MST_E_ERR );
        //EXPANSION_PORT_I2C_MASTER_CLEAR_START;
        //EXPANSION_PORT_I2CMasterSendStop();
        //SetExpansionScbConfigurationMaster();
        //initExpansionSerial( CONFIG_BITS );
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
        //SetExpansionScbConfigurationMaster();
        //initExpansionSerial(CONFIG_BITS);
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
    writeDevRegister( SCMD_MST_E_STATUS, EXPANSION_PORT_I2CMasterWriteBuf( address, buffer, 3, EXPANSION_PORT_I2C_MODE_COMPLETE_XFER ) );
    
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
        //SetExpansionScbConfigurationMaster();
        //initExpansionSerial(CONFIG_BITS); 
    }

    if( EXPANSION_PORT_I2CMasterClearStatus() & EXPANSION_PORT_I2C_MSTAT_ERR_XFER ) incrementDevRegister( SCMD_MST_E_ERR );
    EXPANSION_PORT_I2CMasterClearReadBuf();
    EXPANSION_PORT_I2CMasterClearWriteBuf();

    return returnVar;
}

//****************************************************************************//
//
//  Clock divider calculators
//
//****************************************************************************//
void calcUserDivider( uint8_t configBitsVar )
{
    saveKeysFullAccess(); //allow writes to registers
    
    //Config USER_PORT
    if((configBitsVar == 0) || (configBitsVar == 0x0D) || (configBitsVar == 0x0E)) //UART
    {
        writeDevRegister(SCMD_U_PORT_CLKDIV_U, (SCBCLK_UART_DIVIDER_TABLE[readDevRegister(SCMD_U_BUS_UART_BAUD) & 0x07] & 0xFF00) >> 8);
        writeDevRegister(SCMD_U_PORT_CLKDIV_L, SCBCLK_UART_DIVIDER_TABLE[readDevRegister(SCMD_U_BUS_UART_BAUD) & 0x07] & 0x00FF);
        writeDevRegister(SCMD_U_PORT_CLKDIV_CTRL, 0);
    }
    else if(configBitsVar == 1) //SPI
    {
        writeDevRegister(SCMD_U_PORT_CLKDIV_U, 0);
        writeDevRegister(SCMD_U_PORT_CLKDIV_L, 1);
        writeDevRegister(SCMD_U_PORT_CLKDIV_CTRL, 0);
    }
    else if((configBitsVar >= 0x3)&&(configBitsVar <= 0xC)) //I2C
    {
        writeDevRegister(SCMD_U_PORT_CLKDIV_U, 0);
        writeDevRegister(SCMD_U_PORT_CLKDIV_L, 1);
        writeDevRegister(SCMD_U_PORT_CLKDIV_CTRL, 0);
    }
    
    //Clear all flags
    clearChangedStatus( SCMD_U_PORT_CLKDIV_U );
    clearChangedStatus( SCMD_U_PORT_CLKDIV_L );
    clearChangedStatus( SCMD_U_PORT_CLKDIV_CTRL );
    
    restoreKeys();//Replace previous keys
}

//This configures 
void calcExpansionDivider( uint8_t configBitsVar )
{
    saveKeysFullAccess(); //allow writes to registers
 
    //Config EXPANSION_PORT
    if(configBitsVar == 2) //Slave
    {
        writeDevRegister(SCMD_E_PORT_CLKDIV_U, 0);
        writeDevRegister(SCMD_E_PORT_CLKDIV_L, 1);
        writeDevRegister(SCMD_E_PORT_CLKDIV_CTRL, 0);
    }
    else
    {
        writeDevRegister(SCMD_E_PORT_CLKDIV_U, 0);
        writeDevRegister(SCMD_E_PORT_CLKDIV_L, SCBCLK_I2C_DIVIDER_TABLE[readDevRegister(SCMD_E_BUS_SPEED) & 0x03]);
        writeDevRegister(SCMD_E_PORT_CLKDIV_CTRL, 0);
    }
    //Clear flags
    clearChangedStatus( SCMD_E_PORT_CLKDIV_U );
    clearChangedStatus( SCMD_E_PORT_CLKDIV_L );
    clearChangedStatus( SCMD_E_PORT_CLKDIV_CTRL );

    restoreKeys();//Replace previous keys

}

extern void custom_USER_PORT_SPI_UART_ISR( void );
extern void parseI2C( void );
//****************************************************************************//
//
//  init functions
//
//****************************************************************************//
void initUserSerial( uint8_t configBitsVar ) //Pass configuration word
{
    //Config USER_PORT
    if((configBitsVar == 0) || (configBitsVar == 0x0D) || (configBitsVar == 0x0E)) //UART
    {
        SetScbConfiguration( OP_MODE_UART );
    }
    else if(configBitsVar == 1) //SPI
    {
        SetScbConfiguration( OP_MODE_SPI );
        USER_PORT_SpiUartClearRxBuffer();
        
        //rerouting interrupts
        //USER_PORT_SetCustomInterruptHandler(parseSPI);
        
        //overwrite custom ISR to vector table
        CyIntSetVector( USER_PORT_ISR_NUMBER, &custom_USER_PORT_SPI_UART_ISR );
    }
    else if((configBitsVar >= 0x3)&&(configBitsVar <= 0xE)) //I2C
    {
        SetScbConfiguration( OP_MODE_I2C );
        USER_PORT_SetCustomInterruptHandler( parseI2C );
    }
}

extern void parseSlaveI2C( void );
void initExpansionSerial( uint8_t configBitsVar ) //Pass configuration word
{
    //Config EXPANSION_PORT
    if(configBitsVar == 2) //Slave
    {
        SetExpansionScbConfigurationSlave();
        EXPANSION_PORT_SetCustomInterruptHandler( parseSlaveI2C );
    }
    else
    {
        SetExpansionScbConfigurationMaster();
    }
}
