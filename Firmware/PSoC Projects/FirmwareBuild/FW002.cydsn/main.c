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

uint8_t debugLED = 0;

volatile uint8_t CONFIG_BITS = 0x3;
    //0 -- UART
    //1 -- SPI
    //2 -- I2C slave on slave port
    //3 -- I2C @ 0x58

/*******************************************************************************
* Define Interrupt service routine and allocate an vector to the Interrupt
********************************************************************************/
//CY_ISR(InterruptHandler)
//{
//	/* Clear TC Inerrupt */
//   	PWM_1_ClearInterrupt(PWM_1_INTR_MASK_TC);
//    
//	/* Increment the Compare for LED brighrness decrease */ 
//    PWM_1_WriteCompare(10000u);
//}


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

const EXPANSION_PORT_I2C_INIT_STRUCT expansionConfigI2C =
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

cystatus SetExpansionScbConfiguration(void)
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
    EXPANSION_PORT_I2CInit(&expansionConfigI2C);
    EXPANSION_PORT_Start(); /* Enable component after configuration change */
    return (status);
}

int main()
{
   
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    LED_R_Write(1);
    MODE_Write(1);
    A_EN_Write(1);
    B_EN_Write(1);
    
    initDevRegisters();  //Prep device registers
    writeDevRegister(0x03, CONFIG_BITS);
    writeDevRegister(0x10, 0x80);
    writeDevRegister(0x11, 0x80);
    
    
    //P2_4_Write(1);

    //stuff from example TCPWM
       
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

    }
    if(CONFIG_BITS == 3) //I2C 0x58 
    {
        SetScbConfiguration(OP_MODE_I2C);
    }
    
    Clock_1_Start();
    SetExpansionScbConfiguration();
        
    CyGlobalIntEnable; 

//    isr_1_Start();      /* Initializing the ISR */
//    USER_PORT_Start();     /* Enabling the UART */

    
    uint8_t addressPointer = 0;
        
    char ch;
    uint8_t rxBufferPtr = 0;

    char rxBuffer[20];
    
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
        if(CONFIG_BITS == 2) //Slave
        {

        }
        if(CONFIG_BITS == 3) //I2C 0x58 
        {
            /* Write complete: parse command packet */
            if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_WR_CMPLT))
            {
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
            USER_PORT_I2CSlaveClearWriteStatus();
            USER_PORT_I2CSlaveClearWriteBuf();
                
            }
            //always expose buffer?
            /*expose buffer to master */
            bufferTx[0] = readDevRegister(addressPointer);
    
            if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_RD_CMPLT))
            {
                LED_R_Write(LED_R_Read()^0x01);
                /* Clear slave read buffer and status */
                USER_PORT_I2CSlaveClearReadBuf();
                (void) USER_PORT_I2CSlaveClearReadStatus();
            }
        }
        
        //Do slave
        {
            /* Write complete: parse command packet */
            if (0u != (EXPANSION_PORT_I2CSlaveStatus() & EXPANSION_PORT_I2C_SSTAT_WR_CMPLT))
            {
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

        }
        
        
        //Set outputs
        
        PWM_1_WriteCompare(readDevRegister(0x10));
        PWM_2_WriteCompare(readDevRegister(0x11));    
        
    }//End of while loop

}






/* [] END OF FILE */
