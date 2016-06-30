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

uint8_t debugLED = 0;

volatile uint8_t CONFIG_BITS = 0x03;

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


/* The clock divider value written into the register has to be one less from calculated */
#define SCBCLK_I2C_DIVIDER (14u) /* I2C Slave: 100 kbps Required SCBCLK = 1.6 MHz, Div = 15 */
#define SCBCLK_UART_DIVIDER (12u) /* UART: 115200 kbps with OVS = 16. Required SCBCLK = 1.846 MHz, Div = 13 */
/* Operation mode: I2C slave or UART */
#define OP_MODE_UART (1u)
#define OP_MODE_I2C (2u)
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
else
{
status = CYRET_BAD_PARAM; /* Uknowns operation mode - no actions */
}
return (status);
}

int main()
{
   
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    LED_R_Write(1);
    DEBUGPIN0_Write(0);
    DEBUGPIN1_Write(1);
    DEBUGPIN2_Write(0);
    DEBUGPIN3_Write(1);
    P2_3_MODE_Write(1);
    P2_5_A_EN_Write(1);
    P2_7_B_EN_Write(1);
    
    
    //P2_4_Write(1);

    //stuff from example TCPWM
       
        CyDelay(50u);
        
    	/* Start the components */
        PWM_1_Start();
        PWM_1_WriteCompare(1000u);
        PWM_2_Start();
        PWM_2_WriteCompare(3000u);        
        
    Clock_1_Start();
    
    CyGlobalIntEnable; 

//    isr_1_Start();      /* Initializing the ISR */
    USER_PORT_Start();     /* Enabling the UART */

    

    
    char ch;
    uint8_t rxBufferPtr = 0;

    char rxBuffer[20];
    
    if(CONFIG_BITS == 0) //UART
    {
        SetScbConfiguration(OP_MODE_UART);
        
        for(;;)
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
    }
    if(CONFIG_BITS == 1) //SPI
    {

    }
    if(CONFIG_BITS == 2) //Slave
    {

    }
    if(CONFIG_BITS == 3) //I2C 0x58 
    {
        SetScbConfiguration(OP_MODE_I2C);
    }
    while(1)
    {
        /* Write complete: parse command packet */
        if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_WR_CMPLT))
        {
            /* Check packet length */
//            if (PACKET_SIZE == I2CS_I2CSlaveGetWriteBufSize())
//            {
//                /* Check start and end of packet markers */
//                if ((i2cWriteBuffer[PACKET_SOP_POS] == PACKET_SOP) &&
//                    (i2cWriteBuffer[PACKET_EOP_POS] == PACKET_EOP))
//                {
//                    //ExecuteCommand(i2cWriteBuffer[PACKET_CMD_POS]);
//                }
//            }
        if(bufferRx[0] == 0xE9)
        {
            DEBUGPIN2_Write(DEBUGPIN2_Read()^0x01);
        }
          DEBUGPIN3_Write(DEBUGPIN3_Read()^0x01);

            /* Clear slave write buffer and status */
            USER_PORT_I2CSlaveClearWriteBuf();
            (void) USER_PORT_I2CSlaveClearWriteStatus();

            /* Update read buffer */
            //i2cReadBuffer[PACKET_STS_POS] = status;
            //status = STS_CMD_FAIL;
        }

        /* Read complete: expose buffer to master */
        bufferTx[0] = 0xAA;
        if (0u != (USER_PORT_I2CSlaveStatus() & USER_PORT_I2C_SSTAT_RD_CMPLT))
        {
            LED_R_Write(LED_R_Read()^0x01);
            /* Clear slave read buffer and status */
            USER_PORT_I2CSlaveClearReadBuf();
            (void) USER_PORT_I2CSlaveClearReadStatus();
        }
    }

    
}






/* [] END OF FILE */
