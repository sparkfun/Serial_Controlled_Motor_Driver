//#include "project.h"
#include "devRegisters.h"
#include "customSpiInt.h"
#include "USER_PORT_PVT.h"
#include "USER_PORT_SPI_UART_PVT.h"

static void parseSPI( void );

uint8_t rxTemp[10];
uint8_t rxTempPtr = 0;
uint8_t masterAddressPointer = 0;
//This replaces USER_PORT_SPI_UART_ISR

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

static void parseSPI( void )
{
    if(USER_PORT_SpiUartGetRxBufferSize())
    {
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
            volatile uint8_t reg = readDevRegister(masterAddressPointer);
            USER_PORT_SpiUartWriteTxData(reg); //This will be available on next read, during command bits
            //We should check that the buffer doesn't have unused data and count!
            //But we won't...
            USER_PORT_SpiUartClearRxBuffer();
            rxTempPtr = 0;
            
        }
        else
        {
            //Command is WRITE
            //Ok, if packet has more than 1 byte
            if(rxTempPtr > 1)
            {
                masterAddressPointer = rxTemp[0] & 0x7F;
                writeDevRegister(masterAddressPointer, rxTemp[1]); //Write the next byte
                //We should check that the buffer doesn't have unused data and count!
                //But we won't...
                USER_PORT_SpiUartClearRxBuffer();
                rxTempPtr = 0;
            }
            //If not enough data, do nothing and wait for next loop.
        }
        
        
        //USER_PORT_SpiUartWriteTxData(0xA1);
        LED_R_Write(LED_R_Read()^0x01);
    }

}


//static void parseSPI( void )
//{
//   
//    uint8_t rxTemp[10];
//    uint8_t rxTempPtr = 0;
//    //rxTempPtr = 
//    //check for interrupt, if so, toggle LED
//    if(USER_PORT_SpiUartGetRxBufferSize())
//    {
//        rxTempPtr = 0;
//        while(USER_PORT_SpiUartGetRxBufferSize())  //This reads out all rx data
//        {
//            rxTemp[rxTempPtr] = USER_PORT_SpiUartReadRxData();
//            if(rxTempPtr < 9) rxTempPtr++;
//        }
//        USER_PORT_SpiUartClearRxBuffer();
//
//        //check first bit
//        if(rxTemp[0] & 0x80)
//        {
//            //Command is READ, NOT WRITE
//            masterAddressPointer = rxTemp[0] & 0x7F;
//            USER_PORT_SpiUartClearTxBuffer();
//            volatile uint8_t reg = readDevRegister(masterAddressPointer);
//            USER_PORT_SpiUartWriteTxData(reg); //This will be available on next read, during command bits
//        }
//        else
//        {
//            //Command is WRITE
//            //Ok, if packet has more than 1 byte
//            if(rxTempPtr > 1)
//            {
//                masterAddressPointer = rxTemp[0] & 0x7F;
//                writeDevRegister(masterAddressPointer, rxTemp[1]); //Write the next byte
//            }
//        }
//        
//        
//        //USER_PORT_SpiUartWriteTxData(0xA1);
//        LED_R_Write(LED_R_Read()^0x01);
//    }
//
//}

/* [] END OF FILE */