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

uint8_t debugLEDs = 0;

int main()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    LED_R_Write(1);
    LED_G_Write(0);
    LED_B_Write(1);
    
    CyGlobalIntEnable; 

    isr_1_Start();      /* Initializing the ISR */
    UART_1_Start();     /* Enabling the UART */

    

    
    char ch;
    for(;;)
    {
        ch = UART_1_UartGetChar(); 

        if (0u != ch)
        {   
            debugLEDs = ch;
            UART_1_UartPutChar(ch); /* Sending the data */
        }
        /* Place your application code here. */
        //UART_1_UartPutChar('A'); /* Sending the data */
        if(debugLEDs & 0x01)
        {
            LED_R_Write(0);
        }
        else
        {
            LED_R_Write(1);
        }
        if(debugLEDs & 0x02)
        {
            LED_G_Write(0);
        }
        else
        {
            LED_G_Write(1);
        }
        if(debugLEDs & 0x04)
        {
            LED_B_Write(0);
        }
        else
        {
            LED_B_Write(1);
        }
        //CyDelay(1000);
        //debugLEDs++;
        //if(debugLEDs > 7) debugLEDs = 0;
    }
}

/* [] END OF FILE */
