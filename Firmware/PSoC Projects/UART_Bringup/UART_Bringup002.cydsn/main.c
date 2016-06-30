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

int main()
{
   
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    LED_R_Write(1);
    DEBUGPIN0_Write(0);
    DEBUGPIN1_Write(1);
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
        
        if((debugLEDs >= '0')&&(debugLEDs <= '9'))
        {
            //calc num
            uint16_t num = (debugLEDs - 47) * 1000;
            PWM_1_WriteCompare(num);

        }        
        if((debugLEDs >= 'a')&&(debugLEDs <= 'j'))
        {
            //calc num
            uint16_t num = (debugLEDs - 96) * 1000;
            PWM_2_WriteCompare(num);
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
            DEBUGPIN0_Write(0);
        }
        else
        {
            DEBUGPIN0_Write(1);
        }
        if(debugLEDs & 0x04)
        {
            DEBUGPIN1_Write(0);
        }
        else
        {
            DEBUGPIN1_Write(1);
        }
        //CyDelay(1000);
        //debugLEDs++;
        //if(debugLEDs > 7) debugLEDs = 0;
    }
}

/* [] END OF FILE */
