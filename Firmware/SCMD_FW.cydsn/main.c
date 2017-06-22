/******************************************************************************
main.c
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
#include "charMath.h"
#include "SCMD_config.h"
#include "serial.h"
#include "diagLEDS.h"
#include "slaveEnumeration.h"
#include "customSerialInterrupts.h"
#include "registerHandlers.h"

//Debug stuff
//If USE_SW_CONFIG_BITS is defined, program will use CONFIG_BITS instead of the solder jumpers on the board:
//#define USE_SW_CONFIG_BITS

//Otherwise, this global is used to get the bits from the hardware jumpers.
//Valid options:
//  0x0         -- UART
//  0x1         -- SPI
//  0x2         -- I2C slave on slave port
//  0x3 to 0xE  -- I2C @ 0x58 to 0x63
//  0xF         -- PWM mode (not implemented)
volatile uint8_t CONFIG_BITS = 0x0;

//Prototypes
static void systemInit( void ); //get the system off the ground - calls warm init - run once at start

//Variables for use with timer ISR to measure time. 
volatile uint16_t masterSendCounter = 65000;
volatile bool masterSendCounterReset = 0;
volatile bool breakCounterWait = false;

//Reboot variable
extern volatile bool slaveResetRequested;

//Functions
int main()
{
    systemInit();

    while(1)
    {
		DEBUG_TIMER_Stop();
		DEBUG_TIMER_WriteCounter(0);
		DEBUG_TIMER_Start();
		
        if((CONFIG_BITS == 0) || (CONFIG_BITS == 0x0D) || (CONFIG_BITS == 0x0E)) //UART
        {
            parseUART();
        }
        else if((CONFIG_BITS >= 0x3)&&(CONFIG_BITS <= 0xC)) //I2C
        {
            //parceI2C is also called before interrupts occur on the bus, but check here too to catch residual buffers
            parseI2C();
        }
        //parseSPI() now called from within the interrupt only

       
        if(CONFIG_BITS == 2) //Slave
        {
            parseSlaveI2C();
            tickSlaveSM();
            processSlaveRegChanges();
        }
        else //Do master operations
        {
			tickMasterSM();
            processMasterRegChanges();

        }        
        //operations regardless       
        processRegChanges();
        
		//Save time
		uint32_t tempValue = DEBUG_TIMER_ReadCounter() / 100;
		if(tempValue > 0xFF) tempValue = 0xFF;
		//do 'peak hold' on SCMD_UPORT_TIME -- write 0 to reset
		if(tempValue > readDevRegister(SCMD_LOOP_TIME)) writeDevRegister(SCMD_LOOP_TIME, tempValue);
    }//End of while loop
}

/*******************************************************************************
* Define Interrupt service routine and allocate an vector to the Interrupt
********************************************************************************/
CY_ISR(FSAFE_TIMER_Interrupt)
{
    CyGlobalIntDisable;
    /* Clear TC Inerrupt */
   	FSAFE_TIMER_ClearInterrupt(FSAFE_TIMER_INTR_MASK_CC_MATCH);
    incrementDevRegister( SCMD_FSAFE_FAULTS );
    
    //  Plan to escape counter wait
    breakCounterWait = true;
    
    //  Collect behavior information
    uint8_t drive_behavior = readDevRegister(SCMD_FSAFE_CTRL) & SCMD_FSAFE_DRIVE_KILL;
    uint8_t reset_behavior = (readDevRegister(SCMD_FSAFE_CTRL) & SCMD_FSAFE_RESTART_MASK) >> 1;
    uint8_t user_behavior = (readDevRegister(SCMD_FSAFE_CTRL) & SCMD_FSAFE_CYCLE_USER) >> 3;
    uint8_t expansion_behavior = (readDevRegister(SCMD_FSAFE_CTRL) & SCMD_FSAFE_CYCLE_EXP) >> 4;
    
    //  Stop the motor
    if( drive_behavior == 1)
    {
        writeDevRegister( SCMD_MA_DRIVE, 0x80 );
        writeDevRegister( SCMD_MB_DRIVE, 0x80 );
    }
	//  Restart user or expansion ports
    if( user_behavior == 1)
    {
        initUserSerial(CONFIG_BITS);
    }
    if( expansion_behavior == 1)
    {
        initExpansionSerial(CONFIG_BITS); 
    }
    //  Re-enumerate or reboot
    switch(reset_behavior)
    {
        default:
        case 0x00:
        break;
        case 0x01: //Reboot
            hardReset();
        break;
        case 0x02: //Re-enumerate
            reEnumerate();
        break;
    }
    
    //  Tell someone
    setDiagMessage(7,8);
    
    //  reset the counter
    FSAFE_TIMER_Stop();
    FSAFE_TIMER_WriteCounter(0);
    FSAFE_TIMER_Start();
    CyGlobalIntEnable;

}

/*******************************************************************************
* Define Interrupt service routine and allocate an vector to the Interrupt
********************************************************************************/
CY_ISR(ConfigInBehaviorHandler)
{
    CyGlobalIntDisable;
    M_IN_ISR_Stop();
    CONFIG_IN_ClearInterrupt();
   
    //Send the reset request
    switch( readDevRegister( SCMD_MST_E_IN_FN ) & SCMD_M_IN_RESTART_MASK )
    {
        default:
        case 0:
            //  Restart user or expansion ports
            if(readDevRegister( SCMD_MST_E_IN_FN ) & SCMD_M_IN_CYCLE_USER )
            {
                initUserSerial( readDevRegister( SCMD_CONFIG_BITS ) );
                CyDelay(100);
            }
            if(readDevRegister( SCMD_MST_E_IN_FN ) & SCMD_M_IN_CYCLE_EXP )
            {
                initExpansionSerial( readDevRegister( SCMD_CONFIG_BITS ) );
                CyDelay(100);
            }
            M_IN_ISR_StartEx(ConfigInBehaviorHandler);
        break;
        case 1:
            hardReset();
        break;
        case 2:
            //  Restart user or expansion ports
            if(readDevRegister( SCMD_MST_E_IN_FN ) & SCMD_M_IN_CYCLE_USER )
            {
                initUserSerial( readDevRegister( SCMD_CONFIG_BITS ) );
                CyDelay(100);
            }
            if(readDevRegister( SCMD_MST_E_IN_FN ) & SCMD_M_IN_CYCLE_EXP )
            {
                initExpansionSerial( readDevRegister( SCMD_CONFIG_BITS ) );
                CyDelay(100);
            }
            writeDevRegister( SCMD_CONTROL_1, readDevRegister( SCMD_CONTROL_1 ) | SCMD_RE_ENUMERATE_BIT ); //Set bit
            M_IN_ISR_StartEx(ConfigInBehaviorHandler);
        break;
    }
    
    CyGlobalIntEnable;

}

/* ISR prototype declaration */
CY_ISR_PROTO(SYSTICK_ISR);

/* User ISR function definition */
CY_ISR(SYSTICK_ISR)
{
	/* User ISR Code*/
    if(masterSendCounterReset)
    {
        //clear reset
        masterSendCounterReset = 0;
        masterSendCounter = 0;
    }
    else
    {
        if(masterSendCounter < 0xFFFF) //hang at top value
        {
            masterSendCounter++;
        }
    }
}

//get the system off the ground - run once at start
static void systemInit( void )
{
    initDevRegisters();  //Prep device registers, set initial values (triggers actions)
#ifndef USE_SW_CONFIG_BITS
    CONFIG_BITS = readDevRegister(SCMD_CONFIG_BITS); //Get the bits value
#endif
    
    DIAG_LED_CLK_Stop();
    DIAG_LED_CLK_Start();
    KHZ_CLK_Stop();
    KHZ_CLK_Start();
    //setDiagMessage(0, CONFIG_BITS);
    
    CONFIG_IN_Write(0); //Tell the slaves to start fresh
    //Do a boot-up delay
    CyDelay(100u);
    if(CONFIG_BITS != 0x02) CyDelay(1000u); //Give the slaves extra time
    
    MODE_Write(1);
    
    //Debug timer
    DEBUG_CLK_Stop();
    DEBUG_CLK_Start();
    DEBUG_TIMER_Stop();
    DEBUG_TIMER_Start();
    
    //SysTick timer (software driven)
    // Map systick ISR to the user defined ISR. SysTick_IRQn is already defined in core_cm0.h file
	CyIntSetSysVector((SysTick_IRQn + 16), SYSTICK_ISR);
	
    /* Enable Systick timer with desired period/number of ticks */
	SysTick_Config(24000);  //Interrupt should occur every 1 ms
    
    //Failsafe timer
    FSAFE_ISR_StartEx(FSAFE_TIMER_Interrupt);
    FSAFE_CLK_Stop();
    FSAFE_CLK_Start();
    FSAFE_TIMER_Stop();
    FSAFE_TIMER_Start();
       
    CyDelay(50u);
    
    /* Start the components */
    PWM_1_Start();
    PWM_1_WriteCompare(128u);
    PWM_2_Start();
    PWM_2_WriteCompare(128u);        
    
    //This configures the serial based on the config structures in serial.h
    //It connects the functions in customSerialInterrupts.h, and is defined in that file.
	calcUserDivider(CONFIG_BITS);
	calcExpansionDivider(CONFIG_BITS);

    initUserSerial(CONFIG_BITS);
	initExpansionSerial(CONFIG_BITS);
    
    //Clock_1 is the motor PWM clock
    Clock_1_Stop();
    Clock_1_Start();
    
    //Config in behavior
    if(CONFIG_BITS != 0x02) M_IN_ISR_StartEx(ConfigInBehaviorHandler);
    
    CyGlobalIntEnable; 
    
    setDiagMessage(1, 1);

}

/* [] END OF FILE */
