/******************************************************************************
diagLEDs.c
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
#include "diagLEDS.h"

//Prototypes
static void setDiagLeds(uint8_t input);

//Variables
uint8_t lastDiagLeds = 0;
uint8_t errorLevelMemory[8] = {0,0,0,0,0,0,0,0}; //variable to hold flash numbers.  High index prioritized

//Functions
static void setDiagLeds(uint8_t input)
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

//put the appropriate layer on the physical LEDs
//  This starts at max depth in the errorLevelMemory array
//  and stop at the first with data.  The output gives
//  priority to higher element numbers.
void displayDiagMessage( void ) //send error level
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

//put 'number of blinks' in message layers
void setDiagMessage( uint8_t errorLevel, uint8_t message ) //send error level
{
    errorLevelMemory[errorLevel] = message;
    displayDiagMessage();
}

//remove a message from the layers
void clearDiagMessage( uint8_t errorLevel ) //send error level
{
    errorLevelMemory[errorLevel] = 0;
    displayDiagMessage();
}
