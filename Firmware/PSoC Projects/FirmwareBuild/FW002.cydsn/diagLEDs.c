#include <project.h>
#include "diagLEDS.h"


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
