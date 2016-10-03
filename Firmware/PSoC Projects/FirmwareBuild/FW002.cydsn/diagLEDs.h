/******************************************************************************
diagLEDs.h
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
#if !defined(DIAGLEDS_H)
#define DIAGLEDS_H

#include <stdint.h>
    
void setDiagLeds(uint8_t input);

void sendDiagMessage( void ); //send error level

void setDiagMessage( uint8_t errorLevel, uint8_t message ); //send error level

void clearDiagMessage( uint8_t errorLevel ); //send error level
    
    
#endif