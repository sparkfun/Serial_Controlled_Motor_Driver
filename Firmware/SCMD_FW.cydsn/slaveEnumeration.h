/******************************************************************************
slaveEnumeration.h
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
#if !defined(SLAVEENUMERATION_H)
#define SLAVEENUMERATION_H
#include <stdint.h> 
#include <stdbool.h>

void tickMasterSM( void );
void tickSlaveSM( void );

void resetMasterSM( void );
bool masterSMDone( void );
void hardReset( void );
void reEnumerate( void );

#endif