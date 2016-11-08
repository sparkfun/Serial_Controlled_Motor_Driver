/******************************************************************************
devRegisters.h
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
#if !defined(DEV_REGISTERS_H)
#define DEV_REGISTERS_H
#include <stdint.h> 
#include <stdbool.h>

void initDevRegisters( void );
uint8_t readDevRegister( uint8_t regNumberIn );
void writeDevRegister( uint8_t regNumberIn, uint8_t dataToWrite );
void writeDevRegisterUnprotected( uint8_t regNumberIn, uint8_t dataToWrite );
void incrementDevRegister( uint8_t );
bool getChangedStatus( uint8_t regNumberIn );
void clearChangedStatus( uint8_t regNumberIn );
void setColdInitValues( void );
void setWarmInitValues( void );
void setBusyBitMem( uint8_t );// Send register value
void clearBusyBitMem( uint8_t );// Send register value

#endif