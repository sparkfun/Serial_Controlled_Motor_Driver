/******************************************************************************
customSerialInterrupts.h
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
#if !defined(CUSTOMSERIALINTERRUPTS_H)
#define CUSTOMSERIALINTERRUPTS_H

#include "project.h"

CY_ISR_PROTO(custom_USER_PORT_SPI_UART_ISR);

//Prototypes
void parseSPI( void );
void parseUART( void );
void parseI2C( void );
void parseSlaveI2C( void );

#endif
