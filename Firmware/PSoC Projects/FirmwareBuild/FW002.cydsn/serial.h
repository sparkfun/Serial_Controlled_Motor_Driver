/******************************************************************************
serial.h
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
#if !defined(SERIAL_H)
#define SERIAL_H

#include <project.h>

#define USER_PORT_BUFFER_SIZE (10u)

/* Common buffers or I2C and UART */
uint8 bufferRx[USER_PORT_BUFFER_SIZE + 1u];/* RX software buffer requires one extra entry for correct operation in UART mode */
uint8 bufferTx[USER_PORT_BUFFER_SIZE]; /* TX software buffer */

/* Buffers for expansion port */
uint8 expansionBufferRx[USER_PORT_BUFFER_SIZE + 1u];/* RX software buffer requires one extra entry for correct operation in UART mode */
uint8 expansionBufferTx[USER_PORT_BUFFER_SIZE]; /* TX software buffer */


/* The clock divider value written into the register has to be one less from calculated */
#define SCBCLK_I2C_DIVIDER (14u) // I2C Slave: 100 kbps Required SCBCLK = 1.6 MHz, Div = 15  ----- good for 100kHZ
//#define SCBCLK_I2C_DIVIDER (4u) // I2C Slave: 100 kbps Required SCBCLK = 1.6 MHz, Div = 15  ----- Kind of ok for 400kHZ
//!! SCBCLK_URAT_DIVIDER is now dynamic and assigned as a variable in "serial.c"!!
//#define SCBCLK_UART_DIVIDER (12u) // UART: 115200 kbps with OVS = 16. Required SCBCLK = 1.846 MHz, Div = 13
#define SCBCLK_SPI_DIVIDER (1u) //Max for now -- Does this even matter?

/* Operation mode: I2C slave or UART */
#define OP_MODE_UART (1u)
#define OP_MODE_I2C (2u)
#define OP_MODE_SPI (3u)

cystatus SetScbConfiguration(uint32 opMode);

// ***Known good config for slave*** //
cystatus SetExpansionScbConfigurationSlave(void);
cystatus ResetExpansionScbConfigurationSlave(void);

//Expansion port config
cystatus SetExpansionScbConfigurationMaster(void);

uint8 ReadSlaveData( uint8_t address, uint8_t offset );
uint8 WriteSlaveData( uint8_t address, uint8_t offset, uint8_t data );
uint8 WriteSlave2Data( uint8_t address, uint8_t offset, uint8_t data0, uint8_t data1 );


#endif