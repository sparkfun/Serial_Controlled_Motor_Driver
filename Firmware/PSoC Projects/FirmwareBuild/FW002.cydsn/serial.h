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

//Expansion port config
cystatus SetExpansionScbConfigurationMaster(void);

uint8 ReadSlaveData( uint8_t address, uint8_t offset );

uint8 WriteSlaveData( uint8_t address, uint8_t offset, uint8_t data );


#endif