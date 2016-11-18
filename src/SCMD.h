/******************************************************************************
SCMD.h
Serial Controlled Motor Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/Serial_Controlled_Motor_Driver
https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.7
Teensy loader 1.27

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef __SERIALCONTROLLEDMOTORDRIVERAPI_H__
#define __SERIALCONTROLLEDMOTORDRIVERAPI_H__

#include <stdint.h>
#include "SCMD_config.h"

//  Setting options for commInterface
#define I2C_MODE 0
#define SPI_MODE 1

//  SCMDSettings
//
//    This is used by the SCMD class to hold settings.  It is public within that class
//  and the user is expected to write desired values into the settings before calling
//  .begin();
struct SCMDSettings
{
  public:
	
  //Main Interface and mode settings
    uint8_t commInterface;  //Set equal to I2C_MODE or SPI_MODE
    uint8_t I2CAddress;  //Set to address that master is configured to in case of I2C usage
    uint8_t chipSelectPin;  //Set to chip select pin used on Arduino in case of SPI

};

//  SCMDDiagnostics
//
//    This can be created by the application and used to hold data.  Pass the created
//  object to the diagnostic reading functions and they will be written with the
//  aquired data.
struct SCMDDiagnostics
{
	public:
	//Attainable metrics from SCMD
	uint8_t numberOfSlaves = 0;
	uint8_t U_I2C_RD_ERR = 0;
	uint8_t U_I2C_WR_ERR = 0;
	uint8_t U_BUF_DUMPED = 0;
	uint8_t E_I2C_RD_ERR = 0;
	uint8_t E_I2C_WR_ERR = 0;
	uint8_t LOOP_TIME = 0;
	uint8_t SLV_POLL_CNT = 0;
	uint8_t MST_E_ERR = 0;
	uint8_t MST_E_STATUS = 0;
	uint8_t FSAFE_FAULTS = 0;
	uint8_t REG_OOR_CNT = 0;
	uint8_t REG_RO_WRITE_CNT = 0;

 };

//  SCMD
//
//    This object provides control of a motor driver and attached slave chain, for
//  a single master address.
//
//  create an object in the global space, then configure and run begin();
class SCMD
{
  public:
    //settings
    SCMDSettings settings;

	//Constructor generates default SCMDSettings.
    SCMD( void );
	
	
    uint8_t begin( void );  //Call to apply SCMDSettings and returns ID word
	bool ready( void ); //Returns 1 when enumeration is complete
	bool busy( void ); //Returns 1 while the SCMD is busy with tasks that should not be interrupted
	void enable( void ); //Sets all connected SCMDs to enable
	void disable( void ); //Sets all connected SCMDs to disable
	void reset( void );  //Software reset routine
    void setDrive( uint8_t motorNum, uint8_t direction, uint8_t level );//apply drive levels to motors
	void inversionMode( uint8_t motorNum, uint8_t polarity );//Set inversion states for motors
	void bridgingMode( uint8_t driverNum, uint8_t bridged );//Enable bridging ('B' channel will have no effect in bridged mode)
	void getDiagnostics( SCMDDiagnostics &diagObjectReference );//Gets and formats the diagnostic information.  Make sure the passed char array is big enough (size not determined yet)
	void getRemoteDiagnostics( uint8_t address, SCMDDiagnostics &diagObjectReference );//send remote address
	void resetDiagnosticCounts( void );
	void resetRemoteDiagnosticCounts( uint8_t address );
	
	
    //The following utilities read and write
    uint8_t readRegister(uint8_t offset);//reads a byte
    void writeRegister(uint8_t offset, uint8_t dataToWrite);//Writes a byte;
    uint8_t readRemoteRegister(uint8_t address, uint8_t offset);//Reads a slave through the slave access registers
    void writeRemoteRegister(uint8_t address, uint8_t offset, uint8_t dataToWrite);//Writes a slave through the slave access registers
	
	//Diagnostic
	uint16_t i2cFaults; //Location to hold i2c faults for alternate driver
	
};



#endif  // End of __BME280_H__ definition check
