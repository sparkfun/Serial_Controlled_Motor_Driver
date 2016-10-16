/******************************************************************************
ArduinoPSoCMotorDriverAPI.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/______________

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE _______
Teensy loader ________

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

// Test derived class for base class SparkFunIMU
#ifndef __SERIALCONTROLLEDMOTORDRIVERAPI_H__
#define __SERIALCONTROLLEDMOTORDRIVERAPI_H__

#include <stdint.h>
#include "SCMD_config.h"

#define I2C_MODE 0
#define SPI_MODE 1

//Class SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)
//
struct SCMDSettings
{
  public:
	
  //Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;

};

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
	uint8_t UPORT_TIME = 0;
	uint8_t SLV_POLL_CNT = 0;
	uint8_t MST_E_ERR = 0;
	uint8_t FSAFE_FAULTS = 0;

 };

//This is the man operational class of the driver.

class SCMD
{
  public:
    //settings
    SCMDSettings settings;

	//Constructor generates default SCMDSettings.
    SCMD( void );
	
	
    uint8_t begin( void );  //Call to apply SCMDSettings and enable drivers.
	void reset( void );  //Software reset routine
    void setDrive( uint8_t channel, uint8_t direction, uint8_t level );//apply drive levels to motors
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
