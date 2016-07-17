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
#ifndef __PSOCMD_H__
#define __PSOCMD_H__

#include "stdint.h"

#define I2C_MODE 0
#define SPI_MODE 1

//Register names:
#define PSoCMD_STATUS                   0x00
#define PSoCMD_ID                       0x01
#define PSoCMD_ADDRESS                  0x02
#define PSoCMD_CONFIG_BITS              0x03
#define PSoCMD_DEBUG                    0x0A
#define PSoCMD_MA_DRIVE                 0x10
#define PSoCMD_MB_DRIVE                 0x11


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
struct PSoCMDSettings
{
  public:
	
  //Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;
	uint8_t invertA;
	uint8_t invertB;
};


//This is the man operational class of the driver.

class PSoCMD
{
  public:
    //settings
    PSoCMDSettings settings;

	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
    PSoCMD( void );
    //~BME280() = default;
	
	//Call to apply SensorSettings.
	//This also gets the SensorCalibration constants
    uint8_t begin( void );

	//Software reset routine
	void reset( void );
	
    //Returns the values as floats.
    void setDrive( uint8_t, uint8_t, uint8_t );
	
    //The following utilities read and write

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//a chunk of memory into that array.
    void readRegisterRegion(uint8_t*, uint8_t, uint8_t );
	//readRegister reads one register
    uint8_t readRegister(uint8_t);
    //Reads two regs, LSByte then MSByte order, and concatenates them
	//Used for two-byte reads
	int16_t readRegisterInt16( uint8_t offset );
	//Writes a byte;
    void writeRegister(uint8_t, uint8_t);
    
};



#endif  // End of __BME280_H__ definition check
