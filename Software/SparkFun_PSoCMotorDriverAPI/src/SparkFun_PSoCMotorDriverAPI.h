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
#define SCMD_STATUS          0x00
#define SCMD_ID              0x01
#define SCMD_SLAVE_ADDR      0x02
#define SCMD_CONFIG_BITS     0x03
#define SCMD_I2C_FAULTS      0x04
#define SCMD_I2C_RD_ERR      0x05
#define SCMD_I2C_WR_ERR      0x06
#define SCMD_SPI_FAULTS      0x07
#define SCMD_UART_FAULTS     0x08
#define SCMD_UPORT_TIME	     0x09
#define SCMD_SLV_POLL_CNT    0x0A
#define SCMD_SLV_TOP_ADDR    0x0B

#define SCMD_SLAVE_ID        0x10
#define SCMD_REM_ADDR        0x11
#define SCMD_REM_OFFSET      0x12
#define SCMD_REM_DATA_WR     0x13
#define SCMD_REM_DATA_RD     0x14
#define SCMD_REM_WRITE       0x15
#define SCMD_REM_READ        0x16

#define SCMD_FSAFE_TIME      0x18
#define SCMD_FSAFE_FAULTS    0x19

#define SCMD_MA_DRIVE        0x20
#define SCMD_MB_DRIVE        0x21
#define SCMD_S1A_DRIVE       0x22
#define SCMD_S1B_DRIVE       0x23
#define SCMD_S2A_DRIVE       0x24
#define SCMD_S2B_DRIVE       0x25
#define SCMD_S3A_DRIVE       0x26
#define SCMD_S3B_DRIVE       0x27
#define SCMD_S4A_DRIVE       0x28
#define SCMD_S4B_DRIVE       0x29
#define SCMD_S5A_DRIVE       0x2A
#define SCMD_S5B_DRIVE       0x2B
#define SCMD_S6A_DRIVE       0x2C
#define SCMD_S6B_DRIVE       0x2D
#define SCMD_S7A_DRIVE       0x2E
#define SCMD_S7B_DRIVE       0x2F
#define SCMD_S8A_DRIVE       0x30
#define SCMD_S8B_DRIVE       0x31
#define SCMD_S9A_DRIVE       0x32
#define SCMD_S9B_DRIVE       0x33
#define SCMD_S10A_DRIVE       0x34
#define SCMD_S10B_DRIVE       0x35
#define SCMD_S11A_DRIVE       0x36
#define SCMD_S11B_DRIVE       0x37
#define SCMD_S12A_DRIVE       0x38
#define SCMD_S12B_DRIVE       0x39
#define SCMD_S13A_DRIVE       0x3A
#define SCMD_S13B_DRIVE       0x3B
#define SCMD_S14A_DRIVE       0x3C
#define SCMD_S14B_DRIVE       0x3D
#define SCMD_S15A_DRIVE       0x3E
#define SCMD_S15B_DRIVE       0x3F
#define SCMD_S16A_DRIVE       0x40
#define SCMD_S16B_DRIVE       0x41




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

	//Constructor generates default PSoCMDSettings.
    PSoCMD( void );
	
	//Call to apply PSoCMDSettings.
    uint8_t begin( void );

	//Software reset routine
	void reset( void );
	
    //Operate the device
    void setDrive( uint8_t, uint8_t, uint8_t );
	void getDiagnostics( char * );
	
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
