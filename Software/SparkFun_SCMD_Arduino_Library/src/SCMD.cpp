/******************************************************************************
SparkFun_PSoCMotorDriverAPI.cpp
SCMD Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/______________

<multiline verbose description of file functionality>

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
<arduino/development environment version>
<hardware version>
<etc>

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/
//Define USE_ALT_I2C to use the teesny 3 i2c_t3.h library, which allows I2C bus hang resolution
//#define USE_ALT_I2C

//See _____ for additional topology notes.

#include <stdint.h>
#include <math.h>
#include "SCMD.h"
#include "SCMD_config.h"

#ifdef USE_ALT_I2C
#include <i2c_t3.h>
#define I2C_FAULT_TIMEOUT 1000 //in microseconds
#else
#include "Wire.h"
#endif

#include "SPI.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
SCMD::SCMD( void )
{
	//Construct with these default settings if nothing is specified

	//Select interface mode
	settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
	//Select address for I2C.  Does nothing for SPI
	settings.I2CAddress = 0x58; //Ignored for SPI_MODE
	//Select CS pin for SPI.  Does nothing for I2C
	settings.chipSelectPin = 10;
	

}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t SCMD::begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	switch (settings.commInterface)
	{

	case I2C_MODE:
		Wire.begin(settings.I2CAddress);
		break;

	case SPI_MODE:
		// start the SPI library:
		SPI.begin();
		// Maximum SPI frequency is 10MHz, could divide by 2 here:
		SPI.setClockDivider(SPI_CLOCK_DIV32);
		// Data is read and written MSb first.
		SPI.setBitOrder(MSBFIRST);
		// Data is captured on rising edge of clock (CPHA = 0)
		// Base value of the clock is HIGH (CPOL = 1)
		// This was SPI_MODE3 for RedBoard, but I had to change to
		// MODE0 for Teensy 3.1 operation
		SPI.setDataMode(SPI_MODE0);
		// initalize the  data ready and chip select pins:
		pinMode(settings.chipSelectPin, OUTPUT);
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
	
	//dummy read
	readRegister(SCMD_ID);
	writeRegister(SCMD_DRIVER_ENABLE, 0x01);
//	delay(10);

	return readRegister(SCMD_ID);
}

//Reset function
void SCMD::reset( void )
{
#ifdef USE_ALT_I2C
	//Blast the Teensy 3.1 i2c control register
	uint8_t * I2C_CTRL1_reg;
	I2C_CTRL1_reg = (uint8_t *)0x40066002;
	*I2C_CTRL1_reg = 0x00;
	//Waiting seems like a good idea
	delay(10);
	Wire.resetBus(); //Strictly resets.  Run .begin() afterwards
	Wire.begin();
#endif
	
}
//****************************************************************************//
//
//  Drive Section
//
//****************************************************************************//
void SCMD::setDrive( uint8_t channel, uint8_t direction, uint8_t level )
{
	//convert to 7 bit
	level = level >> 1;
	int16_t driveValue; //use to build value to actually write to register
	
	//switch(channel)
	//{
	//	case 0:  //master
	//		driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
	//		driveValue += 128;
	//		writeRegister(SCMD_MA_DRIVE, driveValue);
	//		break;
	//	case 1:  //master
	//		driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
	//		driveValue += 128;
	//		writeRegister(SCMD_MB_DRIVE, driveValue);
	//		break;
	//	default:
	//	break;
	//}
	if(channel < 34)
	{
		driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
		driveValue += 128;
		writeRegister(SCMD_MA_DRIVE + channel, driveValue);
	}
}

void SCMD::inversionMode( uint8_t motorNum, uint8_t polarity )
{
	uint8_t regTemp;
	//Select target register
	if( motorNum < 2 )
	{
		//master
		if( motorNum == 0 ) writeRegister(SCMD_MOTOR_A_INVERT, polarity & 0x01);
		if( motorNum == 1 ) writeRegister(SCMD_MOTOR_B_INVERT, polarity & 0x01);
	}
	else
	{
		if( motorNum < 10 )
		{
			//register: SCMD_INV_2_9
			regTemp = SCMD_INV_2_9;
			motorNum -= 2;

		}
		else if( motorNum < 18 )
		{
			//register: SCMD_INV_10_17
			regTemp = SCMD_INV_10_17;
			motorNum -= 10;
		}
		else if( motorNum < 26 )
		{
			//register: SCMD_INV_18_25
			regTemp = SCMD_INV_18_25;
			motorNum -= 18;
		}
		else if( motorNum < 34 )
		{
			//register: SCMD_INV_26_33
			regTemp = SCMD_INV_26_33;
			motorNum -= 26;
		}
		else
		{
			//out of range
			return;
		}
		//convert motorNum to one-hot mask
		uint8_t data = readRegister( regTemp ) & ~( 1 << motorNum );
		writeRegister( regTemp, data | ((polarity & 0x01) << motorNum) );
	}

}

void SCMD::bridgingMode( uint8_t driverNum, uint8_t bridged )
{
	uint8_t regTemp;
	//Select target register
	if( driverNum < 1 )
	{
		//master
		writeRegister(SCMD_BRIDGE, bridged & 0x01);
	}
	else
	{
		if( driverNum < 9 )
		{
			//register: SCMD_BRIDGE_SLV_L
			regTemp = SCMD_BRIDGE_SLV_L;
			driverNum -= 1;

		}
		else if( driverNum < 17 )
		{
			//register: SCMD_BRIDGE_SLV_H
			regTemp = SCMD_BRIDGE_SLV_H;
			driverNum -= 9;
		}
		else
		{
			//out of range
			return;
		}
		//convert driverNum to one-hot mask
		uint8_t data = readRegister( regTemp ) & ~( 1 << driverNum );
		writeRegister( regTemp, data | ((bridged & 0x01) << driverNum) );
	}
	
}

//****************************************************************************//
//
//  Diagnostics
//
//****************************************************************************//
void SCMD::getDiagnostics( SCMDDiagnostics &diagObjectReference )
{
	diagObjectReference.userPortI2CTime = readRegister(SCMD_UPORT_TIME);
	//Clear uport time
	writeRegister(SCMD_UPORT_TIME, 0);
	diagObjectReference.rxErrorCount = readRegister(SCMD_U_I2C_RD_ERR);
	diagObjectReference.txErrorCount = readRegister(SCMD_U_I2C_WR_ERR);
	uint8_t topAddr = readRegister(SCMD_SLV_TOP_ADDR);
	if( (topAddr >= START_SLAVE_ADDR) && (topAddr < (START_SLAVE_ADDR + 16)))
	{
		//in valid range
		diagObjectReference.numberOfSlaves = topAddr - START_SLAVE_ADDR + 1;
	}
}

uint8_t SCMD::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	switch (settings.commInterface) {

	case I2C_MODE:
		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
#ifdef USE_ALT_I2C
		if(Wire.endTransmission(I2C_STOP, I2C_FAULT_TIMEOUT)) i2cFaults++;
#else
		Wire.endTransmission();
#endif
#ifdef USE_ALT_I2C
		if( Wire.requestFrom(settings.I2CAddress, numBytes, I2C_STOP, I2C_FAULT_TIMEOUT) == 0 )i2cFaults++;
#else
		Wire.requestFrom(settings.I2CAddress, numBytes);
#endif
		while ( Wire.available() ) // slave may send less than requested
		{
			result = Wire.read(); // receive a byte as a proper uint8_t
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		
		
		for(volatile int i = 0; i < 25; i++);
		
		//do a dummy read
		digitalWrite(settings.chipSelectPin, LOW);
		// send a value of 80 to read the first byte returned:
		result = SPI.transfer(0x80);
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		
		for(volatile int i = 0; i < 25; i++);
		
		break;

	default:
		break;
	}
	return result;
}

void SCMD::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	switch (settings.commInterface)
	{
	case I2C_MODE:
		//Write the byte
		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.write(dataToWrite);
#ifdef USE_ALT_I2C
		if(Wire.endTransmission(I2C_STOP,I2C_FAULT_TIMEOUT)) i2cFaults++;
#else
		Wire.endTransmission();
#endif
//		delay(1);
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset & 0x7F);
		// send a value of 0 to read the first byte returned:
		SPI.transfer(dataToWrite);
		// decrement the number of bytes left to read:
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		
		for(volatile int i = 0; i < 25; i++);
		
		break;

	default:
		break;
	}
}