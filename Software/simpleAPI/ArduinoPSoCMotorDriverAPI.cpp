/******************************************************************************
ArduinoPSoCMotorDriverAPI.cpp
PSoCMD Arduino and Teensy Driver
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
//See ArduinoPSoCMotorDriverAPI.h for additional topology notes.

#include "ArduinoPSoCMotorDriverAPI.h"
#include "stdint.h"
#include <math.h>

#include "Wire.h"
#include "SPI.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
PSoCMD::PSoCMD( void )
{
	//Construct with these default settings if nothing is specified

	//Select interface mode
	settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
	//Select address for I2C.  Does nothing for SPI
	settings.I2CAddress = 0x58; //Ignored for SPI_MODE
	//Select CS pin for SPI.  Does nothing for I2C
	settings.chipSelectPin = 10;
	settings.invertA = 0;
	settings.invertB = 0;

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
uint8_t PSoCMD::begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	switch (settings.commInterface)
	{

	case I2C_MODE:
		Wire.begin();
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
		SPI.setDataMode(SPI_MODE3);
		// initalize the  data ready and chip select pins:
		pinMode(settings.chipSelectPin, OUTPUT);
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
	
	return readRegister(0x01);
}

//Strictly resets.  Run .begin() afterwards
void PSoCMD::reset( void )
{
	//No reset currently handled
	//writeRegister(BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Drive Section
//
//****************************************************************************//
void PSoCMD::setDrive( uint8_t channel, uint8_t direction, uint8_t level )
{
	//convert to 7 bit
	level = level >> 1;
	int16_t driveValue; //use to build value to actually write to register
	
	switch(channel)
	{
		case 0:  //master
			direction ^= settings.invertA;
			driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
			driveValue += 128;
			writeRegister(PSoCMD_MA_DRIVE, driveValue);
			break;
		case 1:  //master
			direction ^= settings.invertB;
			driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
			driveValue += 128;
			writeRegister(PSoCMD_MB_DRIVE, driveValue);
			break;
		default:
		break;
	}
}
//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void PSoCMD::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	switch (settings.commInterface)
	{

	case I2C_MODE:
		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.endTransmission();

		// request bytes from slave device
		Wire.requestFrom(settings.I2CAddress, length);
		while ( (Wire.available()) && (i < length))  // slave may send less than requested
		{
			c = Wire.read(); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		while ( i < length ) // slave may send less than requested
		{
			c = SPI.transfer(0x00); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}

}

uint8_t PSoCMD::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	switch (settings.commInterface) {

	case I2C_MODE:
		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.endTransmission();

		Wire.requestFrom(settings.I2CAddress, numBytes);
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
		// send a value of 0 to read the first byte returned:
		result = SPI.transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
	return result;
}

int16_t PSoCMD::readRegisterInt16( uint8_t offset )
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	return output;
}

void PSoCMD::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	switch (settings.commInterface)
	{
	case I2C_MODE:
		//Write the byte
		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.write(dataToWrite);
		Wire.endTransmission();
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
		break;

	default:
		break;
	}
}
