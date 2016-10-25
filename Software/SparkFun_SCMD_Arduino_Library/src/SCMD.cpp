/******************************************************************************
SCMD.cpp
SCMD Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/Serial_Controlled_Motor_Driver

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
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values

#ifdef USE_ALT_I2C
#include <i2c_t3.h>
#define I2C_FAULT_TIMEOUT 1000 //in microseconds
#else
#include "Wire.h"
#endif

#include "SPI.h"

//****************************************************************************//
//
//  Constructor
//
//    Initalizes settings to default.
//
//****************************************************************************//
SCMD::SCMD( void )
{
	//Construct with these default settings if nothing is specified

	//Select interface mode
	settings.commInterface = I2C_MODE; //I2C_MODE or SPI_MODE
	//Select default I2C address.
	settings.I2CAddress = 0x58; //Ignored for SPI_MODE
	//Select default SPI CS pin.
	settings.chipSelectPin = 10; //Ignored for I2C_MODE

}


//****************************************************************************//
//
//  Configuration section
//
//    This uses the stored SensorSettings to start the IMU.
//  
//  Example usage:
//  //Configure for SPI mode using default CS pin of 10
//  mySensor.settings.commInterface = SPI_MODE; 
//  mySensor.begin();
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

//setDrive( ... )
//
//    Drive a motor at a level
//
//  uint8_t motorNum -- Motor number from 0 to 33
//  uint8_t direction -- 0 or 1 for forward and backward
//  uint8_t level -- 0 to 255 for drive strength
void SCMD::setDrive( uint8_t motorNum, uint8_t direction, uint8_t level )
{
	//convert to 7 bit
	level = level >> 1;
	int16_t driveValue; //use to build value to actually write to register
	
	//Make sure the motor number is valid
	if(motorNum < 34)
	{
		driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
		driveValue += 128;
		writeRegister(SCMD_MA_DRIVE + motorNum, driveValue);
	}
}

//inversionMode( ... )
//
//    Configure a motor's direction inversion
//
//  uint8_t motorNum -- Motor number from 0 to 33
//  uint8_t polarity -- 0 or 1 for default or inverted
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

//bridgingMode( ... )
//
//    Configure a driver's bridging state
//
//  uint8_t driverNum -- Number of driver.  Master is 0, slave 1 is 1, etc.  0 to 16
//  uint8_t bridged -- 0 or 1 for forward and backward
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

//getDiagnostics( ... )
//
//    Get diagnostic information from the master
//
//  SCMDDiagnostics &diagObjectReference -- Object to contain returned data
void SCMD::getDiagnostics( SCMDDiagnostics &diagObjectReference )
{
	diagObjectReference.U_I2C_RD_ERR = readRegister( SCMD_U_I2C_RD_ERR );
	diagObjectReference.U_I2C_WR_ERR = readRegister( SCMD_U_I2C_WR_ERR );
	diagObjectReference.U_BUF_DUMPED = readRegister( SCMD_U_BUF_DUMPED );
	diagObjectReference.E_I2C_RD_ERR = readRegister( SCMD_E_I2C_RD_ERR );
	diagObjectReference.E_I2C_WR_ERR = readRegister( SCMD_E_I2C_WR_ERR );
	diagObjectReference.UPORT_TIME = readRegister( SCMD_UPORT_TIME );
	diagObjectReference.SLV_POLL_CNT = readRegister( SCMD_SLV_POLL_CNT );
	//Count slaves
	uint8_t topAddr = readRegister( SCMD_SLV_TOP_ADDR );
	if( (topAddr >= START_SLAVE_ADDR) && (topAddr < (START_SLAVE_ADDR + 16)))
	{
		//in valid range
		diagObjectReference.numberOfSlaves = topAddr - START_SLAVE_ADDR + 1;
	}
	diagObjectReference.MST_E_ERR = readRegister( SCMD_MST_E_ERR );
	diagObjectReference.FSAFE_FAULTS = readRegister( SCMD_FSAFE_FAULTS );
	diagObjectReference.REG_OOR_CNT = readRegister( SCMD_REG_OOR_CNT );
	diagObjectReference.REG_RO_WRITE_CNT = readRegister( SCMD_REG_RO_WRITE_CNT );
	
}

//getRemoteDiagnostics( ... )
//
//    Get diagnostic information from a slave
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
//  SCMDDiagnostics &diagObjectReference -- Object to contain returned data
void SCMD::getRemoteDiagnostics( uint8_t address, SCMDDiagnostics &diagObjectReference )
{
	diagObjectReference.U_I2C_RD_ERR = readRemoteRegister( address, SCMD_U_I2C_RD_ERR );
	diagObjectReference.U_I2C_WR_ERR = readRemoteRegister( address, SCMD_U_I2C_WR_ERR );
	diagObjectReference.U_BUF_DUMPED = readRemoteRegister( address, SCMD_U_BUF_DUMPED );
	diagObjectReference.E_I2C_RD_ERR = readRemoteRegister( address, SCMD_E_I2C_RD_ERR );
	diagObjectReference.E_I2C_WR_ERR = readRemoteRegister( address, SCMD_E_I2C_WR_ERR );
	diagObjectReference.UPORT_TIME = readRemoteRegister( address, SCMD_UPORT_TIME );
	diagObjectReference.SLV_POLL_CNT = readRemoteRegister( address, SCMD_SLV_POLL_CNT );
	//Count slaves
	uint8_t topAddr = readRemoteRegister( address, SCMD_SLV_TOP_ADDR );
	if( (topAddr >= START_SLAVE_ADDR) && (topAddr < (START_SLAVE_ADDR + 16)))
	{
		//in valid range
		diagObjectReference.numberOfSlaves = topAddr - START_SLAVE_ADDR + 1;
	}
	diagObjectReference.MST_E_ERR = readRemoteRegister( address, SCMD_MST_E_ERR );
	diagObjectReference.FSAFE_FAULTS = readRemoteRegister( address, SCMD_FSAFE_FAULTS );
	diagObjectReference.REG_OOR_CNT = readRemoteRegister( address, SCMD_REG_OOR_CNT );
	diagObjectReference.REG_RO_WRITE_CNT = readRemoteRegister( address, SCMD_REG_RO_WRITE_CNT );
	
}

//resetDiagnosticCounts( ... )
//
//    Reset the master's diagnostic counters
//
void SCMD::resetDiagnosticCounts( void )
{
	writeRegister( SCMD_U_I2C_RD_ERR, 0 );
	writeRegister( SCMD_U_I2C_WR_ERR, 0 );
	writeRegister( SCMD_U_BUF_DUMPED, 0 );
	writeRegister( SCMD_E_I2C_RD_ERR, 0 );
	writeRegister( SCMD_E_I2C_WR_ERR, 0 );
	//Clear uport time
	writeRegister( SCMD_UPORT_TIME, 0 );
	writeRegister( SCMD_MST_E_ERR, 0 );
	writeRegister( SCMD_FSAFE_FAULTS, 0 );
	writeRegister( SCMD_REG_OOR_CNT, 0 );
	writeRegister( SCMD_REG_RO_WRITE_CNT, 0 );
	
}

//resetRemoteDiagnosticCounts( ... )
//
//    Reset a slave's diagnostic counters
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
void SCMD::resetRemoteDiagnosticCounts( uint8_t address )
{
	writeRemoteRegister( address, SCMD_U_I2C_RD_ERR, 0 );
	writeRemoteRegister( address, SCMD_U_I2C_WR_ERR, 0 );
	writeRemoteRegister( address, SCMD_U_BUF_DUMPED, 0 );
	writeRemoteRegister( address, SCMD_E_I2C_RD_ERR, 0 );
	writeRemoteRegister( address, SCMD_E_I2C_WR_ERR, 0 );
	//Clear uport time
	writeRemoteRegister( address, SCMD_UPORT_TIME, 0 );
	writeRemoteRegister( address, SCMD_ID, 0 );
	writeRemoteRegister( address, SCMD_FSAFE_FAULTS, 0 );
	writeRemoteRegister( address, SCMD_REG_OOR_CNT, 0 );
	writeRemoteRegister( address, SCMD_REG_RO_WRITE_CNT, 0 );	
}

//****************************************************************************//
//
//  Register Access Functions
//
//****************************************************************************//

//readRegister( ... )
//
//    Read data from the master
//
//  uint8_t offset -- Address of data to read.  Can be 0x00 to 0x7F
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
		
		//For loop delay for 16 Mhz CPU
		for(volatile int i = 0; i < 25; i++);
		
		//do a dummy read
		digitalWrite(settings.chipSelectPin, LOW);
		// send a value of 80 to read the first byte returned:
		result = SPI.transfer(0x80);
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		
		//For loop delay for 16 Mhz CPU
		for(volatile int i = 0; i < 25; i++);
		
		break;

	default:
		break;
	}
	return result;
}

//writeRegister( ... )
//
//    Write data to the master
//
//  uint8_t offset -- Address of data to write.  Can be 0x00 to 0x7F
//  uint8_t dataToWrite -- Data to write.
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

//readRegister( ... )
//
//    Read data from a slave.  Note that this waits 5ms for slave data to be aquired
//  before making the final read.
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
//  uint8_t offset -- Address of data to read.  Can be 0x00 to 0x7F
uint8_t SCMD::readRemoteRegister(uint8_t address, uint8_t offset)
{
	writeRegister(SCMD_REM_ADDR, address);
	writeRegister(SCMD_REM_OFFSET, offset);
	writeRegister(SCMD_REM_READ, 1);
	delay(5);
	return readRegister(SCMD_REM_DATA_RD);
	
}

//writeRegister( ... )
//
//    Write data from a slave
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
//  uint8_t offset -- Address of data to write.  Can be 0x00 to 0x7F
//  uint8_t dataToWrite -- Data to write.
void SCMD::writeRemoteRegister(uint8_t address, uint8_t offset, uint8_t dataToWrite)
{
	writeRegister(SCMD_REM_ADDR, address);
	writeRegister(SCMD_REM_OFFSET, offset);
	writeRegister(SCMD_REM_DATA_WR, dataToWrite);
	writeRegister(SCMD_REM_WRITE, 1);
	
}
