/******************************************************************************
MotorTest.ino
Serial Controlled Motor Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/Serial_Controlled_Motor_Driver

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
//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Note:  While using SPI, the defualt LEDPIN will not toggle
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

//#defines
#define LEDPIN 13

//Variables
//***** Create the Motor Driver object*****//
SCMD myMotorDriver;

void setup()
{
	Serial.begin(115200);
	pinMode(LEDPIN, OUTPUT);

	Serial.println("Starting sketch.");

	//***** Configure the Motor Driver's Settings *****//

	//  .commInter face can be I2C_MODE or SPI_MODE
	//myMotorDriver.settings.commInterface = I2C_MODE;
	myMotorDriver.settings.commInterface = SPI_MODE;

	//  set address if I2C configuration selected with the config jumpers
	myMotorDriver.settings.I2CAddress = 0x5A; //config pattern "0101" on board for address 0x5A
	//  set chip select if SPI selected with the config jumpers
	myMotorDriver.settings.chipSelectPin = 10;
	
	delay(500);
	
	//  initialize the driver and enable the motor outputs
	Serial.print("Starting driver... ID = 0x");
	Serial.println(myMotorDriver.begin(), HEX);
	Serial.println();
	
}

void loop()
{
	//***** Operate the Motor Driver *****//
	//  This walks through all 34 motor positions driving them forward and back.
	//  It uses .setDrive( motorNum, direction, level ) to drive the motors.
	
	Serial.println("Now stepping through the motors.");
	for(int i = 0; i < 34; i++)
	{
		digitalWrite( LEDPIN, 1 );
		myMotorDriver.setDrive( i, 1, 255); //Drive motor i forward at full speed
		delay(250);
		digitalWrite( LEDPIN, 0 );
		myMotorDriver.setDrive( i, 0, 255); //Drive motor i backward at full speed
		delay(250);
		myMotorDriver.setDrive( i, 1, 0);
	}
}
