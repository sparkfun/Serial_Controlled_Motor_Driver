//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Note:  While using SPI, the defualt LEDPIN will not toggle


#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h"
#include "Wire.h"

//#defines
#define LEDPIN 13

//Variables
SCMD myMotorDriver;

void setup()
{
	Serial.begin(115200);
	pinMode(LEDPIN, OUTPUT);

	Serial.println("Starting sketch.");

	//Specify configuration for the driver
	//  Can be I2C_MODE, SPI_MODE
	//myMotorDriver.settings.commInterface = I2C_MODE;
	myMotorDriver.settings.commInterface = SPI_MODE;
	
	myMotorDriver.settings.I2CAddress = 0x5A;
	myMotorDriver.settings.chipSelectPin = 10;
	delay(500);
	
	Serial.print("Starting driver... ID = 0x");
	Serial.println(myMotorDriver.begin(), HEX);
	
	Serial.println();
	Serial.println("Now stepping through the motors.");
}

void loop()
{
//	digitalWrite( LEDPIN, 1 );
//	while(1);
	//User code
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
