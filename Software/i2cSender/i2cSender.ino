// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#define LEDPIN 13

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
}

byte x = 0;

void loop()
{
	uint8_t result = 0;
	//Wire.beginTransmission(0x58);
	//Wire.write(0xE9);
	//if( Wire.endTransmission() != 0 )
	//{
		//error thing
	//}
	Wire.requestFrom(0x58, 1);
	while ( Wire.available() ) // slave may send less than requested
	{
		result = Wire.read(); // receive a byte as a proper uint8_t
	}
	Serial.print("0x");
	Serial.println(result, HEX);

	digitalWrite(LEDPIN, digitalRead(LEDPIN)^0x01); // set the LED on
    delay(1000);
	
	Wire.beginTransmission(0x58);
	Wire.write(0xE9);
	if( Wire.endTransmission() != 0 )
	{
		//error thing
	}
	
	digitalWrite(LEDPIN, digitalRead(LEDPIN)^0x01); // set the LED on
	delay(1000);

	Wire.beginTransmission(0x58);
	Wire.write(0xEA);
	if( Wire.endTransmission() != 0 )
	{
		//error thing
	}
	
	digitalWrite(LEDPIN, digitalRead(LEDPIN)^0x01); // set the LED on
	delay(1000);
	
}