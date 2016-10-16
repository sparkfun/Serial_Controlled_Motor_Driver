//To write a register:
// "Wnnhh"  Write hh to nn
//
//To read a register:
// "Rnn"  Read register nn
//
//  Pin 4 can be used to trigger a scope, and placed at the relevant location

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h"
#include "Wire.h"

//#defines
#define LEDPIN 13

//Variables
SCMD myMotorDriver;

//Serial packet defines
#define PACKET_LENGTH 24

char lastchar;
char packet[PACKET_LENGTH];
char lastPacket[PACKET_LENGTH];
char packetPending = 0;

char packet_ptr;

//Function prototypes
int char2hex(char);
int ishex(char);

void setup()
{
	Serial.begin(115200);
	pinMode(LEDPIN, OUTPUT);

	pinMode(4, OUTPUT);
	digitalWrite(4, 0);
	
	Serial.println("Starting sketch.");

	//Specify configuration for the driver
	//  Can be I2C_MODE, SPI_MODE
	myMotorDriver.settings.commInterface = I2C_MODE;
	//myMotorDriver.settings.commInterface = SPI_MODE;
	myMotorDriver.settings.I2CAddress = 0x5A;
	myMotorDriver.settings.chipSelectPin = 10;
	delay(500);
	
	Serial.print("Starting driver... ID = 0x");
	digitalWrite(4, 1);
	digitalWrite(4, 0);
	digitalWrite(4, 1);
	Serial.println(myMotorDriver.begin(), HEX);
	
	//Fill packet with null, reset the pointer
	for( int i = 0; i < PACKET_LENGTH; i++ )
	{
	packet[i] = 0;
	}
	//reset the pointer
	packet_ptr = 0;
	
	Serial.println();
	Serial.println("Use capital letters for 'R' and 'W'");
	Serial.println("Address and data must be upper case hex");
	Serial.println("'Wnnhh'  Write hh to nn (ex: W20F0 for drive, M0)");
	Serial.println("'Rnn'  Read register nn (ex: R01 to get ID)");
}

void loop()
{
	//User code
	while(Serial.available())
	{
		lastchar = Serial.read();
		if( ( packet_ptr < PACKET_LENGTH ) )//check for room in the packet
		{
			//put the char in the packet
			packet[packet_ptr] = lastchar;
			//advance the pointer
			packet_ptr++;
			//turn on LED
		}
		else
		{
			//Just overwrite to the last position
			packet[PACKET_LENGTH - 1] = lastchar;
		}
	}
	if((packet[packet_ptr - 1] == '\n')||(packet[packet_ptr - 1] == '\r'))
	{
		uint8_t addressTemp = 0;
		uint8_t dataTemp = 0;
		
		//do the things
        switch(packet[0])
        {
            case 'W':
            case 'w':
            //Check for hex
            if( ishex(packet[1])&&ishex(packet[2])&&ishex(packet[3])&&ishex(packet[4]) )
            {
                addressTemp = char2hex(packet[1]) << 4 | char2hex(packet[2]);
                dataTemp = char2hex(packet[3]) << 4 | char2hex(packet[4]);
                myMotorDriver.writeRegister(addressTemp, dataTemp);
                Serial.print("(W) Addr: 0x");
				if( addressTemp < 0x10) Serial.print('0');
				Serial.print(addressTemp, HEX);
				Serial.print(", Data: 0x");
				if( dataTemp < 0x10) Serial.print('0');
				Serial.print(dataTemp, HEX);
				Serial.print("\r\n");
            }
            else
            {
                //is not all hex
                Serial.print("Not Hex\r\n");
            }                    
            break;
            case 'R':
            case 'r':
            //Check for hex
            if( ishex(packet[1])&&ishex(packet[2]) )
            {
                addressTemp = char2hex(packet[1]) << 4 | char2hex(packet[2]);
                dataTemp = myMotorDriver.readRegister(addressTemp);
                Serial.print("(R) Addr: 0x");
				if( addressTemp < 0x10) Serial.print('0');
				Serial.print(addressTemp, HEX);
				Serial.print(", Data: 0x");
				if( dataTemp < 0x10) Serial.print('0');
				Serial.print(dataTemp, HEX);
				Serial.print("\r\n");            }
            else
            {
                //is not all hex
                Serial.print("Not Hex\r\n");
            }
            break;
            default:
            Serial.print("Invalid Command\r\n");
            break;
		}
		
		//Fill packet with null, reset the pointer
		for( int i = 0; i < PACKET_LENGTH; i++ )
		{
		packet[i] = 0;
		}
		//reset the pointer
		packet_ptr = 0;
	}
	

	delay(100);
	digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );
	//myMotorDriver.inversionMode(0 + (2*currentSwitchValues[3]), currentSwitchValues[i]);
}

// This takes a char input and converts to int
//
// The output will be an int if the char
//  is a number
//  is A-F
//  is a-f
//
// Otherwise, the output is zero.
//
int char2hex(char charin)
{
  int hexout;
  if(charin >= 0x30)
  {
    if(charin <= 0x39)
    {
      hexout = charin - 0x30;
    }
  }

  if(charin >= 0x61)
  {
    if(charin <= 0x66)
    {
      hexout = charin - 0x61 + 10;
    }
  }
  if(charin >= 0x41)
  {
    if(charin <= 0x46)
    {
      hexout = charin - 0x41 + 10;
    }
  }
  if(charin == ' ')
  {
	  hexout = 10;
  }
  return hexout;
}

//Returns 1 if the input char is in the hex range
int ishex(char charin)
{
  int answer;
  answer = 0;
  if(charin >= 0x30)
  {
    if(charin <= 0x39)
    {
      answer = 1;
    }
  }

  if(charin >= 0x61)
  {
    if(charin <= 0x66)
    {
      answer = 1;
    }
  }
  if(charin >= 0x41)
  {
    if(charin <= 0x46)
    {
      answer = 1;
    }
  }
  return answer;
}
