/******************************************************************************
RegisterRWTool.ino
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
//  This does the FW functional test 

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

//#defines
#define LEDPIN 13
#define PACKET_LENGTH 24

//Variables
SCMD myMotorDriver;

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
	//Set spare CS lines to high for alternate CS pin
	pinMode(9, OUTPUT);
	digitalWrite(9, 1);

	Serial.println("Starting sketch.");

	//Specify configuration for the driver
	//  Can be I2C_MODE, SPI_MODE
	myMotorDriver.settings.commInterface = I2C_MODE;
	//myMotorDriver.settings.commInterface = SPI_MODE;
	myMotorDriver.settings.I2CAddress = 0x61;
	myMotorDriver.settings.chipSelectPin = 9;
	delay(500);
	
	Serial.print("Starting driver... ID = 0x");
	Serial.println(myMotorDriver.begin(), HEX);
	
	Serial.print("Waiting for enumeration...");
	while( myMotorDriver.ready() == false );
	Serial.println("Done.");
	
	test();
	
	//Prepare the input buffer --
	//Fill packet with null, reset the pointer
	for( int i = 0; i < PACKET_LENGTH; i++ )
	{
		packet[i] = 0;
	}
	//reset the pointer
	packet_ptr = 0;
	
	//Splash help
	Serial.println();
	Serial.println("Address and data must be hex");
	Serial.println("'Wnnhh'  Write hh to nn (ex: W20F0 for drive, M0)");
	Serial.println("'Rnn'  Read register nn (ex: R01 to get ID)");
}

void loop()
{
	//Get and parse serial data
	while(Serial.available())
	{
		lastchar = Serial.read();
		if( ( packet_ptr < PACKET_LENGTH ) )//check for room in the packet
		{
			//put the char in the packet
			packet[packet_ptr] = lastchar;
			//advance the pointer
			packet_ptr++;
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
		
		//Packet acquired!  Now do the things
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
			Serial.println(myMotorDriver.readRegister( 0x01 ), HEX);
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

}

void test( void )
{
	cleanInit();

	//This sets numSlaves to the number of slaves, or 0 if no slaves
	uint8_t numSlaves = myMotorDriver.readRegister( SCMD_SLV_TOP_ADDR );
	if( numSlaves >= START_SLAVE_ADDR )
	{
		numSlaves = numSlaves - START_SLAVE_ADDR + 1;
	}
	else
	{
		numSlaves = 0;
	}

	//Test driving registers
	Serial.println("Test driving registers, all should read 0x80");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MA_DRIVE), HEX);
		Serial.print(", ");
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MB_DRIVE), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Test driving registers, all should read 0xAC");
	for( int i = SCMD_MA_DRIVE; i <= SCMD_S16B_DRIVE; i++ )
	{
		myMotorDriver.writeRegister(i, 0xAC);
	}
	delay(10);
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MA_DRIVE), HEX);
		Serial.print(", ");
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MB_DRIVE), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println();

	//Test inversion TX
	Serial.println("Test transfer of inversion");
	Serial.println("Initial inversion should be zero.");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_A_INVERT), HEX);
		Serial.print(", ");
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_B_INVERT), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering inversion pattern 0xA5E5D451");
	myMotorDriver.writeRegister(SCMD_INV_2_9, 0xA5);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_10_17, 0xE5);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_18_25, 0xD4);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_26_33, 0x51);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_A_INVERT), HEX);
		Serial.print(", ");
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_B_INVERT), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering inversion pattern 0x5A1A2BAE");
	myMotorDriver.writeRegister(SCMD_INV_2_9, 0x5A);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_10_17, 0x1A);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_18_25, 0x2B);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_26_33, 0xAE);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_A_INVERT), HEX);
		Serial.print(", ");
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_B_INVERT), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println();	

	//Test bridging TX
	Serial.println("Test transfer of bridging");
	Serial.println("Initial bridging should be zero.");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_BRIDGE), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering bridging pattern 0xA5E5");
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_L, 0xA5);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_H, 0xE5);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_BRIDGE), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering bridging pattern 0x5A1A");
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_L, 0x5A);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_H, 0x1A);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_MOTOR_A_INVERT), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println();	

	//Test Enable
	Serial.println("Test driving enables, all should read 0x00");
	Serial.print(myMotorDriver.readRegister(SCMD_DRIVER_ENABLE), HEX);
	Serial.print(", ");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_DRIVER_ENABLE), HEX);
		Serial.print(", ");

	}
	Serial.println();
	Serial.println("Test driving registers, all should read 0x01");
	myMotorDriver.writeRegister(SCMD_DRIVER_ENABLE, 0x01);
	while(myMotorDriver.busy());	
	Serial.print(myMotorDriver.readRegister(SCMD_DRIVER_ENABLE), HEX);
	Serial.print(", ");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_DRIVER_ENABLE), HEX);
		Serial.print(", ");

	}
	Serial.println();
	Serial.println("Test driving registers, all should read 0x00");
	myMotorDriver.writeRegister(SCMD_DRIVER_ENABLE, 0x00);
	while(myMotorDriver.busy());	
	Serial.print(myMotorDriver.readRegister(SCMD_DRIVER_ENABLE), HEX);
	Serial.print(", ");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_DRIVER_ENABLE), HEX);
		Serial.print(", ");

	}
	Serial.println();
	Serial.println();
	
	//Test user lock TX
	Serial.println("Test transfer of user lock");
	Serial.println("Initial locks should all be 0x5C.");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_LOCAL_USER_LOCK), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering key 0x1A");
	myMotorDriver.writeRegister(SCMD_USER_LOCK, 0x1A);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_LOCAL_USER_LOCK), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering key 0x5C");
	myMotorDriver.writeRegister(SCMD_USER_LOCK, 0x5C);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_LOCAL_USER_LOCK), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println();

	//Test master lock TX
	Serial.println("Test transfer of master lock");
	Serial.println("Initial locks should all be 0x00.");
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_LOCAL_MASTER_LOCK), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering key 0x1A");
	myMotorDriver.writeRegister(SCMD_MASTER_LOCK, 0x1A);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_LOCAL_MASTER_LOCK), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Entering key 0x9B");
	myMotorDriver.writeRegister(SCMD_MASTER_LOCK, 0x9B);
	while(myMotorDriver.busy());
	for( int i = 1; i <= numSlaves; i++ )
	{
		uint8_t slvAddr = START_SLAVE_ADDR + i - 1;
		Serial.print(myMotorDriver.readRemoteRegister(slvAddr, SCMD_LOCAL_MASTER_LOCK), HEX);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println();

	//Test local user lock behavior
	Serial.println("Do attempts at locked user access");
	myMotorDriver.writeRegister(SCMD_REG_RO_WRITE_CNT, 0x00);
	myMotorDriver.writeRegister(SCMD_USER_LOCK, 0x00);
	myMotorDriver.writeRegister(SCMD_MASTER_LOCK, 0x00);
	while(myMotorDriver.busy());
	Serial.print("Initial master SCMD_REG_RO_WRITE_CNT should equal 0.  It is ");
	Serial.println(myMotorDriver.readRegister(SCMD_REG_RO_WRITE_CNT));
	Serial.println("Now writing all protected addresses.");
	myMotorDriver.writeRegister(SCMD_MOTOR_A_INVERT, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_MOTOR_B_INVERT, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_LOCAL_MASTER_LOCK, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_FSAFE_CTRL, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_2_9, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_10_17, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_18_25, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_26_33, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_L, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_H, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_DRIVER_ENABLE, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_UPDATE_RATE, 0x0A);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_E_BUS_SPEED, 0x01);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_MASTER_LOCK, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_FSAFE_TIME, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_CONTROL_1, 0);
	while(myMotorDriver.busy());
	Serial.print("protected master SCMD_REG_RO_WRITE_CNT should equal 17.  It is ");
	Serial.println(myMotorDriver.readRegister(SCMD_REG_RO_WRITE_CNT));

	myMotorDriver.writeRegister(SCMD_REG_RO_WRITE_CNT, 0x00);
	Serial.println("Now user lock is inserted");
	myMotorDriver.writeRegister(SCMD_USER_LOCK, 0x5C);
	while(myMotorDriver.busy());
	Serial.println("Now writing all protected addresses.");
	myMotorDriver.writeRegister(SCMD_MOTOR_A_INVERT, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_MOTOR_B_INVERT, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_LOCAL_MASTER_LOCK, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_FSAFE_CTRL, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_2_9, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_10_17, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_18_25, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_INV_26_33, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_L, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_BRIDGE_SLV_H, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_DRIVER_ENABLE, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_UPDATE_RATE, 0x0A);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_E_BUS_SPEED, 0x01);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_MASTER_LOCK, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_FSAFE_TIME, 0);
	while(myMotorDriver.busy());
	myMotorDriver.writeRegister(SCMD_CONTROL_1, 0);
	while(myMotorDriver.busy());
	Serial.print("Unprotected master SCMD_REG_RO_WRITE_CNT should equal 0.  It is ");
	Serial.println(myMotorDriver.readRegister(SCMD_REG_RO_WRITE_CNT));
	Serial.println();
	Serial.println();

	cleanInit();	
	
}

void cleanInit( void )
{
	myMotorDriver.writeRegister( SCMD_CONTROL_1, SCMD_FULL_RESET_BIT ); //issue full reset
	Serial.print("Starting driver... ID = 0x");
	Serial.println(myMotorDriver.begin(), HEX);
	Serial.print("Waiting for enumeration...");
	while( myMotorDriver.ready() == false );
	delay(1000);
	Serial.println("Done.");	
}
//  This takes a char input in the hex range and converts to int
//    Accepts upper or lower case alpha chars
//    Outputs 0 with bad input
//
int char2hex(char charin)
{
  int hexout = 0;
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
