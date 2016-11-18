/******************************************************************************
  RegisterRWTool.ino
  Serial Controlled Motor Driver
  Marshall Taylor @ SparkFun Electronics
  Sept 15, 2016
  https://github.com/sparkfun/Serial_Controlled_Motor_Driver
  https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

  Resources:
  Uses Wire.h for i2c operation
  Uses SPI.h for SPI operation

  Development environment specifics:
  Arduino IDE 1.6.7
  Teensy loader 1.27

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/
//  This example allows reading and writing of driver registers by human input,
//    using the serial terminal at 115200 baud.  All numbers should be entered
//    as 2-digit hex.
//
//  To write a register:
//    "Wnnhh"  Write hh to nn
//
//  To read a register:
//    "Rnn"  Read register nn
//
//  This example does not verify that the SCMD is attached or ready before allowing communication!!!

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
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Starting sketch.");

  //Specify configuration for the driver
  //  Can be I2C_MODE, SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;
  myMotorDriver.settings.I2CAddress = 0x5A;
  myMotorDriver.settings.chipSelectPin = 10;
  delay(500);

  Serial.print("Starting driver... ID = 0x");
  Serial.println(myMotorDriver.begin(), HEX);

  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");

  //Prepare the input buffer --
  //Fill packet with null, reset the pointer
  for ( int i = 0; i < PACKET_LENGTH; i++ )
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
  while (Serial.available())
  {
    lastchar = Serial.read();
    if ( ( packet_ptr < PACKET_LENGTH ) ) //check for room in the packet
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
  if ((packet[packet_ptr - 1] == '\n') || (packet[packet_ptr - 1] == '\r'))
  {
    uint8_t addressTemp = 0;
    uint8_t dataTemp = 0;

    //Packet acquired!  Now do the things
    switch (packet[0])
    {
      case 'W':
      case 'w':
        //Check for hex
        if ( ishex(packet[1]) && ishex(packet[2]) && ishex(packet[3]) && ishex(packet[4]) )
        {
          addressTemp = char2hex(packet[1]) << 4 | char2hex(packet[2]);
          dataTemp = char2hex(packet[3]) << 4 | char2hex(packet[4]);
          myMotorDriver.writeRegister(addressTemp, dataTemp);
          Serial.print("(W) Addr: 0x");
          if ( addressTemp < 0x10) Serial.print('0');
          Serial.print(addressTemp, HEX);
          Serial.print(", Data: 0x");
          if ( dataTemp < 0x10) Serial.print('0');
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
        if ( ishex(packet[1]) && ishex(packet[2]) )
        {
          addressTemp = char2hex(packet[1]) << 4 | char2hex(packet[2]);
          dataTemp = myMotorDriver.readRegister(addressTemp);
          Serial.print("(R) Addr: 0x");
          if ( addressTemp < 0x10) Serial.print('0');
          Serial.print(addressTemp, HEX);
          Serial.print(", Data: 0x");
          if ( dataTemp < 0x10) Serial.print('0');
          Serial.print(dataTemp, HEX);
          Serial.print("\r\n");
        }
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
    for ( int i = 0; i < PACKET_LENGTH; i++ )
    {
      packet[i] = 0;
    }
    //reset the pointer
    packet_ptr = 0;
  }

  delay(100);
  digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );

}

//  This takes a char input in the hex range and converts to int
//    Accepts upper or lower case alpha chars
//    Outputs 0 with bad input
//
int char2hex(char charin)
{
  int hexout = 0;
  if (charin >= 0x30)
  {
    if (charin <= 0x39)
    {
      hexout = charin - 0x30;
    }
  }

  if (charin >= 0x61)
  {
    if (charin <= 0x66)
    {
      hexout = charin - 0x61 + 10;
    }
  }
  if (charin >= 0x41)
  {
    if (charin <= 0x46)
    {
      hexout = charin - 0x41 + 10;
    }
  }
  if (charin == ' ')
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
  if (charin >= 0x30)
  {
    if (charin <= 0x39)
    {
      answer = 1;
    }
  }

  if (charin >= 0x61)
  {
    if (charin <= 0x66)
    {
      answer = 1;
    }
  }
  if (charin >= 0x41)
  {
    if (charin <= 0x46)
    {
      answer = 1;
    }
  }
  return answer;
}