/******************************************************************************
  TwoMasterMotorTest.ino
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
//This example is a double master configuration of the MotorTest example.
//Attach two chains of motor drivers to the same bus.
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

//#defines
#define LEDPIN 13

//***** Create the Motor Driver objects*****//
SCMD myMotorDriverA;
SCMD myMotorDriverB;

uint8_t motorCountA = 34;
uint8_t motorCountB = 34;

void setup()
{
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//

  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriverA.settings.commInterface = I2C_MODE;
  //myMotorDriverA.settings.commInterface = SPI_MODE;
  myMotorDriverA.settings.I2CAddress = 0x5A; //config pattern "0101" on board for address 0x5A
  myMotorDriverA.settings.chipSelectPin = 10;

  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriverB.settings.commInterface = I2C_MODE;
  //myMotorDriverB.settings.commInterface = SPI_MODE;
  myMotorDriverB.settings.I2CAddress = 0x61; //config pattern "1100" on board for address 0x61
  myMotorDriverB.settings.chipSelectPin = 9;

  delay(500);

  //  initialize the driver and enable the motor outputs
  Serial.print("'A' Read ID = 0x");
  Serial.println(myMotorDriverA.begin(), HEX);

  //  initialize the driver and enable the motor outputs
  while ( myMotorDriverA.begin() != 0xA9 )
  {
    Serial.println( "'A' ID mismatch, trying again" );
    delay(50);
  }
  Serial.println( "ID matches 0xA9" );

  Serial.print("Waiting for ready");
  while ( myMotorDriverA.ready() == false );
  Serial.println(" 'A' Done.");

  while ( myMotorDriverA.busy() );
  myMotorDriverA.enable();

  Serial.println();



  //  initialize the driver and enable the motor outputs
  Serial.print("'B' Read ID = 0x");
  Serial.println(myMotorDriverB.begin(), HEX);

  //  initialize the driver and enable the motor outputs
  while ( myMotorDriverB.begin() != 0xA9 )
  {
    Serial.println( "'B' ID mismatch, trying again" );
    delay(50);
  }
  Serial.println( "ID matches 0xA9" );

  Serial.print("Waiting for ready");
  while ( myMotorDriverB.ready() == false );
  Serial.println(" 'B' Done.");

  while ( myMotorDriverB.busy() );
  myMotorDriverB.enable();

  motorCountA = myMotorDriverA.readRegister( SCMD_SLV_TOP_ADDR );
  if ((motorCountA >= START_SLAVE_ADDR) && (motorCountA <= MAX_SLAVE_ADDR))
  {
    motorCountA = 4 + 2 * ( motorCountA - START_SLAVE_ADDR );
  }
  else
  {
    motorCountA = 2;
  }

  motorCountB = myMotorDriverB.readRegister( SCMD_SLV_TOP_ADDR );
  if ((motorCountB >= START_SLAVE_ADDR) && (motorCountB <= MAX_SLAVE_ADDR))
  {
    motorCountB = 4 + 2 * ( motorCountB - START_SLAVE_ADDR );
  }
  else
  {
    motorCountB = 2;
  }

  Serial.println();

}

void loop()
{
  //***** Operate the Motor Driver *****//
  //  This walks through all 34 motor positions driving them forward and back.
  //  It uses .setDrive( motorNum, direction, level ) to drive the motors.

  Serial.println("Now stepping through the motors.");
  for (int i = 0; i < motorCountA; i++)
  {
    digitalWrite( LEDPIN, 1 );
    myMotorDriverA.setDrive( i, 1, 255); //Drive motor i forward at full speed
    delay(250);
    digitalWrite( LEDPIN, 0 );
    myMotorDriverA.setDrive( i, 0, 255); //Drive motor i backward at full speed
    delay(250);
    myMotorDriverA.setDrive( i, 1, 0);
  }

  Serial.println("Now stepping through the motors.");
  for (int i = 0; i < motorCountB; i++)
  {
    digitalWrite( LEDPIN, 1 );
    myMotorDriverB.setDrive( i, 1, 255); //Drive motor i forward at full speed
    delay(250);
    digitalWrite( LEDPIN, 0 );
    myMotorDriverB.setDrive( i, 0, 255); //Drive motor i backward at full speed
    delay(250);
    myMotorDriverB.setDrive( i, 1, 0);
  }
}