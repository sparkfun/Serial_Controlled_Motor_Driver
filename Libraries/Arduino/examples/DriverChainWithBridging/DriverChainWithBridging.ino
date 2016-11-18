/******************************************************************************
  DriverChainWithBridging.ino
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
//This example demonstrates some of the more advanced usage of the motor driver.
//It uses 3 motor drivers, with the master attached as SPI.  One slave is bridged,
//and the sketch test drives each motor. (There will be a break when the overtaken motor
//channel is activated.)
//
//This also shows how to count the number of connected slaves and report, as well as
//arbitrary register access.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

//#defines

//Variables
//***** Create the Motor Driver object*****//
SCMD myMotorDriver;

void setup()
{
  Serial.begin(9600);

  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//

  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5A; //config pattern "0101" on board for address 0x5A
  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  delay(2500); //Give the serial driver time to check for slaves

  //  initialize the driver and enable the motor outputs
  uint8_t tempReturnValue = myMotorDriver.begin();
  while ( tempReturnValue != 0xA9 )
  {
    Serial.print( "ID mismatch, read as 0x" );
    Serial.println( tempReturnValue, HEX );
    delay(500);
    tempReturnValue = myMotorDriver.begin();
  }
  Serial.println( "ID matches 0xA9" );

  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");

  //  Report number of slaves found
  uint8_t tempAddr = myMotorDriver.readRegister(SCMD_SLV_TOP_ADDR);
  if ( tempAddr >= START_SLAVE_ADDR )
  {
    Serial.print("Detected ");
    Serial.print(tempAddr - START_SLAVE_ADDR + 1); //Top address minus bottom address + 1 = number of slaves
    Serial.println(" slaves.");
  }
  else
  {
    Serial.println("No slaves detected");
  }

  //Configure bridging modes
  myMotorDriver.bridgingMode( 1, 1 ); //( DriverNum 1, bridged state = 1 )  This will bridge the first slave

  //Uncomment to set inversion

  //myMotorDriver.inversionMode(0, 1); //invert master, channel A
  //myMotorDriver.inversionMode(1, 1); //invert master, channel B
  //myMotorDriver.inversionMode(2, 1); //invert slave 1, channel A
  //    no need to configure motor 3, this position does nothing because the slave is bridged.
  //myMotorDriver.inversionMode(4, 1); //invert slave 2, channel A
  //myMotorDriver.inversionMode(5, 1); //invert slave 2, channel B

  //Enable the motors.
  myMotorDriver.enable();

  pinMode(8, INPUT_PULLUP);

}

void loop()
{
  //***** Operate the Motor Driver *****//
  //  This walks through all 34 motor positions driving them forward and back.
  //  It uses .setDrive( motorNum, direction, level ) to drive the motors.
  //
  //  Notice that when i == 3, no motor spins.  This position is made inactive by bridging the first slave.
  Serial.println("Now stepping through the motors.");
  for (int i = 0; i < 6; i++)
  {
    Serial.print("Driving motor ");
    Serial.println(i);

    myMotorDriver.setDrive( i, 1, 255); //Drive motor i forward at full speed
    delay(1000);
    myMotorDriver.setDrive( i, 0, 255); //Drive motor i backward at full speed
    delay(1000);
    myMotorDriver.setDrive( i, 1, 0);
  }
}