/******************************************************************************
  Run-timeDiagnostics.ino
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
//  This example reports diagnostic information before and after a fixed interval.
//  It uses a SCMDDiagnostics object to get data from the library, and leaves the drivers in a
//  2 second failsafe mode.
//
//  Some members of SCMDDiagnostic are unused depending on master/slave status of call, and will
//  return invalid values.
//
//  The example does the following basic actions
//    * Configures Driver
//    * Reports current diagnostic values
//    * Clears and reports new diagnostic valuse
//    * Reads, then writes to SCMD's ram for about 5 seconds
//    * Reports diagnostic values again.
//    * Waits for user to restart the test.
//
//  Try grounding the lines of the I2C bus to see what happens.  Watch the slaves' LEDs for activity blips
//  and see if the bus recovers or not, as well as the types of failures recorded.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

//#defines
#define LEDPIN 13
#define FAILSAFE_TIMEOUT_10MS_LSB 200 //Configure for 2 second timeout between serial reception

//Variables
//***** Create the Motor Driver objects*****//
SCMD myMotorDriver;
SCMDDiagnostics myDiagnostics;

void setup()
{
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  //Set possible CS lines to high to start in good state
  pinMode(9, OUTPUT);
  digitalWrite(9, 1);
  pinMode(10, OUTPUT);
  digitalWrite(10, 1);

  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//

  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5A;
  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;
  delay(500);

  //  initialize the driver and enable the motor outputs
  Serial.print("Read ID = 0x");
  Serial.println(myMotorDriver.begin(), HEX);

  //  initialize the driver
  while ( myMotorDriver.begin() != 0xA9 )
  {
    Serial.println( "ID mismatch, trying again" );
    delay(50);
  }
  Serial.println( "ID matches 0xA9" );

  Serial.print("Waiting for ready");
  while ( myMotorDriver.ready() == false );

  Serial.println(" Done.");
  Serial.println( "Re-enumerating slaves");
  myMotorDriver.writeRegister( SCMD_CONTROL_1, SCMD_FULL_RESET_BIT );

  Serial.print("Waiting for ready");
  while ( myMotorDriver.ready() == false );
  Serial.println(" Done.");

  Serial.println();

  //  Configure the failsafe time and display
  myMotorDriver.writeRegister(SCMD_USER_LOCK, USER_LOCK_KEY);
  while ( myMotorDriver.busy() );
  myMotorDriver.writeRegister(SCMD_MASTER_LOCK, MASTER_LOCK_KEY);
  while ( myMotorDriver.busy() );
  myMotorDriver.writeRegister(SCMD_FSAFE_TIME, FAILSAFE_TIMEOUT_10MS_LSB);
  Serial.print("Failsafe time set to ");
  Serial.print(FAILSAFE_TIMEOUT_10MS_LSB * 10);
  Serial.println("ms!!!\n\n");

}

void loop()
{
  //***** Operate the Motor Driver *****//

  //Reports current diagnostic values
  Serial.println("Starting test.  Initial diagnostic counts:");
  printDiag(); // print initial/last
  printRemoteDiag();

  //Clears and reports new diagnostic valuse
  Serial.println("Resetting diagnostic counters in remote, now reads:");
  myMotorDriver.resetDiagnosticCounts();

  uint8_t topAddressTemp = myMotorDriver.readRegister( SCMD_SLV_TOP_ADDR );
  //START_SLAVE_ADDR is a #define common to this library and the SCMD firmware.
  //  It is the address of the first slave, 0x50.  Here, it's compared to
  //  the motor driver's top slave address to know how to reset each slave
  //  in a loop.
  for ( int i = START_SLAVE_ADDR; i <= topAddressTemp; i++ )
  {
    myMotorDriver.resetRemoteDiagnosticCounts((uint8_t)i);
  }
  delay(200);

  printDiag();
  printRemoteDiag();

  //Reads, then writes to SCMD's ram for about 5 seconds
  Serial.println("Performing write read test for 5s, please induce errors");
  Serial.print("number of errors: ");
  Serial.println(readWriteTest());
  Serial.println();

  //Reports diagnostic values again.
  Serial.println("Final diagnostic counts:");
  printDiag();
  printRemoteDiag();

  //Waits for user to restart the test
  Serial.println("Test done.  Enter any characters to restart the test.");
  Serial.println();
  Serial.println();
  while (Serial.available() == 0);
  delay(100); //wait for RX buffer to fill
  while (Serial.available())
  {
    //clear rx buffer
    Serial.read();
  }

  digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );

}

//  This function gets the diagnostic information from the master and prints
//    it to the serial terminal.
void printDiag( void )
{
  Serial.println("Master");
  myMotorDriver.getDiagnostics(myDiagnostics);
  Serial.print(" numberOfSlaves: ");
  Serial.println(myDiagnostics.numberOfSlaves);
  Serial.print(" User port I2C bus errors (reads): ");
  Serial.println(myDiagnostics.U_I2C_RD_ERR);
  Serial.print(" User port I2C bus errors (writes): ");
  Serial.println(myDiagnostics.U_I2C_WR_ERR);
  Serial.print(" Times user port buffers were discarded: ");
  Serial.println(myDiagnostics.U_BUF_DUMPED);
  Serial.print(" Expansion port I2C bus errors (reads): ");
  Serial.println(myDiagnostics.E_I2C_RD_ERR);
  Serial.print(" Expansion port I2C bus errors (writes): ");
  Serial.println(myDiagnostics.E_I2C_WR_ERR);
  Serial.print(" Peak main loop operation time (in ms): ");
  Serial.println( (float)myDiagnostics.LOOP_TIME / 10.0);
  Serial.print(" Slave poll count register: ");
  Serial.println(myDiagnostics.SLV_POLL_CNT);
  Serial.print(" Expansion master I2C bus faults detected: ");
  Serial.println(myDiagnostics.MST_E_ERR);
  Serial.print(" Last master status register: 0x");
  Serial.println(myDiagnostics.MST_E_STATUS, HEX);
  Serial.print(" Number of failsafes that have occured: ");
  Serial.println(myDiagnostics.FSAFE_FAULTS);
  Serial.print(" Out of range register accesses: ");
  Serial.println(myDiagnostics.REG_OOR_CNT);
  Serial.print(" RO writes attempted (locked regs): ");
  Serial.println(myDiagnostics.REG_RO_WRITE_CNT);
  Serial.println();
}

//  This function gets the diagnostic information from the slaves and prints
//    it to the serial terminal.
void printRemoteDiag( void )
{
  uint8_t topAddressTemp = myMotorDriver.readRegister( SCMD_SLV_TOP_ADDR );
  for ( int i = START_SLAVE_ADDR; i <= topAddressTemp; i++ )
  {
    Serial.print("Slave at address: 0x");
    Serial.println(i, HEX);
    myMotorDriver.getRemoteDiagnostics((uint8_t)i, myDiagnostics);
    Serial.print(" Expansion port I2C bus errors (reads): ");
    Serial.println(myDiagnostics.E_I2C_RD_ERR);
    Serial.print(" Expansion port I2C bus errors (writes): ");
    Serial.println(myDiagnostics.E_I2C_WR_ERR);
    Serial.print(" Peak main loop operation time (in ms): ");
    Serial.println( (float)myDiagnostics.LOOP_TIME / 10.0);
    Serial.print(" Number of failsafes that have occured: ");
    Serial.println(myDiagnostics.FSAFE_FAULTS);
    Serial.print(" Out of range register accesses: ");
    Serial.println(myDiagnostics.REG_OOR_CNT);
    Serial.print(" RO writes attempted (locked regs): ");
    Serial.println(myDiagnostics.REG_RO_WRITE_CNT);
    Serial.println();
  }

}

//  This function runs a pre-determined number of Write then Read operations
//    to an unused register in the master SCMD.  It counts numbers of read operations
//    that did not match the previous write data and returns it.
uint16_t readWriteTest( void )
{
  uint16_t errorCounts = 0;
  for ( int i = 0; i < 2500; i++ )
  {
    myMotorDriver.writeRegister( 0x45, (uint8_t)i);
    delay(2);
    if ( myMotorDriver.readRegister( 0x45 ) != (uint8_t)i )errorCounts++;
  }
  return errorCounts;
}