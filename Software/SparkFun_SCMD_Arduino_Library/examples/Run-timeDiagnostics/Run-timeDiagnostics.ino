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
#define FAILSAFE_TIMEOUT_10MS_LSB 200
//Variables
SCMD myMotorDriver;
SCMDDiagnostics myDiagnostics;

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
	Serial.println();
	
	myMotorDriver.writeRegister(SCMD_FSAFE_TIME, 200);
	Serial.print("Failsafe time set to ");
	Serial.print(FAILSAFE_TIMEOUT_10MS_LSB * 10);
	Serial.println("ms!!!\n\n");

}

void loop()
{
	//User code
	Serial.println("Starting test.  Initial diagnostic counts:");
	printDiag(); // print initial/last
	printRemoteDiag();
	Serial.println("Resetting diagnostic counters in remote, now reads:");
	myMotorDriver.resetDiagnosticCounts();
	uint8_t topAddressTemp = myMotorDriver.readRegister( SCMD_SLV_TOP_ADDR );
	for( int i = START_SLAVE_ADDR; i <= topAddressTemp; i++ )
	{
		myMotorDriver.resetRemoteDiagnosticCounts((uint8_t)i);
	}
	delay(200);
	printDiag(); // print initial/last
	printRemoteDiag();

	Serial.println("Performing write read test for 5s, please induce errors");
	Serial.print("number of errors: ");
	Serial.println(readWriteTest());
	Serial.println();

	Serial.println("Final diagnostic counts:");
	printDiag();
	printRemoteDiag();
	Serial.println("Test done.  Enter any characters to restart the test.");
	Serial.println();
	Serial.println();
	while(Serial.available() == 0);
	delay(100); //wait for RX buffer to fill
	while(Serial.available())
	{
		//clear rx buffer
		Serial.read();
	}
		
	digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );

}

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
	Serial.print(" User port slave I2C ISR duration (peak, in us): ");
	Serial.println(myDiagnostics.UPORT_TIME);
	Serial.print(" Slave poll count register: ");
	Serial.println(myDiagnostics.SLV_POLL_CNT);
	Serial.print(" Expansion master I2C bus faults detected: ");
	Serial.println(myDiagnostics.MST_E_ERR);
	Serial.print(" Number of failsafes that have occured: ");
	Serial.println(myDiagnostics.FSAFE_FAULTS);
	Serial.print(" Out of range register accesses: ");
	Serial.println(myDiagnostics.REG_OOR_CNT);
	Serial.print(" RO writes attempted (locked regs): ");
	Serial.println(myDiagnostics.REG_RO_WRITE_CNT);
	Serial.println();	
}

void printRemoteDiag( void )
{
	uint8_t topAddressTemp = myMotorDriver.readRegister( SCMD_SLV_TOP_ADDR );
	for( int i = START_SLAVE_ADDR; i <= topAddressTemp; i++ )
	{
		Serial.print("Slave at address: 0x");
		Serial.println(i, HEX);
		myMotorDriver.getRemoteDiagnostics((uint8_t)i, myDiagnostics);
		Serial.print(" numberOfSlaves: ");
		Serial.print(" Expansion port I2C bus errors (reads): ");
		Serial.println(myDiagnostics.E_I2C_RD_ERR);
		Serial.print(" Expansion port I2C bus errors (writes): ");
		Serial.println(myDiagnostics.E_I2C_WR_ERR);
		Serial.print(" Expansion master I2C bus faults detected: ");
		Serial.println(myDiagnostics.MST_E_ERR);
		Serial.print(" Number of failsafes that have occured: ");
		Serial.println(myDiagnostics.FSAFE_FAULTS);
		Serial.print(" Out of range register accesses: ");
		Serial.println(myDiagnostics.REG_OOR_CNT);
		Serial.print(" RO writes attempted (locked regs): ");
		Serial.println(myDiagnostics.REG_RO_WRITE_CNT);
		Serial.println();		
	}
	
}

uint16_t readWriteTest( void )
{
	uint16_t errorCounts = 0;
	for( int i = 0; i < 2500; i++ )
	{
		myMotorDriver.writeRegister( 0x45, (uint8_t)i);
		delay(2);
		if( myMotorDriver.readRegister( 0x45 ) != (uint8_t)i )errorCounts++;
	}
	return errorCounts;
}
