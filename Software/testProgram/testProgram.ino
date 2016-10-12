//This example sends knob inputs out to motors 1, 2, 3, 4 via I2C
//It returns diagnostic information



//HOW TO OPERATE
//  Make TimerClass objects for each thing that needs periodic service
//  pass the interval of the period in ticks
//
//  Set maxInterval to rollover rate
//  Set maxTimer to the max foreseen interval of any timer.
//  maxTimer + maxInterval = max countable value.
//#include "HOS_char.h"
#include "SCMD.h"
#include "SCMD_config.h"

//Globals
uint16_t maxTimer = 60000;
uint16_t maxInterval = 2000;

#define LEDPIN 13
#include "timerModule.h"
#include "stdint.h"

//Not used by this sketch but dependant on one 
#include "Wire.h"

//If 328p based, do this
#ifdef __AVR__
#include <Arduino.h>
#endif

//If Teensy 3.1 based, do this
#ifdef __MK20DX256__
IntervalTimer myTimer;
#endif

//Globals
TimerClass debugTimer( 3000 ); //milliseconds between calls
TimerClass outputTimer( 10 );  //Read knobs and drive motors

//Required for timers:
uint16_t msTicks = 0;
uint8_t msTicksLocked = 1; //start locked out

uint8_t lastSwitchValues[5] = {1,1,1,1,1};
uint8_t currentSwitchValues[5] = {0,0,0,0,0};

SCMD myMotorDriver;

//Diagnostic
uint16_t durationRecord[1] = {0};

void setup()
{
	Serial.begin(115200);
	pinMode(LEDPIN, OUTPUT);

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);

	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);
    pinMode(10, INPUT_PULLUP);
	pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
	
	pinMode(4, OUTPUT);
	digitalWrite(4, 0);
	
	delay(1500);
	Serial.println("Starting sketch.");

	//Specify configuration for the driver
	//  Can be I2C_MODE, SPI_MODE
	myMotorDriver.settings.commInterface = I2C_MODE;
	//myMotorDriver.settings.commInterface = SPI_MODE;
	myMotorDriver.settings.I2CAddress = 0x5A;
	//myMotorDriver.settings.chipSelectPin = 10;

	Serial.println("Starting timer.");
	//If 328p based, do this -- Creates interrupt based timer
#ifdef __AVR__
	// initialize Timer1
	cli();          // disable global interrupts
	TCCR1A = 0;     // set entire TCCR1A register to 0
	TCCR1B = 0;     // same for TCCR1B

	// set compare match register to desired timer count:
	OCR1A = 16000;

	// turn on CTC mode:
	TCCR1B |= (1 << WGM12);

	// Set CS10 and CS12 bits for 1 prescaler:
	TCCR1B |= (1 << CS10);

	// enable timer compare interrupt:
	TIMSK1 |= (1 << OCIE1A);

	// enable global interrupts:
	sei();
#endif

	//If Teensy 3.1 based, do this
#ifdef __MK20DX256__
	// initialize IntervalTimer
	myTimer.begin(serviceMS, 1000);  // serviceMS to run every 0.001 seconds
#endif
	
	Serial.print("Starting driver... ID = 0x");
	delay(500);
	digitalWrite(4, 1);
	digitalWrite(4, 0);
	digitalWrite(4, 1);
	Serial.println(myMotorDriver.begin(), HEX);
//	while(1);
	myMotorDriver.bridgingMode(0,1);
	myMotorDriver.inversionMode(2,1);
}

void loop()
{
	//Update the timers, but only once per interrupt
	if( msTicksLocked == 0 )
	{
		//**Copy to make a new timer******************//  
		//msTimerA.update(msTicks);
		debugTimer.update(msTicks);
		outputTimer.update(msTicks);
		
		//Done?  Lock it back up
		msTicksLocked = 1;
	}  //The ISR will unlock.

	//Check the timers
	if(debugTimer.flagStatus() == PENDING)
	{
		//User code
		digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );
		Serial.println("\nOOoo..   Debug Interval Timer   ..ooOO");
		Serial.print("ID: 0x");
		Serial.println(myMotorDriver.readRegister(SCMD_ID), HEX);
		
		Serial.println("MEMORY DUMP:");
		for( int i = 0x17; i < 0x1A; i++)
		{
			Serial.print("0x");
			Serial.print(i, HEX);
			Serial.print(": 0x");
			Serial.println(myMotorDriver.readRegister(i), HEX);
		}		
		Serial.println("MEMORY DUMP:");
		for( int i = 0x60; i < 0x68; i++)
		{
			Serial.print("0x");
			Serial.print(i, HEX);
			Serial.print(": 0x");
			Serial.println(myMotorDriver.readRegister(i), HEX);
		}
		//Retreive diagnostics and store in SCMDDiagnostics struct
		SCMDDiagnostics myDiagnostics;
		myMotorDriver.getDiagnostics(myDiagnostics);
		//Do something with the data
		Serial.println("DIAGNSOTICS:");
		Serial.print("outputTimer Fn Duration Peak: ");
			Serial.print(durationRecord[0]);
			Serial.println("ms");
			durationRecord[0] = 0;
		Serial.print("I2C routine duration: ");
			Serial.print(myDiagnostics.userPortI2CTime);
			Serial.println("us");
		Serial.print("I2C RX error cnt: ");
			Serial.println(myDiagnostics.rxErrorCount);
		Serial.print("I2C WR error cnt: ");
			Serial.println(myDiagnostics.txErrorCount);
		Serial.print("Number of slaves: ");
			Serial.println(myDiagnostics.numberOfSlaves);
		//Serial.print("debug out: ");
		//	Serial.println(myMotorDriver.readRegister(SCMD_MA_DRIVE));
	}
		
	//**Script timer******************//  
	if(outputTimer.flagStatus() == PENDING)
	{
		currentSwitchValues[0] = digitalRead(8) ^ 0x01;
		currentSwitchValues[1] = digitalRead(9) ^ 0x01;
		currentSwitchValues[2] = digitalRead(10) ^ 0x01;
		currentSwitchValues[3] = digitalRead(11) ^ 0x01;
		currentSwitchValues[4] = digitalRead(12) ^ 0x01;
		
		for( int i = 0; i < 5; i++ )
		{
			if( lastSwitchValues[i] != currentSwitchValues[i] )
			{
				lastSwitchValues[i] = currentSwitchValues[i];
				Serial.println(currentSwitchValues[i]);
				switch(i)
				{
					case 0:
					//myMotorDriver.inversionMode(0 + (2*currentSwitchValues[3]), currentSwitchValues[i]);
					
					break;
					case 1:
					myMotorDriver.inversionMode(1 + (2*currentSwitchValues[3]), currentSwitchValues[i]);
					break;
					case 2:
					myMotorDriver.bridgingMode(0 + currentSwitchValues[3], currentSwitchValues[i]);
					break;
					case 3:
					break;
					case 4:
					myMotorDriver.writeRegister(SCMD_DRIVER_ENABLE, currentSwitchValues[i]);
					break;
					default:
					break;
				}
			}
		}
		uint16_t timeVariable = msTicks;
		//Read inputs
		int8_t inputAnalog[4];
		
		for( int i = 0; i < 4; i++)
		{
			//Convert to -128 to 127
			inputAnalog[i] = (int16_t)(analogRead(i) >> 2) - 128;
		}
			
		//Send outputs
		for( int i = 1; i < 4; i++)
		{
			if(inputAnalog[i] >= 0)
			{
				myMotorDriver.setDrive(i,1,inputAnalog[i]); //Drive motor i forward
			}
			else
			{
				myMotorDriver.setDrive(i,0,inputAnalog[i] * -1); //Drive motor i backward
			}
		}
		timeVariable = msTicks - timeVariable;
		if( timeVariable > durationRecord[0] )
		{
			durationRecord[0] = timeVariable;
		}
		myMotorDriver.writeRegister(SCMD_UPDATE_RATE, (inputAnalog[0] + 128) );
	}
}

//If 328p based, do this
#ifdef __AVR__
ISR(TIMER1_COMPA_vect)
#else
#endif
//If Teensy 3.1 based, do this
#ifdef __MK20DX256__
void serviceMS(void)
#else
#endif
{
	uint32_t returnVar = 0;
	if( msTicks >= ( maxTimer + maxInterval ) )
	{
		returnVar = msTicks - maxTimer;

	}
	else
	{
		returnVar = msTicks + 1;
	}
	msTicks = returnVar;
	msTicksLocked = 0;  //unlock
}

