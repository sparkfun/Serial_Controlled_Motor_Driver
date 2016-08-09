//This example sends knob inputs out to motors 1, 2, 3, 4 via I2C
//It returns diagnostic information



//HOW TO OPERATE
//  Make TimerClass objects for each thing that needs periodic service
//  pass the interval of the period in ticks
//
//  Set maxInterval to rollover rate
//  Set maxTimer to the max foreseen interval of any timer.
//  maxTimer + maxInterval = max countable value.
#include "HOS_char.h"
#include "SparkFun_PSoCMotorDriverAPI.h"

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
TimerClass outputTimer( 50 );  //Read knobs and drive motors

//Required for timers:
uint16_t msTicks = 0;
uint8_t msTicksLocked = 1; //start locked out

PSoCMD myMotorDriver;

//Diagnostic
uint16_t durationRecord[1] = {0};

void setup()
{
	Serial.begin(115200);
	pinMode(LEDPIN, OUTPUT);
	delay(1500);
	Serial.println("Starting sketch.");

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	
	//Specify configuration for the driver
	myMotorDriver.settings.commInterface = I2C_MODE;
	myMotorDriver.settings.I2CAddress = 0x5A;
	//myMotorDriver.settings.chipSelectPin = 10;
	//myMotorDriver.settings.invertA = 1;
	//myMotorDriver.settings.invertB = 1;

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
	Serial.println(myMotorDriver.begin(), HEX);

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
		Serial.println(myMotorDriver.readRegister(0x01), HEX);
		
		//Serial.println("MEMORY DUMP:");
		//for( int i = 0x0; i < 0x28; i++)
		//{
		//	Serial.print("0x");
		//	Serial.print(myMotorDriver.readRegister(i), HEX);
		//	Serial.print(": 0x");
		//	Serial.println(myMotorDriver.readRegister(i), HEX);
		//}		
		Serial.print("outputTimer Fn Duration Peak: ");
		Serial.print(durationRecord[0]);
		Serial.println("ms");
		durationRecord[0] = 0;
		Serial.print("I2C routine duration: ");
		Serial.print(myMotorDriver.readRegister(0x0A));
		Serial.println("us");
		Serial.print("I2C RX error cnt: ");
		Serial.println(myMotorDriver.readRegister(0x05));
		Serial.print("I2C WR error cnt: ");
		Serial.println(myMotorDriver.readRegister(0x06));
		}
	//**Script timer******************//  
	if(outputTimer.flagStatus() == PENDING)
	{
		uint16_t timeVariable = msTicks;
		//Read inputs
		int8_t inputAnalog[4];
		
		for( int i = 0; i < 4; i++)
		{
			//Convert to -128 to 127
			inputAnalog[i] = (int16_t)(analogRead(i) >> 2) - 128;
		}
			
		//Send outputs
		for( int i = 0; i < 4; i++)
		{
			if(inputAnalog[i] >= 0)
			{
				myMotorDriver.setDrive(i,1,inputAnalog[i]); //Drive motor i forward
			}
			else
			{
				myMotorDriver.setDrive(i,0,inputAnalog[i] * -1); //Drive motor i backward
			}
			delay(1);
		}
		timeVariable = msTicks - timeVariable;
		if( timeVariable > durationRecord[0] )
		{
			durationRecord[0] = timeVariable;
		}
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

