//HOW TO OPERATE
//  Make TimerClass objects for each thing that needs periodic service
//  pass the interval of the period in ticks
//
//  Set maxInterval to rollover rate
//  Set maxTimer to the max foreseen interval of any timer.
//  maxTimer + maxInterval = max countable value.
#include "HOS_char.h"
#include "ArduinoPSoCMotorDriverAPI.h"

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
//**Copy to make a new timer******************//  
//TimerClass32 msTimerA( 200 ); //200 ms
TimerClass debugTimer( 10000 ); //milliseconds between calls
TimerClass serialReadTimer( 10 );  //check for new characters
TimerClass scriptTimer( 50 );  //check for new characters

//Note on TimerClass-
//Change with msTimerA.setInterval( <the new interval> );


uint16_t msTicks = 0;

//  The lock works like this:
//
//    When the interrupt fires, the lock is removed.  Now
//    the main free-wheeling loop can update the change to
//    the timerModules.  Once complete, the lock is replaced
//    so that it can't update more than once per firing
//    of the interrupt

uint8_t msTicksLocked = 1; //start locked out

//Serial packet defines
#define PACKET_LENGTH 24
#define START_SYMBOL '*'

char lastchar;
char packet[PACKET_LENGTH];
char lastPacket[PACKET_LENGTH];
char packetPending = 0;

char packet_ptr;

uint8_t userInputValue = 0;
uint8_t localAddress = 0;

#define OPTIONLIMIT 13
int16_t optionCounter = 0;
uint8_t pendingScript = 0;

PSoCMD myMotorDriver;

void setup()
{
	Serial.begin(115200);
	pinMode(LEDPIN, OUTPUT);


	//If 328p based, do this
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
	
	myMotorDriver.settings.commInterface = I2C_MODE;
	myMotorDriver.settings.I2CAddress = 0x58;
	//myMotorDriver.settings.chipSelectPin = 10;
	//myMotorDriver.settings.invertA = 1;
	//myMotorDriver.settings.invertB = 1;
	delay(2000);
	Serial.println("Starting sketch");
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
		serialReadTimer.update(msTicks);
		scriptTimer.update(msTicks);
		
		//Done?  Lock it back up
		msTicksLocked = 1;
	}  //The ISR will unlock.

	//**Copy to make a new timer******************//  
	//if(msTimerA.flagStatus() == PENDING)
	//{
	//	//User code
	//}
	
	if(debugTimer.flagStatus() == PENDING)
	{
		//User code
		digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 0x01 );
		Serial.println("OOoo..   Debug Interval Timer   ..ooOO");
	}
	//**Check for new characters******************//  
	if(serialReadTimer.flagStatus() == PENDING)
	{
		//User code
		if (Serial.available())
		{
			lastchar = Serial.read();
			//look for packet start (START_SYMBOL)
			if( lastchar == START_SYMBOL )
			{
				//Flag that the packet needs to be serviced
				packetPending = 1;
				
				//Fill packet with null, reset the pointer
				for( int i = 0; i < PACKET_LENGTH; i++ )
				{
				packet[i] = 0;
				}
				//write the start char
				packet[0] = START_SYMBOL;
				//reset the pointer
				packet_ptr = 1;
			}
			else
			if( ( packet_ptr < PACKET_LENGTH ) && (packet_ptr > 0) )//check for room in the packet, and that the start char has been seen
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
		
		uint8_t changed = 0;
		
		//if the packet is full and the last char is LF or CR, *do something here*
//		if((packetPending == 1) && ((packet_ptr == PACKET_LENGTH) && ((packet[PACKET_LENGTH - 1] == 0x0A) || (packet[PACKET_LENGTH - 1] == 0x0D))) )     ------OLD LINE OF CODE
		if((packetPending == 1) && (((packet[packet_ptr - 1] == 0x0A) || (packet[packet_ptr - 1] == 0x0D))) )
		{
			digitalWrite(13, digitalRead(13)^1);
			//check for new data
			packetPending = 0;
			for(int k = 0; k < PACKET_LENGTH; k++)
			{
				//if(packet[k] != lastPacket[k])
				{
					lastPacket[k] = packet[k];
					//Serial.print(packet[k]);
					//Serial.println("change marked");
				} 
			}
			//Check for valid 2 digit hex at lower position
			if((((packet[1] >= '0')&&(packet[1] <= '9'))||((packet[1] >= 'A')&&(packet[1] <= 'F')))&&(((packet[2] >= '0')&&(packet[2] <= '9'))||((packet[2] >= 'A')&&(packet[2] <= 'F'))))
			{
				userInputValue = char2hex(packet[2]) | (char2hex(packet[1]) << 4);
				pendingScript = 1;
			}
			else if(packet[1] == '+')
			{
				//move forward through options
				optionCounter++;
				if(optionCounter > OPTIONLIMIT) optionCounter = 0;
				Serial.print("option: ");
				Serial.println(optionCounter);
			}
			else if(packet[1] == '-')
			{
				//Move backwards through options
				optionCounter--;
				if(optionCounter < 0) optionCounter = OPTIONLIMIT;
				Serial.print("option: ");
				Serial.println(optionCounter);
			}
			else
			{
				//increment selection (and display message)
				Serial.println("non-characters");
			}
		}

	}
	//**Script timer******************//  
	if(scriptTimer.flagStatus() == PENDING)
	{
		if(pendingScript)
		{
			uint8_t result = 0;
			switch(optionCounter)
			{
				case 0:
				Serial.println("Getting status register");
				Serial.print("0x");
				Serial.println(myMotorDriver.readRegister(0x00), HEX);
				break;
				
				
				case 1:
				Serial.println("Getting ID register");
				Serial.print("0x");
				Serial.println(myMotorDriver.readRegister(0x01), HEX);
				break;


				case 2:
				Serial.println("Running A forward.");
				for( int i = 0x00; i <= 0xFF; i = i + 0x08)
				{
					myMotorDriver.setDrive(0,1,i);
					delay(100);
				}
				break;
				
				
				case 3:
				Serial.println("Running A backward.");
				for( int i = 0x00; i <= 0xFF; i = i + 0x08)
				{
					myMotorDriver.setDrive(0,0,i);
					delay(100);
				}
				break;


				case 4:
				Serial.println("Running B forward.");
				for( int i = 0x00; i <= 0xFF; i = i + 0x08)
				{
					myMotorDriver.setDrive(1,1,i);
					delay(100);
				}
				break;

				
				case 5:
				Serial.println("Running B backward.");
				for( int i = 0x00; i <= 0xFF; i = i + 0x08)
				{
					myMotorDriver.setDrive(1,0,i);
					delay(100);
				}
				break;
				
				
				default:
				break;
			}
			pendingScript = 0;
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

