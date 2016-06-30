//HOW TO OPERATE
//  Make TimerClass objects for each thing that needs periodic service
//  pass the interval of the period in ticks
//
//  Set maxInterval to rollover rate
//  Set maxTimer to the max foreseen interval of any timer.
//  maxTimer + maxInterval = max countable value.
#include "HOS_char.h"

//Globals
uint16_t maxTimer = 60000;
uint16_t maxInterval = 2000;

#define LEDPIN 13
#include "timerModule.h"
#include "stdint.h"

//Not used by this sketch but dependant on one 
//#include "Wire.h"
#include "SPI.h"

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

#define chipSelectPin 10

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
	SPI.begin();
	// Maximum SPI frequency is 10MHz, could divide by 2 here:
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	// Data is read and written MSb first.
	SPI.setBitOrder(MSBFIRST);
	// Data is captured on rising edge of clock (CPHA = 0)
	// Base value of the clock is HIGH (CPOL = 1)
	// This was SPI_MODE3 for RedBoard, but I had to change to
	// MODE0 for Teensy 3.1 operation
	SPI.setDataMode(SPI_MODE0);
	// initalize the  data ready and chip select pins:
	pinMode(chipSelectPin, OUTPUT);
	digitalWrite(chipSelectPin, HIGH);


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
				Serial.println("Sending some SPI data");
				digitalWrite(chipSelectPin, LOW);
				// send the device the register you want to read:
				SPI.transfer(0x84 & 0x7F);
				// send a value of 0 to read the first byte returned:
				SPI.transfer(0x66);
				// decrement the number of bytes left to read:
				// take the chip select high to de-select:
				digitalWrite(chipSelectPin, HIGH);
				break;
				
				
				case 1:
				Serial.println("Sending other SPI data");
				digitalWrite(chipSelectPin, LOW);
				// send the device the register you want to read:
				SPI.transfer(0xAA & 0x7F);
				// send a value of 0 to read the first byte returned:
				SPI.transfer(0xCF);
				// decrement the number of bytes left to read:
				// take the chip select high to de-select:
				digitalWrite(chipSelectPin, HIGH);
				break;


				case 2:
				Serial.print("Setting remote address: 0x");
				Serial.println(userInputValue, HEX);
				localAddress = userInputValue;
				digitalWrite(chipSelectPin, LOW);
				// send the device the register you want to read:
				SPI.transfer(localAddress | 0x80);
				// decrement the number of bytes left to read:
				// take the chip select high to de-select:
				digitalWrite(chipSelectPin, HIGH);
				break;
				
				
				case 3:
				Serial.println("Getting remote value");
				digitalWrite(chipSelectPin, LOW);
				// send the device the register you want to read:
				result = SPI.transfer(localAddress | 0x80);
				// decrement the number of bytes left to read:
				// take the chip select high to de-select:
				digitalWrite(chipSelectPin, HIGH);
				Serial.print("0x");
				Serial.println(result, HEX);	
				break;


				case 4:
				Serial.print("Setting local address (for writes): 0x");
				Serial.println(userInputValue, HEX);
				localAddress = userInputValue;
				break;

				
				case 5:
				Serial.print("Setting remote address: 0x");
				Serial.print(localAddress, HEX);
				Serial.print(" to value: ");
				Serial.print(userInputValue, HEX);
				digitalWrite(chipSelectPin, LOW);
				// send the device the register you want to read:
				SPI.transfer(localAddress & 0x7F);
				// send a value of 0 to read the first byte returned:
				SPI.transfer(userInputValue);
				// decrement the number of bytes left to read:
				// take the chip select high to de-select:
				digitalWrite(chipSelectPin, HIGH);
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

