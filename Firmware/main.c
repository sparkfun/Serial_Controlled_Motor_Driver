#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "main.h"

//================================================================
//Define Global Variables
//================================================================
volatile char motor_current_monitor='0';
volatile int motor_current=0;


//================================================================
//Interrupt Routines
//================================================================
ISR (SIG_ADC)
{
	cli();
	
	motor_current = ADCL;
	motor_current |= ADCH<<8;	
	
	if(motor_current > CURRENT_THRESHOLD)
	{
		//If an over current is detected, stop the motors
		set_speed('0', '0');
		set_speed('1', '0');
		printf("ALERT: %c, %d\n", motor_current_monitor, motor_current);
	}
	
	//printf("%c, %d\n", motor_current_monitor, motor_current);
	
	if(motor_current_monitor == '0')motor_current_monitor='1';
	else motor_current_monitor='0';
	
	ADMUX=motor_current_monitor-'0';
	
	sei();
}

int main (void)
{	
	char command_buffer[MAX_COMMAND_LENGTH];

	//Initialize AVR I/O, UART and Interrupts
    ioinit();
	set_direction('1', 'f');
	set_speed('1', '0');
	set_direction('2', 'f');
	set_speed('2', '0');
	printf("Ready\n");
	
	sei();
	while(1){
		printf("> ");

		if(get_command(command_buffer))
		{
			printf("\n");
			if(check_command(command_buffer))
			{
				control_motor(command_buffer);
			}
		}
		else printf("\nCommand is WAY too long!\n");
		printf("%d\n", motor_current);
	}
	
	
    return (0);
}

//==================================================
//Core functions
//==================================================
//Function: ioinit
//Purpose:	Initialize AVR I/O, UART and Interrupts
//Inputs:	None
//Outputs:	None
void ioinit(void)
{
    //1 = output, 0 = input

	DDRB = (1<<PWM1)|(1<<PWM2)|(1<<MOSI)|(1<<SCK);	//Enable PWM and SPI pins as outputs
	PORTB = (1<<MISO);	//Enable pull-up on MISO pin

	DDRC = 0xFF;	//Initialize Port B to all inputs
	//PORTC = 0xFF;	//Activate all pull-ups on Port B
	
	DDRD = 0xFF;	//Set Port D to all outputs
	DDRD &= ~(1<<RX);	//Set bit 0(Rx) to be an input
	PORTD = (1<<RX);	//Enable the pull-up on Rx

	
	// USART Baud rate: 115200 (With 16 MHz Clock)
    UBRR0H = (MYUBRR >> 8) & 0x7F;	//Make sure highest bit(URSEL) is 0 indicating we are writing to UBRRH
	UBRR0L = MYUBRR;
    UCSR0A = (1<<U2X0);					//Double the UART Speed
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);		//Enable Rx and Tx in UART
    UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);		//8-Bit Characters
    stdout = &mystdout; //Required for printf init
	
	// Init timer 2
	//Set Prescaler to 8. (Timer Frequency set to 16Mhz)
	//Used for delay routines
	//TCCR2B = (1<<CS20); 	//Divde clock by 1 for 16 Mhz Timer 2 Frequency	
	
	//Init Timer 1 for 10 BIT Fast Mode PWM with output toggle on OC1A and OC1B
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10) | (1<<WGM11); //Set toggle mode for PWM pins and set 10bit fast PWM mode
	TCCR1B = (1<<WGM12);	//Set 8bit fast PWM mode
	OCR1A = 0;
	OCR1B = 0;
	TCCR1B |= (1<<CS12);
	
	//Initialize the ADC for Free Running Reads to control motor shut-down in case of over current
	ADMUX = motor_current_monitor-'0';	//Set the ADC channel to 0.
	ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Enable the ADC, auto-triggering, the ADC Interrupt and set the ADC prescaler to 64
																	  //ADC clock = 250kHz
	ADCSRA |= (1<<ADSC);	//Start the first conversion
	
}


static int uart_putchar(char c, FILE *stream)
{
  if (c == '\n')
    uart_putchar('\r', stream);
  
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
	return(UDR0);
}

/*
//General short delays
void delay_ms(uint16_t x)
{
    for (; x > 0 ; x--)
        delay_us(1000);
}

//General short delays
void delay_us(uint16_t x)
{    
    TIFR2 = (1<<TOV2); //Clear any interrupt flags on Timer2
    TCNT2= 240; //Setting counter to 239 will make it 16 ticks until TOV is set. .0625uS per click means 1 uS until TOV is set
    while(x>0){
		while( (TIFR2 & (1<<TOV2)) == 0);
		TIFR2 = (1<<TOV2); //Clear any interrupt flags on Timer2
		TCNT2=240;
		x--;
	}
} 
*/

char get_command(char * command)
{
	char receive_char=0, command_buffer_count=0;
	
	//Get a command from the prompt (A command can have a maximum of MAX_COMMAND_LENGTH characters). Command is ended with a carriage return ('Enter' key)
	receive_char = uart_getchar();
	while(receive_char != '\r'){
		*command=receive_char;
		printf("%c", *command++);
		command_buffer_count++;
		if(command_buffer_count == (MAX_COMMAND_LENGTH))return 0;
		receive_char =  uart_getchar();
	}
	*command='\0';		//Terminate the command string with a NULL character. This is so command_buffer[] can be treated as a string.

	return 1;
}

//Command should always be in format M,D,S (Motor, Direction, Speed)
//Sample Commands:

void control_motor(char * command)
{
	char motor = command[0];	
	char direction = tolower(command[1]);
	char speed = command[2];
	
	set_direction(motor, direction);
	set_speed(motor, speed);
}

char check_command(char * command)
{
	char command_length=strlen(command);
	
	if(command_length != 3)
	{
		printf("Incorrect Command Length\n");
		return 0;
	}
	if(command[0] != '1' && command[0] !='2')
	{
		printf("Invalid Motor\n");
		return 0;
	}
	if(tolower(command[1]) != 'f' && tolower(command[1]) !='r')
	{
		printf("Invalid Direction\n");
		return 0;
	}	
	if(!isdigit(command[2]))
	{
		printf("Invalid Speed\n");
		return 0;
	}
	
	return 1;
}

void set_direction(char motor, char direction)
{
	if(motor == '1')
	{
		if(direction == 'f')
		{
			M1_FORWARD();
		}
		else
		{
			M1_REVERSE();
		}
	}
	else
	{
		if(direction == 'f')
		{
			M2_FORWARD();
		}
		else
		{
			M2_REVERSE();
		}
	}
}

void set_speed(char motor, char speed)
{
	if(motor=='1')
	{
		if(speed=='9')OCR1A = 0x3FF;
		else OCR1A = SPEED_UNIT*(speed-48);
	}
	else
	{
		if(speed=='9')OCR1B = 0x3FF;
		else OCR1B = SPEED_UNIT*(speed-48);
	}
}