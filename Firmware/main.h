//*******************************************************
//					GPIO Definitions
//*******************************************************
//Port C Pin Assignments
#define SENSE_1	0
#define SENSE_2	1

//Port D Pin Assignments
#define RX		0
#define TX		1
#define	M1_P	2
#define M1_N	3
#define M2_P	4
#define M2_N	7

//Port B Pin Assignments
#define PWM1	1
#define PWM2	2
#define MOSI	3
#define MISO	4
#define SCK		5

//*******************************************************
//						Macros
//*******************************************************
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define M1_FORWARD()	sbi(PORTD, M1_P);cbi(PORTD, M1_N)
#define M1_REVERSE()	sbi(PORTD, M1_N);cbi(PORTD, M1_P)

#define M2_FORWARD()	sbi(PORTD, M2_P);cbi(PORTD, M2_N)
#define M2_REVERSE()	sbi(PORTD, M2_N);cbi(PORTD, M2_P)

//*******************************************************
//					General Definitions
//*******************************************************
#define MYUBRR 16	//Used to set the AVR Baud Rate TO 115200 (External 16MHz Oscillator)
#define MAX_COMMAND_LENGTH	10

#define SPEED_UNIT	114
#define AVERAGE	16

#define CURRENT_THRESHOLD	150 //This is compared against the 10 bit ADC value and corresponds to roughly 1.5A on the Current Sense pin
								//from the Motor Controller

//=======================================================
//					Function Definitions
//=======================================================
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
void ioinit(void);
void delay_ms(uint16_t x);
void delay_us(uint16_t x);
char get_command(char * command);
void control_motor(char * command);
char check_command(char * command);
void set_direction(char motor, char direcion);
void set_speed(char motor, char speed);
uint16_t read_sense(char motor);