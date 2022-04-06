//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// Atmel AVR MCU low-level driver.
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
// updated - CTRL PORT, and DIR - 2022 AD
//-------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/pgmspace.h>

#define KS0108_DATA_PORT	PORTD 	//PD0 - PD7
#define KS0108_DATA_DIR		DDRD
#define KS0108_DATA_PIN		PIND
/*
#define KS0108_DATA_PORT_DB0	PORTD 	//DB0
#define KS0108_DATA_DIR_DB0		DDRD
#define KS0108_DATA_PIN_DB0		0
#define KS0108_DATA_PORT_DB1	PORTD 	//DB1
#define KS0108_DATA_DIR_DB1		DDRD
#define KS0108_DATA_PIN_DB1		1
#define KS0108_DATA_PORT_DB2	PORTD 	//DB2
#define KS0108_DATA_DIR_DB2		DDRD
#define KS0108_DATA_PIN_DB2		2
#define KS0108_DATA_PORT_DB3	PORTD 	//DB3
#define KS0108_DATA_DIR_DB3		DDRD
#define KS0108_DATA_PIN_DB3		3
#define KS0108_DATA_PORT_DB4	PORTD 	//DB4
#define KS0108_DATA_DIR_DB4		DDRD
#define KS0108_DATA_PIN_DB4		4
#define KS0108_DATA_PORT_DB5	PORTD 	//DB5
#define KS0108_DATA_DIR_DB5		DDRD
#define KS0108_DATA_PIN_DB5		5
#define KS0108_DATA_PORT_DB6	PORTD 	//DB6
#define KS0108_DATA_DIR_DB6		DDRD
#define KS0108_DATA_PIN_DB6		6
#define KS0108_DATA_PORT_DB7	PORTD 	//DB7
#define KS0108_DATA_DIR_DB7		DDRD
#define KS0108_DATA_PIN_DB7		7
*/
//#define KS0108_CTRL_PORT	PORTC
//#define KS0108_CTRL_DIR	DDRC
/*
#define KS0108_CTRL_PORT_CS1	PORTC
#define KS0108_CTRL_DIR_CS1		DDRC
#define KS0108_CTRL_PORT_CS2	PORTC
#define KS0108_CTRL_DIR_CS2		DDRC
#define KS0108_CTRL_PORT_CS3	PORTC
#define KS0108_CTRL_DIR_CS3		DDRC
*/
#define KS0108_CTRL_PORT_CS1	PORTB
#define KS0108_CTRL_DIR_CS1		DDRB
#define KS0108_CTRL_PORT_CS2	PORTB
#define KS0108_CTRL_DIR_CS2		DDRB
#define KS0108_CTRL_PORT_CS3	PORTB
#define KS0108_CTRL_DIR_CS3		DDRB

#define KS0108_CTRL_PORT_RS		PORTC
#define KS0108_CTRL_DIR_RS		DDRC
#define KS0108_CTRL_PORT_RW		PORTC
#define KS0108_CTRL_DIR_RW		DDRC
#define KS0108_CTRL_PORT_EN		PORTC
#define KS0108_CTRL_DIR_EN		DDRC

#define KS0108_RS			(1 << 0)	//DI
#define KS0108_RW			(1 << 1)
#define KS0108_EN			(1 << 2)
/*
#define KS0108_CS1			(1 << 3)
#define KS0108_CS2			(1 << 4)
#define KS0108_CS3			(1 << 4)
*/
#define KS0108_CS1			(1 << 1)
#define KS0108_CS2			(1 << 2)
#define KS0108_CS3			(1 << 2)

extern unsigned char screen_x;
extern unsigned char screen_y;

#define DISPLAY_STATUS_BUSY	0x80

//------------------------
// setup I/O pins
//------------------------
/*
void set_DATA_pins_IN(void)
{
// see in datasheet for your MCU. next lines are for atmega328
//example ,,,,DDRA = (1<<DDA1) - set 1,,,,DDRA &=~((1<<DDA1) - set 0
DDRD &=~ ((1<<DDD0)|(1<<DDD1)|(1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7) ); //all port D is set to IN	
//DDRC &=~ ((1<<DDC0)|(1<<DDC1)|(1<<DDC2)|(1<<DDC3)|(1<<DDC4)|(1<<DDC5)|(1<<DDC6)|(1<<DDC7) ); //all port C is set to IN	
//DDRB &=~ ((1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5)|(1<<DDB6)|(1<<DDB7) ); //all port B is set to IN
}

void set_DATA_pins_OUT(void)
{
// see in datasheet for your MCU. next lines are for atmega328
//example ,,,,DDRA = (1<<DDA1) - set 1,,,,DDRA &=~((1<<DDA1) - set 0
DDRD = ((1<<DDD0)|(1<<DDD1)|(1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7) ); //all port D is set to OUT	
//DDRC = ((1<<DDC0)|(1<<DDC1)|(1<<DDC2)|(1<<DDC3)|(1<<DDC4)|(1<<DDC5)|(1<<DDC6)|(1<<DDC7) ); //all port C is set to OUT	
//DDRB = ((1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5)|(1<<DDB6)|(1<<DDB7) ); //all port B is set to OUT
}
*/
//-------------------------------------------------------------------------------------------------
// Delay function
//-------------------------------------------------------------------------------------------------
void GLCD_Delay(void)
{
asm("nop");
}
//-------------------------------------------------------------------------------------------------
// Enable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_EnableController(unsigned char controller)
{
switch(controller){
	case 0 : KS0108_CTRL_PORT_CS1 &= ~KS0108_CS1; break;
	case 1 : KS0108_CTRL_PORT_CS2 &= ~KS0108_CS2; break;
	case 2 : KS0108_CTRL_PORT_CS3 &= ~KS0108_CS3; break;
	}
}
//-------------------------------------------------------------------------------------------------
// Disable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_DisableController(unsigned char controller)
{
switch(controller){
	case 0 : KS0108_CTRL_PORT_CS1 |= KS0108_CS1; break;
	case 1 : KS0108_CTRL_PORT_CS2 |= KS0108_CS2; break;
	case 2 : KS0108_CTRL_PORT_CS3 |= KS0108_CS3; break;
	}
}
//-------------------------------------------------------------------------------------------------
// Read Status from specified controller (0-2)
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadStatus(unsigned char controller)
{
unsigned char status;

KS0108_DATA_DIR = 0x00;	//DDR to 0 - all pins are IN
//set_DATA_pins_IN();
KS0108_CTRL_PORT_RW |= KS0108_RW;
KS0108_CTRL_PORT_RS &= ~KS0108_RS;
GLCD_EnableController(controller);
KS0108_CTRL_PORT_EN |= KS0108_EN;
GLCD_Delay();
status = KS0108_DATA_PIN;
//status = KS0108_DATA_PIN_DB0 | KS0108_DATA_PIN_DB1 | KS0108_DATA_PIN_DB2 | KS0108_DATA_PIN_DB3 | KS0108_DATA_PIN_DB4 | KS0108_DATA_PIN_DB5 | KS0108_DATA_PIN_DB6 | KS0108_DATA_PIN_DB7;
KS0108_CTRL_PORT_EN &= ~KS0108_EN;
GLCD_DisableController(controller);
return status;
}
//-------------------------------------------------------------------------------------------------
// Write command to specified controller
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite, unsigned char controller)
{
while(GLCD_ReadStatus(controller)&DISPLAY_STATUS_BUSY);
KS0108_DATA_DIR = 0xFF;
//set_DATA_pins_OUT();
KS0108_CTRL_PORT_RW &= ~KS0108_RW ; KS0108_CTRL_PORT_RW &= ~KS0108_RS;
GLCD_EnableController(controller);
KS0108_DATA_PORT = commandToWrite;	// set PINs to 0/1

KS0108_CTRL_PORT_EN |= KS0108_EN;
GLCD_Delay();
KS0108_CTRL_PORT_EN &= ~KS0108_EN;
GLCD_DisableController(controller);
}
//-------------------------------------------------------------------------------------------------
// Read data from current position
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
unsigned char data;
while(GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY);
//set_DATA_pins_IN();
KS0108_DATA_DIR = 0x00;
KS0108_CTRL_PORT_RW |= KS0108_RW;
KS0108_CTRL_PORT_RS |= KS0108_RS;
GLCD_EnableController(screen_x / 64);
GLCD_Delay();

KS0108_CTRL_PORT_EN |= KS0108_EN;
GLCD_Delay();
data = KS0108_DATA_PIN;
//data = KS0108_DATA_PIN_DB0 | KS0108_DATA_PIN_DB1 | KS0108_DATA_PIN_DB2 | KS0108_DATA_PIN_DB3 | KS0108_DATA_PIN_DB4 | KS0108_DATA_PIN_DB5 | KS0108_DATA_PIN_DB6 | KS0108_DATA_PIN_DB7;
KS0108_CTRL_PORT_EN &= ~KS0108_EN;
GLCD_DisableController(screen_x / 64);
screen_x++;
return data;
}
//-------------------------------------------------------------------------------------------------
// Write data to current position
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
while(GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY);
KS0108_DATA_DIR = 0xFF;
//set_DATA_pins_OUT();
KS0108_CTRL_PORT_RW &= ~KS0108_RW;
KS0108_CTRL_PORT_RS |= KS0108_RS;
KS0108_DATA_PORT = dataToWrite;
GLCD_EnableController(screen_x / 64);
KS0108_CTRL_PORT_EN |= KS0108_EN;
GLCD_Delay();
KS0108_CTRL_PORT_EN &= ~KS0108_EN;
GLCD_DisableController(screen_x / 64);
screen_x++;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_InitalizePorts(void)
{
KS0108_CTRL_DIR_CS1 |= KS0108_CS1; 
KS0108_CTRL_DIR_CS2 |= KS0108_CS2; 
KS0108_CTRL_DIR_CS3 |= KS0108_CS3; 
KS0108_CTRL_DIR_RS	|= KS0108_RS; 
KS0108_CTRL_DIR_RW	|= KS0108_RW; 
KS0108_CTRL_DIR_EN	|= KS0108_EN;

KS0108_CTRL_PORT_CS1 |= KS0108_CS1; 
KS0108_CTRL_PORT_CS2 |= KS0108_CS2; 
KS0108_CTRL_PORT_CS3 |= KS0108_CS3;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadByteFromROMMemory(char * ptr)
{
return pgm_read_byte(ptr);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
