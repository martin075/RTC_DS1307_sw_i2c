//Atmega168 at 8MHz
//DS1307 , SW I2C
// KS108 - LCD 128x64
/* Includes --------------------------------------------------- */
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "KS0108.h"
#include "graphic.h"
#include "font5x8.h"
#include "soft_I2C.h" 
#include "DS1307.h"

// address DS1307 7-bit 1101000 R/W - 0x68  see DS1307.h


/* Functions ------------------------------------------------- */ 
//  Functions prototypes 

uint8_t bcdtodec(uint8_t bcdnumber);
uint8_t bcd2int(uint8_t value);
uint8_t int2bcd(uint8_t value);

//------------------------------------------------------------


int main(void)
{
	/* Local variables ------------------------------------------------------- */
	
	uint8_t i=0,j=0;	
	uint8_t sw_sec=61,sw_min=63,sw_h=25,sw_date,sw_month,sw_year,sw_day=8;
	uint8_t day;

	char text[128] = "";

	// init section------------
	SoftI2CInit(); 

	GLCD_Initialize();
	GLCD_ClearScreen();
	GLCD_GoTo(5,0);
	GLCD_WriteString("clock with DS1307");
	GLCD_GoTo(5,1);
	GLCD_WriteString("by Atmega168 - 8MHz");
	GLCD_GoTo(5,2);
	GLCD_WriteString("SoftWare i2c");
	/* Enable interrupts by setting the global interrupt mask */
	sei ();
		
	// setting time section
	//DS1307Write(3,3); //Wednesday

	//--------------------------------------
	
	/* Infinite loop */
	while (1)
	{
		if(i == 250)
		{
			if(DS1307Read(0x00,&sw_sec) != 1) sw_sec=99;
			DS1307Read(0x01,&sw_min);
			DS1307Read(0x02,&sw_h);
			DS1307Read(0x03,&sw_day);
			DS1307Read(0x04,&sw_date);
			DS1307Read(0x05,&sw_month);
			DS1307Read(0x06,&sw_year);
			GLCD_GoTo(0, 4);	//x ,y
			sprintf(text, " %02x:%02x:%02x",sw_h,sw_min,sw_sec);
			GLCD_WriteString(text);
			j++;
		}
		if(j == 50)
		{
			j=0;
			day=bcd2int(sw_day);
			//day=bcdtodec(sw_day);
			GLCD_GoTo(0, 5);	//x ,y
			switch(day){
				case 1: GLCD_WriteString("Pondelok-Monday"); break;
				case 2: GLCD_WriteString("Utorok-Tuesday"); break;
				case 3: GLCD_WriteString("Streda-Wednesday"); break;
				case 4: GLCD_WriteString("Stvrtok-Thursday"); break;
				case 5: GLCD_WriteString("Piatok-Friday"); break;
				case 6: GLCD_WriteString("Sobota-Saturday"); break;
				case 7: GLCD_WriteString("Nedela-Sunday"); break;
				default: GLCD_WriteString("blue Monday");
				}
		
			GLCD_GoTo(0, 6);	//x ,y
			sprintf(text, "SWdate %02x/%02x/%02x-%02x",sw_date,sw_month,sw_year,sw_day);
			GLCD_WriteString(text);
		}
		
		if(i>1000)i=0;
		else i++;
		
		} // while loop
		
} //main 	


uint8_t bcdtodec(uint8_t bcdnumber)
{
	uint8_t tens, units, decimal;
	tens = bcdnumber >> 4;
	units = bcdnumber & 0x0F;
	decimal = (tens*10) + units;
	return decimal;
}

/* Convert a uint8_t into a BCD value */
uint8_t int2bcd(uint8_t value) {
  return (value % 10) + 16*(value/10);
} 

/* Convert a one-byte BCD value to a normal integer */
uint8_t bcd2int(uint8_t value) {
  return (value & 0x0f) + 10*(value >> 4);
} 



