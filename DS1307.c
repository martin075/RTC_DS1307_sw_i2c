#include <avr/io.h>
#include <util/delay.h>
#include "Soft_I2C.h"
#include "DS1307.h"
#define BOOL uint8_t //Define a datatype

/* DS1307 functions from Avinash's site */


void DS1307Init(void)
{
	SoftI2CInit();
}



/***************************************************
Function To Read Internal Registers of DS1307
---------------------------------------------
address : Address of the register
data: value of register is copied to this.
Returns:
0= Failure
1= Success
***************************************************/

BOOL DS1307Read(uint8_t address,uint8_t *data)
{
	uint8_t res;   //result

	//Start

	SoftI2CStart();

	//SLA+W (for dummy write to set register pointer)
	res=SoftI2CWriteByte(DS1307_SLA_W); //DS1307 address + W

	//Error
	if(!res) return FALSE;

	//Now send the address of required register

	res=SoftI2CWriteByte(address);

	//Error
	if(!res) return FALSE;

	//Repeat Start
	SoftI2CStart();

	//SLA + R
	res=SoftI2CWriteByte(DS1307_SLA_R); //DS1307 Address + R

	//Error
	if(!res) return FALSE;

	//Now read the value with No ACK (0)
	*data=SoftI2CReadByte(0);

	//Error

	if(!res) return FALSE;

	//STOP
	SoftI2CStop();

	return TRUE;
}

/***************************************************
Function To Write Internal Registers of DS1307
---------------------------------------------
address : Address of the register
data: value to write.
Returns:
0= Failure
1= Success
***************************************************/

BOOL DS1307Write(uint8_t address,uint8_t data)
{
	uint8_t res;   //result

	//Start
	SoftI2CStart();

	//SLA+W
	res=SoftI2CWriteByte(DS1307_SLA_W); //DS1307 address + W

	//Error
	if(!res) return FALSE;

	//Now send the address of required register
	res=SoftI2CWriteByte(address);

	//Error
	if(!res) return FALSE;

	//Now write the value

	res=SoftI2CWriteByte(data);

	//Error
	if(!res) return FALSE;

	//STOP
	SoftI2CStop();

	return TRUE;
}

/*
*
* Functions to change the Binary Coded Decimal (BCD) time to regular binary
*
*/

int GetBinMinutes(void)
{
	int minutes;
	int tmp1;
	int tmp;
	uint8_t data;
	//Get minutes
	DS1307Read(0x01,&data);
	//data = minutes
	//first 4 bits of 01 address are the 0-9 range for the last digit of minutes
	//Apply an & with the right mask to extract the active bits
	tmp = (data & 0b00001111);
	//Same for 10 place of minutes, range is 0 to 5
	//Shift right by 4 places to be able to calculate regular decimal
	tmp1 = ((data & 0b01110000)>>4);
	//tmp1 holds a number from 0 to 5, but since it's the first place of
	//the number, multiply it by 10
	tmp1 *= 10;
	minutes = tmp1 + tmp;
	return minutes;
}

int GetBinHours(void)
{
	int hours;
	uint8_t data;

	int tmp;
	int tmp1;
	//Get hours
	DS1307Read(0x02,&data);
	//data = hours
	//first 4 bits of 01 address are the 0-9 range for the last digit of hours
	//Apply an & with the right mask to extract the active bits
	tmp = (data & 0b00001111);
	//Same for 10 place of hours, range is 1 or 2
	//Shift right by 4 places to be able to calculate regular decimal
	tmp1 = ((data & 0b00110000)>>4);
	//tmp1 holds a number from 0 to 5, but since it's the first place of
	//the number, multiply it by 10
	tmp1 *= 10;
	hours = tmp1 + tmp;
	return hours;
}

/*
*
* Function to set the initial time of the DS1307
* Must be edited manually and carefully to set the time.
* Run it once then remove it from the code & reupload.
* Add more lines to set the DATE registers if you wish
* But not needed here
*
*/

BOOL SetTime(uint8_t hour,uint8_t min,uint8_t sec)
{
	uint8_t temp;
	BOOL result;

	result = DS1307Read(0x00,&temp); // read current bits in 00 register
	temp &= ~(0b10000000); //Clear CH Bit (this enables the clock)
	result = DS1307Write(0x00,temp); //write back with ch = 0

	result = DS1307Read(0x02,&temp); //Set 12 Hour Mode
	temp |= 0b01000000; //Set 12 Hour BIT
	result = DS1307Write(0x02,temp); //Write Back to DS1307

	//Set initial time. Check datasheet to know how to set it up
	//set hour to 23
	temp = 0b00000111;
	result = DS1307Write(0x02,hour);
	//set minutes to 40
	temp = 0b01010000;
	result = DS1307Write(0x01,min);
	//set seconds to 0
	temp = 0b00000000;
	result = DS1307Write(0x00,sec);
	if(result == 0) return FALSE;
	return TRUE;
}

BOOL SetDate(uint8_t day,uint8_t date,uint8_t month,uint8_t year)
{
	BOOL result;

	//Set initial time. Check datasheet to know how to set it up
	//set day to 2
	//0b00000010;
	result = DS1307Write(0x03,day);
	//set date to 10
	//0b00001010;
	result = DS1307Write(0x04,date);
	//set month to 1
	//0b00000001;
	result = DS1307Write(0x05,month);
	//set year to 22
	//0b 0001 0110
	result = DS1307Write(0x06,year);
	if(result == 0) return FALSE;
	return TRUE;
}

//Function To Read Internal Registers of DS1307
//---------------------------------------------
//address : Address of the register
//data: value of register is copied to this.
uint8_t ds1307_sw_read(uint8_t address)
{
	uint8_t data,res=2;

	SoftI2CStart();
	res=SoftI2CWriteByte(DS1307_SLA_W); //DS1307 address + W
	//Error
	if(!res) return 0;
	res=SoftI2CWriteByte(address);
	if(!res) return 0;
	//Repeat Start
	SoftI2CStart();
	res=SoftI2CWriteByte(DS1307_SLA_R); //DS1307 Address + R
	if(!res) return 0;
	data = SoftI2CReadByte(0);
	SoftI2CStop();
	return data;
}

#ifdef DEBUGGING

#define BITDELAY _delay_us(26)
#define TXPIN PB1

//simple macros to set port to on or off
#define TXPIN_LOW (PORTB &= ~(1<<TXPIN))
#define TXPIN_HIGH (PORTB |= (1<<TXPIN))

extern void TXInit(void);
extern void SendChar(char);
extern void SendCR(void);
extern void SendString(char *);




void TXRegisters(void)
{
	uint8_t temp1; // variable to hold data
	BOOL result1;
	result1 = DS1307Read(0x00,&temp1);
	SendChar(temp1);
	result1 = DS1307Read(0x01,&temp1);
	SendChar(temp1);
	result1 = DS1307Read(0x02,&temp1);
	SendChar(temp1);
	/*
	result1 = DS1307Read(0x03,&temp1);
	SendChar(temp1);
	result1 = DS1307Read(0x04,&temp1);
	SendChar(temp1);
	result1 = DS1307Read(0x05,&temp1);
	SendChar(temp1);
	result1 = DS1307Read(0x06,&temp1);
	SendChar(temp1);
	*/
	SendChar(0xff);
}

void TXTime(void)
{
	char Time[12];	//hh:mm:ss AM/PM
	//Now Read and format time
	uint8_t data;
	DS1307Read(0x00,&data);
	Time[7]=48+(data & 0b00001111);
	Time[6]=48+((data & 0b01110000)>>4);
	Time[5]=':';
	DS1307Read(0x01,&data);
	Time[4]=48+(data & 0b00001111);
	Time[3]=48+((data & 0b01110000)>>4);
	Time[2]=':';
	DS1307Read(0x02,&data);
	Time[1]=48+(data & 0b00001111);
	Time[0]=48+((data & 0b00110000)>>4);
	SendString(Time);
	SendCR();
}

//giving some garbage characters
void TXTimeX(void)
{
	char Time[12];	//hh:mm:ss AM/PM
	//Now Read and format time
	uint8_t data;
	DS1307Read(0x00,&data);
	Time[7]=48+(data & 0b00001111);
	Time[6]=48+((data & 0b01110000)>>4);
	Time[5]=':';
	DS1307Read(0x01,&data);
	Time[4]=48+(data & 0b00001111);
	Time[3]=48+((data & 0b01110000)>>4);
	Time[2]=':';
	DS1307Read(0x02,&data);
	Time[1]=48+(data & 0b00001111);
	Time[0]=48+((data & 0b00110000)>>4);
	SendString(Time);
	SendCR();
}

void TXBinTime(void)
{
	//get minutes
	uint8_t data;
	DS1307Read(0x01,&data);
	//data = minutes
	int tmp;
	tmp = (data & 0b00001111);
	//at minutes, with &, should be the last digit of minutes
	int tmp1;
	tmp1 = ((data & 0b01110000)>>4);
	//tmp1 should be the first digit of minutes, so multiply by 10
	tmp1 *= 10;
	int minutes;
	minutes = tmp1 + tmp;
	SendChar(minutes);

	//get hours
	DS1307Read(0x02,&data);
	tmp = (data & 0b00001111);
	//last digit of hours
	tmp1 = ((data & 0b00110000)>>4);
	//tmp1 should be the first digit of minutes, so multiply by 10
	tmp1 *= 10;

	int hours;
	hours = tmp1 + tmp;
	SendChar(hours);

	SendChar(0xff);
}

#endif
