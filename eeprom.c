/*
  eeprom.c - I2C serial eeprom support for LPC2106.  
  Part of Grbl ARM7.   Note that on my board, the LPC2106 I2C pins were allocated
  for other purposes, so the I2C below is bit banged.  

  Copyright (c) 2013 Gary Rensberger

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "lpc210x.h"
#include "config.h"
#include "types.h"
#include "nuts_bolts.h"
#include "eeprom.h"

#define i2cDATA_IN 	IODIR &= ~bfI2C_SDA
#define i2cDATA_OUT IODIR |= bfI2C_SDA
#define i2cCLK_IN 	IODIR &= ~bfI2C_SCL
#define i2cCLK_OUT 	IODIR |= bfI2C_SCL

#define EEPROM_ADDR 0xA0						// 8 bit EEPROM I2C address
#define EEPROM_PAGE_SIZE 32						// 32 bytes in the 'page buffer'
#define EEPROM_PAGE_MASK (EEPROM_PAGE_SIZE-1)	// mask used to know if we hit the end of a page


//-----------------------------------
// private declarations
//-----------------------------------
static uint8_t i2cSendByte(uint8_t data);
static void i2cGetByte(uint8_t *dataPtr);
static void i2cSTART(void);
static void i2cACK(void);                           
static void i2cNAK(void);                          
static void i2cSTOP(void);           	



//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
unsigned char eeprom_get_char( unsigned int addr )
{
	uint8_t temp;
	
	i2cRead(EEPROM_ADDR, addr, &temp, 1);
	return(temp);
}

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
void eeprom_put_char( unsigned int addr, unsigned char value )
{
	i2cWrite(EEPROM_ADDR, addr, &value, 1);
	delay_ms(10);													// 2ms is the page write time, add buffer (could do ACK polling also)
}

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
	uint8_t checksum = 0;
	unsigned int bytesInCurPage;
	unsigned int bytesToWrite;
	
	for(int i=0; i<size; i++) 
	{ 																	// first calculate the checksum
		checksum = (checksum << 1) || (checksum >> 7);
		checksum += *(source+i);
	}

	do																	// now write up to 1 page each time through the loop
	{
		bytesInCurPage = ((~destination) & EEPROM_PAGE_MASK)+1; 		// max bytes we can write in current page
		bytesToWrite = min(size,bytesInCurPage);						// either finish this page, or if last one, partial page.
		
		i2cWrite(EEPROM_ADDR, destination, (uint8_t *)source, bytesToWrite);	// write this page buffer
		delay_ms(10);													// 2ms is the page write time, add buffer (could do ACK polling also)
		source += bytesToWrite;
		destination += bytesToWrite;
		size -= bytesToWrite;
		
	} while (size > 0);
	
	i2cWrite(EEPROM_ADDR, destination, &checksum, 1);
}

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
	unsigned char data, checksum = 0;

	i2cRead(EEPROM_ADDR, source+size, &data, 1);					// get the checksum
	i2cRead(EEPROM_ADDR, source, (uint8_t *)destination,  size);	// read entire array of bytes

	for(; size > 0; size--) 
	{
		checksum = (checksum << 1) || (checksum >> 7);
		checksum += *destination++;    
	}

	return(checksum == data);										// return match result
}


//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
uint8_t i2cWrite(uint8_t addr, uint16_t eeReg, uint8_t *dataPtr, uint16_t count)
{
	uint8_t i2cErr=0;
	
	i2cSTART();                				// start condition
	if ((i2cErr=i2cSendByte(addr))!= 0)	// send Addr Write
	{
		i2cSTOP();
		return(i2cErr);
	}
	
	i2cSendByte(eeReg>>8);
	i2cSendByte(eeReg&0xFF);

	while (count>0)
	{
		if (i2cSendByte(*dataPtr++) != 0)
			break;
		count--;
	}
	i2cSTOP();	
	return(i2cErr);
}

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
uint8_t i2cRead(uint8_t addr, uint16_t eeReg, uint8_t *dataPtr, uint16_t count)
{
	uint8_t i2cErr=0;

	i2cSTART();               				// start condition
	if ((i2cErr=i2cSendByte(addr)) != 0)	// send Addr Write
	{
		i2cSTOP();
		return(i2cErr);
	}
	
	i2cSendByte(eeReg>>8);
	i2cSendByte(eeReg&0xFF);

	i2cSTART();               				// repeated start condition
	i2cErr=i2cSendByte(addr|0x01);			// send addr with Rd bit set
	while (count>0)
	{
		i2cGetByte(dataPtr);				// read next byte from device		
		dataPtr++;
		count--;
		if (count != 0)
			i2cACK();
	}	
	i2cNAK();
	i2cSTOP();
	return(0);
}





//------------------------------------
// Routine:	i2cSendByte
// This routine sends 1 byte to I2C bus.
//------------------------------------
static uint8_t i2cSendByte(uint8_t data)
{
	uint8_t retVal=0;
	
	for (int i=0;i<8;i++)
	{
		if (data&0x80)
			i2cDATA_IN;
		else
			i2cDATA_OUT;
		delay_us(1);
		data <<= 1;
		i2cCLK_IN;
		delay_us(2);
		i2cCLK_OUT;
		delay_us(2);
	}
	i2cDATA_IN;
	delay_us(1);
	i2cCLK_IN;
	delay_us(2);
	if ((IOPIN&bfI2C_SDA)!=0)
		retVal=1;
	i2cCLK_OUT;					// clock low to stop ack
	delay_us(2);
	
	return(retVal);
}
    




//------------------------------------
// Routine:  i2cGetByte
// Inputs:   
// Function: This routine gets 1 byte from the I2C bus.
//------------------------------------
static void i2cGetByte(uint8_t *dataPtr)
{
	uint8_t temp =0x00;			// init receive byte
	
	i2cDATA_IN;					// data line input
	for (int i=0;i<8;i++)
	{
		i2cCLK_IN;				// clock high
		delay_us(2);
		temp<<=1;
		if ((IOPIN&bfI2C_SDA)!=0)
			temp |= 0x01;
		i2cCLK_OUT;				// clock low
		delay_us(2);
	}
	*dataPtr = temp;
}
	

//--------------------------------------------------------
// Routine:  i2cSTART
// This routine sets the start condition
//--------------------------------------------------------
static void i2cSTART(void)
{
	i2cDATA_IN;      			// Data high
	i2cCLK_IN;           		// Force clk hi 
	delay_us(5);              	// Make separation large for this condition
	i2cDATA_OUT;	         	// then Data low first,
	delay_us(5);
	i2cCLK_OUT;          		// followed by clock low
	delay_us(3);
}


//--------------------------------------------------------
// Routine:  i2cACK                                                            
// Function: This routine sends the stop condition                            
//--------------------------------------------------------
static void i2cACK(void)                             
{
	i2cDATA_OUT;	         	// then Data low first,
	i2cCLK_IN;           		// while clk high-low
    delay_us(4);
    i2cCLK_OUT;           		// indicates Ack
    delay_us(2);
    i2cDATA_IN;             	// leave data high
}                                    


//----------------------------------------------------------------
// Routine:  i2cNAK                                                         
// Function: This routine sends /ACK                                
//----------------------------------------------------------------
static void i2cNAK(void)                            
{
 	i2cDATA_IN;	         	// data high
	i2cCLK_IN;           		// while clk high-low
    delay_us(2);
    i2cCLK_OUT;           		// indicates NAK
    delay_us(2);
}


//----------------------------------------------------------------------------
// Routine:  i2cSTOP                                                           
// Function: This routine sends a STOP  
//-----------------------------------------------------------------------------
static void i2cSTOP(void)       //
{
    i2cDATA_OUT;           		// data low
    delay_us(2);				// 
	i2cCLK_IN;           		// clock high followed by
    delay_us(2);
    i2cDATA_IN;          		// data low-high = STOP
    delay_us(2);
}

  
