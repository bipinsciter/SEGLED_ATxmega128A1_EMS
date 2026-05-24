#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"

//----------------------------------------------------------------------------------------
// I2C FUNCTIONS
//----------------------------------------------------------------------------------------
void I2C_Init(void)
{
	SCL_DIR_OUT;           					// Enable SCL as output.
	SDA_DIR_OUT;           					// Enable SDA as output.
	
	SDA_HIGH;           					// Enable pullup on SDA, to set high as released state.
	SCL_HIGH;						        // Enable pullup on SCL, to set high as released state.
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
unsigned char Read_Byte_I2C(unsigned char ACK_Bit)
{
	unsigned char Data=0,i=0;

    SDA_DIR_IN;	
	
	for (i=0;i<8;i++)
	{
		SCL_HIGH;		
		Data<<= 1;
		
		if(SDA_SENSE) Data  |= 0x01;
		
		_delay_us(5);
		SCL_LOW;
		_delay_us(5);
	}
    
	SDA_DIR_OUT;
	
 	if (ACK_Bit == 1)
		SDA_LOW;  // Send ACK		
	else		
		SDA_HIGH; // Send NO ACK				

	_delay_us(5);
	SCL_HIGH;		
	_delay_us(5);
	SCL_LOW;
	
	return Data;
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
void I2C_Start(void)
{
	SDA_HIGH;
	_delay_us(5);
	SCL_HIGH;
	_delay_us(5);
	SDA_LOW;
	_delay_us(5);
	SCL_LOW;
	_delay_us(5);
}

/*---------------------------------------------------------------
 Function for writing byte
---------------------------------------------------------------*/
unsigned char Write_Byte_I2C(unsigned char datum)
{
	unsigned char i=0,error=0;
	
	SCL_LOW;                // Pull SCL LOW.
	
	for (i=0;i<8;i++)
	{
		if(datum & 0x80) SDA_HIGH;
		else 			 SDA_LOW;
		
		_delay_us(5);
		
		SCL_HIGH;
		_delay_us(5);
		SCL_LOW;
		_delay_us(5);
		
		datum<<=1;
	}

	SDA_DIR_IN;
	
  	SCL_HIGH; 
	_delay_us(5);
	if(SDA_SENSE) error=ACK_ERROR; //check ack from i2c slave
	SCL_LOW;
	
	SDA_DIR_OUT;
	
	return error;                       //return error code
}


/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
void I2C_Stop(void)
{
	SDA_LOW;	    	
	_delay_us(5);
	SCL_HIGH;
	_delay_us(5);
	SDA_HIGH;
	_delay_us(5);
	//SCL_LOW;
	//_delay_us(5);
}
