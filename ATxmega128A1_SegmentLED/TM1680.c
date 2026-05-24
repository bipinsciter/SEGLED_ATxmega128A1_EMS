#include "TM1680.h"
#include "i2cmaster.h"

void TM1680Configure(void)
{
	for(uint8_t i=0;i<NO_OF_LED_DISP;i++)
	{
		I2C_Start();					// Start condition
		Write_Byte_I2C(TM1680_ID+i);	// Write device address
		Write_Byte_I2C(SYS_DISABLE);	
		Write_Byte_I2C(COM_OPTION);	
		Write_Byte_I2C(0x98);
		Write_Byte_I2C(BRIGHTNESS | 15);
		Write_Byte_I2C(SYS_ENABLE);
		Write_Byte_I2C(LED_ON);
		I2C_Stop();              		// Stop condition
	}
}	
	
void TM1680WriteCommand(uint8_t cmd)
{
	for(uint8_t i=0;i<NO_OF_LED_DISP;i++)
	{
		I2C_Start();					// Start condition
		Write_Byte_I2C(TM1680_ID+i);	// Write device address
		Write_Byte_I2C(cmd);			// Write address of register
		I2C_Stop();              		// Stop condition
	}
}

void TM1680Brighness(uint8_t Brightness)
{
	if(Brightness > 15) Brightness = 15;
	
	for(uint8_t i=0;i<NO_OF_LED_DISP;i++)
	{
		I2C_Start();					// Start condition
		Write_Byte_I2C(TM1680_ID+i);	// Write device address
		Write_Byte_I2C(BRIGHTNESS | Brightness);			// Write address of register
		I2C_Stop();              		// Stop condition
	}
}

void TM1680Blink(uint8_t BlinkRate)
{
	if(BlinkRate > 3) BlinkRate = 0;
	
	for(uint8_t i=0;i<NO_OF_LED_DISP;i++)
	{
		I2C_Start();					// Start condition
		Write_Byte_I2C(TM1680_ID+i);	// Write device address
		Write_Byte_I2C(BLINK_REG | BlinkRate);
		I2C_Stop();              		// Stop condition
	}
}

void TM1680WritePage(uint8_t Address, uint8_t *data, uint8_t NoOfByte)
{
	uint8_t bytes,*dataptr;
	
	for(uint8_t i=0;i<NO_OF_LED_DISP;i++)
	{
		bytes=NoOfByte;
		dataptr=data;
		
		I2C_Start();					// Start condition
		Write_Byte_I2C(TM1680_ID+i);	// Write device address
		Write_Byte_I2C(Address);			// Write address of register
	
		while(bytes)
		{	
			Write_Byte_I2C(*dataptr);				// DATA
			dataptr++;								// Point to Next Location	 
			bytes--;
		}
	
		I2C_Stop();              // Send a STOP condition on the TWI bus.
	}
}

void TM1680WriteCmdAndPage(uint8_t Cmd, uint8_t Address, uint8_t *data, uint8_t NoOfByte)
{
	for(uint8_t i=0;i<NO_OF_LED_DISP;i++)
	{
		I2C_Start();					// Start condition
		Write_Byte_I2C(TM1680_ID+i);	// Write device address
		Write_Byte_I2C(Cmd);			// Write address of register
		Write_Byte_I2C(Address);		// Write address of register
	
		while(NoOfByte)
		{	
			Write_Byte_I2C(*data);			// DATA
			data++;							// Point to Next Location	 
			NoOfByte--;
		}
		
		I2C_Stop();              // Send a STOP condition on the TWI bus.
	}
}




