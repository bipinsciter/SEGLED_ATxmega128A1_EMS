#ifndef _I2C3MASTER_H
#define _I2C3MASTER_H   

#include <avr/io.h>

#define SDA3_HIGH		PORTE_OUTSET = PIN0_bm
#define SDA3_LOW		PORTE_OUTCLR = PIN0_bm
#define SDA3_SENSE		(PORTE_IN & PIN0_bm)
#define SDA3_DIR_IN		PORTE_DIRCLR = PIN0_bm
#define SDA3_DIR_OUT	PORTE_DIRSET = PIN0_bm

#define SCL3_HIGH		PORTE_OUTSET = PIN1_bm
#define SCL3_LOW		PORTE_OUTCLR = PIN1_bm
#define SCL3_SENSE		(PORTE_IN & PIN1_bm)
#define SCL3_DIR_IN		PORTE_DIRCLR = PIN1_bm
#define SCL3_DIR_OUT	PORTE_DIRSET = PIN1_bm

// Error codes
#define ACK_ERROR					0x01

//------------------ I2C ROUTINS for DP2 -----------------------------------------------
void I2C3_Init(void);
void I2C3_Start(void);
void I2C3_Stop(void);
unsigned char Write_Byte_I2C3(unsigned char Data);
unsigned char Read_Byte_I2C3(unsigned char ACK_Bit);

#endif
