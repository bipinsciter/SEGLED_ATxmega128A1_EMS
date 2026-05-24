#ifndef _I2C4MASTER_H
#define _I2C4MASTER_H   

#include <avr/io.h>

#define SDA4_HIGH		PORTF_OUTSET = PIN0_bm
#define SDA4_LOW		PORTF_OUTCLR = PIN0_bm
#define SDA4_SENSE		(PORTF_IN & PIN0_bm)
#define SDA4_DIR_IN		PORTF_DIRCLR = PIN0_bm
#define SDA4_DIR_OUT	PORTF_DIRSET = PIN0_bm

#define SCL4_HIGH		PORTF_OUTSET = PIN1_bm
#define SCL4_LOW		PORTF_OUTCLR = PIN1_bm
#define SCL4_SENSE		(PORTF_IN & PIN1_bm)
#define SCL4_DIR_IN		PORTF_DIRCLR = PIN1_bm
#define SCL4_DIR_OUT	PORTF_DIRSET = PIN1_bm

// Error codes
#define ACK_ERROR					0x01

//------------------ I2C ROUTINS for DP2 -----------------------------------------------
void I2C4_Init(void);
void I2C4_Start(void);
void I2C4_Stop(void);
unsigned char Write_Byte_I2C4(unsigned char Data);
unsigned char Read_Byte_I2C4(unsigned char ACK_Bit);

#endif
