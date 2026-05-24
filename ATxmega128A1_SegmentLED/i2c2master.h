#ifndef _I2C2MASTER_H
#define _I2C2MASTER_H   

#include <avr/io.h>

#define SDA2_HIGH		PORTD_OUTSET = PIN0_bm
#define SDA2_LOW		PORTD_OUTCLR = PIN0_bm
#define SDA2_SENSE		(PORTD_IN & PIN0_bm)
#define SDA2_DIR_IN		PORTD_DIRCLR = PIN0_bm
#define SDA2_DIR_OUT	PORTD_DIRSET = PIN0_bm

#define SCL2_HIGH		PORTD_OUTSET = PIN1_bm
#define SCL2_LOW		PORTD_OUTCLR = PIN1_bm
#define SCL2_SENSE		(PORTD_IN & PIN1_bm)
#define SCL2_DIR_IN		PORTD_DIRCLR = PIN1_bm
#define SCL2_DIR_OUT	PORTD_DIRSET = PIN1_bm

// Error codes
#define ACK_ERROR					0x01

//------------------ I2C ROUTINS for DP2 -----------------------------------------------
void I2C2_Init(void);
void I2C2_Start(void);
void I2C2_Stop(void);
unsigned char Write_Byte_I2C2(unsigned char Data);
unsigned char Read_Byte_I2C2(unsigned char ACK_Bit);

#endif
