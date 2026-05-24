#ifndef _I2CMASTER_H
#define _I2CMASTER_H   

#include <avr/io.h>

#define SDA_HIGH		PORTC_OUTSET = PIN0_bm
#define SDA_LOW			PORTC_OUTCLR = PIN0_bm
#define SDA_SENSE		(PORTC_IN & PIN0_bm)
#define SDA_DIR_IN		PORTC_DIRCLR = PIN0_bm
#define SDA_DIR_OUT		PORTC_DIRSET = PIN0_bm

#define SCL_HIGH		PORTC_OUTSET = PIN1_bm
#define SCL_LOW			PORTC_OUTCLR = PIN1_bm
#define SCL_SENSE		(PORTC_IN & PIN1_bm)
#define SCL_DIR_IN		PORTC_DIRCLR = PIN1_bm
#define SCL_DIR_OUT		PORTC_DIRSET = PIN1_bm

// Error codes
#define ACK_ERROR					0x01

//------------------ I2C ROUTINS for IDT1338, HTU25, DP1 -----------------------------------------------
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
unsigned char Write_Byte_I2C(unsigned char Data);
unsigned char Read_Byte_I2C(unsigned char ACK_Bit);

#endif
