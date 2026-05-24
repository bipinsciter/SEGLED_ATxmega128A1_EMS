/**
 * \file
 *
 * \brief CDC Application Main functions
 *
 * Copyright (c) 2011-2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel micro controller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "SHT25.h"
#include "DS1307.h"
#include "TM1680.h"
#include "i2cmaster.h"
#include "i2c2master.h"
#include "i2c4master.h"

//***********************************************************************************
//#define ENABLE_BATTERY_DISPLAY
//#define ENABLE_KEY_LOGIC
//#define DISABLE_DOOR_SENSING
//#define BOARD_5_13

// Convenience macro for enabling pull-ups on specified pins on any port.
#define __PORT_PULLUP(port, mask) { \
	PORTCFG.MPCMASK = mask ; \
	port.PIN0CTRL = PORT_OPC_PULLUP_gc; \
}

#define DP_ZERO_DISP_LIMIT_LOW		(-1.9)
#define DP_ZERO_DISP_LIMIT_HIGH		(1.9)
#define MAX_SUPPORTED_DP			2

//***********************************************************************************
#define ENABLE_DP1			0x0001
#define DIFP1_ABSP1			0x0002
#define ENABLE_DP2			0x0004
#define ENABLE_TEMP			0x0008

#define ENABLE_RH			0x0010
#define ENABLE_LOGO			0x0020
//#define ENABLE_LOG			0x0040
#define ENABLE_RTC			0x0080

#define ENABLE_LCD			0x0200
#define ENABLE_ALERT		0x0400

#define NO_DIGIT	13
#define PARAMETER_WORD	(ENABLE_DP1 | ENABLE_DP2 | ENABLE_TEMP | ENABLE_RH | ENABLE_LOGO | ENABLE_RTC | ENABLE_LCD | ENABLE_ALERT)

unsigned short gu16_parameterWord = PARAMETER_WORD;
//***********************************************************************************
// USER CONFIGUARATION MODE END
//***********************************************************************************
//#define ENABLE_BROADCAST
//#define ENABLE_PRINTF
//#define DEBUG_DP1
//#define DEBUG_DP2
//#define DEBUG_RCV_CMD
//#define DEBUG_RTC
//#define DEBUG_SHT25
//#define DEBUG_HR_MEAN
//*************************************************************************
#define FACTORY_PARASET_PWD		1234
#define FACTORY_PASSWORD		1000
#define DFU_PASSWORD			3123
#define SOFT_VER				920  //means 9.20
#define NO_OF_ACKPWD			15
#define NO_OF_XBEE_MAC			2
#define NO_OF_DEVICES_IN_GROUP	5
#define XBEE_MAC_SIZE			16
#define FACT_ACK_PWD			1
#define NO_OF_USER_CAL_DATE		10

#define DEFAULT_LCD_BRIGHTNESS	 10
#define DEFAULT_AUTO_SENT_INTERVAL	 5
#define DEFAULT_XBEE_RST_INTERVAL	 360
#define DEFAULT_DEVICES_IN_GROUP	 5


#define DISP_PER			82.0 // PWM
 
//*************************************************************************
#define INIT_RTC_CORRUPTION_CHECK_CNT	5
#define BATTERY_VOLTAGE_READ_TIME		5

//Display Macro ***********************************************************
#define TEMP_DISPLAY			0
#define RH_DISPLAY				1
#define DP1_DISPLAY				2
#define DP2_DISPLAY				3
#define NO_DISPLAY				4

//*************************************************************************
// DISPLAY CONSTANTS
//*************************************************************************

#define ZERO_AP1_COUNT			2200
#define ZERO_DP1_COUNT			8192
#define ZERO_DP2_COUNT			8192
#define PASCAL_PER_CNT			8.0


#define RAW_DP_CNT_IND			10
#define XBEE_RX_IND_MAX			20
#define RX_IND_MAX				100
#define TX_IND_MAX				100


#define MIN_LOG_INTERVAL		1
#define MAX_LOG_INTERVAL		1440

#define DEFAUT_DP1_MIN				981.0
#define DEFAUT_DP1_MAX				(-981.0)
#define DEFAUT_DP2_MIN				981.0
#define DEFAUT_DP2_MAX				(-981.0)
#define DEFAUT_RH_MIN				100
#define DEFAUT_RH_MAX				0
#define DEFAUT_TEMP_C_MIN			125.0
#define DEFAUT_TEMP_C_MAX			(-40.0)
#define DEFAUT_TEMP_F_MIN			257.0
#define DEFAUT_TEMP_F_MAX			(-40.0)
#define DEFAULT_DP1_UPPER_ALM_ON	550
#define DEFAULT_DP1_UPPER_ALM_OFF	500
#define DEFAULT_DP1_LOWER_ALM_ON	(-550)
#define DEFAULT_DP1_LOWER_ALM_OFF	(-500)
#define DEFAULT_DP2_UPPER_ALM_ON	550
#define DEFAULT_DP2_UPPER_ALM_OFF	500
#define DEFAULT_DP2_LOWER_ALM_ON	(-550)
#define DEFAULT_DP2_LOWER_ALM_OFF	(-500)
#define DEFAULT_TM_C_UPPER_ALM_ON	800
#define DEFAULT_TM_C_UPPER_ALM_OFF	750
#define DEFAULT_TM_C_LOWER_ALM_ON	(-350)
#define DEFAULT_TM_C_LOWER_ALM_OFF	(-300)
#define DEFAULT_TM_F_UPPER_ALM_ON	1760
#define DEFAULT_TM_F_UPPER_ALM_OFF	1400
#define DEFAULT_TM_F_LOWER_ALM_ON	(-310)
#define DEFAULT_TM_F_LOWER_ALM_OFF	(-220)
#define DEFAULT_RH_UPPER_ALM_ON		900
#define DEFAULT_RH_UPPER_ALM_OFF	850
#define DEFAULT_RH_LOWER_ALM_ON		150
#define DEFAULT_RH_LOWER_ALM_OFF	200
#define DEFAULT_DEVICE_ID			1
#define DEFAULT_BUZZER_ON_TIME		1		//In seconds
#define DEFAULT_BUZZER_OFF_TIME		2		//In seconds
#define DEFAULT_LOG_INTERVAL		1		//In Minutes
#define DEFAULT_UART_BAUDRATE		8		//57600
#define DEFAULT_UART_DATABITS		3		//8
#define DEFAULT_UART_PARITYBITS		0		//NONE
#define DEFAULT_UART_STOPBITS		0		//1
#define DEFAULT_CUSTOMER_PWD		100
#define DEFAULT_FACTORY_PWD			4321
#define DEFAULT_PARA_SCROLL_TIME	5
//--------------------------------------------------
	
#define BLANK	20
#define DASH	21
#define C		22
#define A		23
#define L		24
#define D		25
#define N		26
#define E		27
#define B		28
#define t		29
#define r		30
#define P		31
#define I		32
#define F		33
#define U		34
#define H		35
#define M		36
#define Y		37
#define V		38

//--------------------------------------------------

#define RAM_BUF_SIZE			2000
#define RAM_FILL_START			5

//------------------------------------------------------------------------------------

#define NORMAL_LOG				0
#define DP1_ALM_OCCURE_LOG		1
#define DP1_ALM_RESTORE_LOG		2
#define DP2_ALM_OCCURE_LOG		3
#define DP2_ALM_RESTORE_LOG		4
#define TM_ALM_OCCURE_LOG		5
#define TM_ALM_RESTORE_LOG		6
#define RH_ALM_OCCURE_LOG		7
#define RH_ALM_RESTORE_LOG		8
#define ALM_ACK_LOG				9
#define POWER_UP_LOG			10

#define NO_ALARM				0
#define UPPER_ALARM				1
#define LOWER_ALARM				2

#define PROG_ENT				0x01
#define PARA_SELECT				0x02

#define PROG_ENT_KEY			(PORTK_IN & PIN0_bm)
#define PARA_SELECT_KEY			(PORTK_IN & PIN1_bm)
#define UP_KEY					(PORTK_IN & PIN2_bm)
#define DN_KEY					(PORTK_IN & PIN3_bm)

#define DOOR_SENSE				(PORTB_IN & PIN3_bm)
#define DP_CHANGE_SENSE			(PORTD_IN & BIT0)

#define BUZZER_DIR_OP			PORTB_DIRSET = PIN3_bm
#define BUZZER_ON 				PORTB_OUTSET = PIN3_bm
#define BUZZER_OFF				PORTB_OUTCLR = PIN3_bm
#define TOGGLE_BUZZER			PORTB_OUTTGL = PIN3_bm

#define RS485_TX0_DIR_OP		PORTC_DIRSET = PIN4_bm
#define RS485_TX0_ENB 			PORTC_OUTSET = PIN4_bm
#define RS485_TX0_DIS 			PORTC_OUTCLR = PIN4_bm

#define RS485_TX1_DIR_OP		PORTC_DIRSET = PIN5_bm
#define RS485_TX1_ENB 			PORTC_OUTSET = PIN5_bm
#define RS485_TX1_DIS 			PORTC_OUTCLR = PIN5_bm

#define XBEE_RST_DIR_OP			PORTH_DIRSET = PIN2_bm
#define XBEE_RST_HIGH			PORTH_OUTSET = PIN2_bm
#define XBEE_RST_LOW			PORTH_OUTCLR = PIN2_bm

//#define PROG_ENT_KEY			(PORTD_IN & PIN4_bm)
//#define PARA_SELECT_KEY			(PORTD_IN & PIN5_bm)
//#define UP_KEY					(PORTD_IN & PIN6_bm)
//#define DN_KEY					(PORTD_IN & PIN7_bm)
//
//#define DOOR_SENSE				(PORTB_IN & PIN3_bm)
//#define DP_CHANGE_SENSE			(PORTD_IN & BIT0)
//
//#define BUZZER_DIR_OP			PORTH_DIRSET = PIN0_bm
//#define BUZZER_ON 				PORTH_OUTSET = PIN0_bm
//#define BUZZER_OFF				PORTH_OUTCLR = PIN0_bm
//#define TOGGLE_BUZZER			PORTH_OUTTGL = PIN0_bm
	//
//#define RS485_TX0_DIR_OP		PORTC_DIRSET = PIN4_bm
//#define RS485_TX0_ENB 			PORTC_OUTSET = PIN4_bm
//#define RS485_TX0_DIS 			PORTC_OUTCLR = PIN4_bm
//
//#define RS485_TX1_DIR_OP		PORTC_DIRSET = PIN5_bm
//#define RS485_TX1_ENB 			PORTC_OUTSET = PIN5_bm
//#define RS485_TX1_DIS 			PORTC_OUTCLR = PIN5_bm
//
//#define RELAY1_DIR_OP			PORTA_DIRSET = BIT0
//#define RELAY1_ON				PORTA_OUTSET = BIT0
//#define RELAY1_OFF				PORTA_OUTCLR = BIT0
//#define RELAY1_STAT				(PORTA_OUT & BIT0)
//
//#define RELAY2_DIR_OP			PORTA_DIRSET = BIT1
//#define RELAY2_ON				PORTA_OUTSET = BIT1
//#define RELAY2_OFF				PORTA_OUTCLR = BIT1
//#define RELAY2_STAT				(PORTA_OUT & BIT1)
//
//#define RELAY3_DIR_OP			PORTA_DIRSET = BIT2
//#define RELAY3_ON				PORTA_OUTSET = BIT2
//#define RELAY3_OFF				PORTA_OUTCLR = BIT2
//#define RELAY3_STAT				(PORTA_OUT & BIT2)
//
//#define RELAY4_DIR_OP			PORTA_DIRSET = BIT3
//#define RELAY4_ON				PORTA_OUTSET = BIT3
//#define RELAY4_OFF				PORTA_OUTCLR = BIT3
//#define RELAY4_STAT				(PORTA_OUT & BIT3)
//
//#define RELAY5_DIR_OP			PORTA_DIRSET = BIT4
//#define RELAY5_ON				PORTA_OUTSET = BIT4
//#define RELAY5_OFF				PORTA_OUTCLR = BIT4
//#define RELAY5_STAT				(PORTA_OUT & BIT4)
//
//#define RELAY6_DIR_OP			PORTA_DIRSET = BIT5
//#define RELAY6_ON				PORTA_OUTSET = BIT5
//#define RELAY6_OFF				PORTA_OUTCLR = BIT5
//#define RELAY6_STAT				(PORTA_OUT & BIT5)
//
//#define RELAY7_DIR_OP			PORTA_DIRSET = BIT6
//#define RELAY7_ON				PORTA_OUTSET = BIT6
//#define RELAY7_OFF				PORTA_OUTCLR = BIT6
//#define RELAY7_STAT				(PORTA_OUT & BIT6)
//
//#define RELAY8_DIR_OP			PORTA_DIRSET = BIT7
//#define RELAY8_ON				PORTA_OUTSET = BIT7
//#define RELAY8_OFF				PORTA_OUTCLR = BIT7
//#define RELAY8_STAT				(PORTA_OUT & BIT7)
//
//#define XBEE_RST_DIR_OP			PORTH_DIRSET = PIN2_bm
//#define XBEE_RST_HIGH			PORTH_OUTSET = PIN2_bm
//#define XBEE_RST_LOW			PORTH_OUTCLR = PIN2_bm
//--------------------------------------------------

#define SHR(a, b) (-1 >> 1 == -1 ? (a) >> (b) : (a) / (1 << (b)) - ((a) % (1 << (b)) < 0))

#define EPOCH_YEAR				1970
#define TM_YEAR_BASE			2000

//************************************************************************/
// RS485 PARAMETER
//************************************************************************/
	
#define PARA_WRITE_CMD				0x11
#define PARA_READ_CMD				0x10
#define PARA_WITH_ALM_READ_CMD		0x00
//-------------------------------------------------
#define NO_ERROR			0x00
#define INVALID_CMD			0x01
#define INVALID_PARA		0x02
#define DP1_FAULTY			0x04
#define RH_TEMP_FAULTY		0x08
#define DP2_FAULTY			0x10
//-------------------------------------------------
#define DP1UAOFF_ID			0x01
#define DP1UAON_ID			0x02
#define DP1LAOFF_ID			0x03
#define DP1LAON_ID			0x04
#define DP2UAOFF_ID			0x05				
#define DP2UAON_ID			0x06		
#define DP2LAOFF_ID			0x07		
#define DP2LAON_ID			0x08		
#define TMUAOFF_ID			0x09		
#define TMUAON_ID			0x0A		
#define TMLAOFF_ID			0x0B		
#define TMLAON_ID			0x0C		
#define RHUAOFF_ID			0x0D		
#define RHUAON_ID			0x0E		
#define RHLAOFF_ID			0x0F	
#define RHLAON_ID			0x10		
#define HR_ID				0x11
#define MN_ID				0x12		
#define SEC_ID				0x13
#define YR_ID				0x14		
#define MNTH_ID				0x15		
#define DT_ID				0x16		
#define LOGINTVAL_ID		0x19		
#define DVCID_ID			0x1A		
#define DP1MIN_ID			0x1C
#define DP1MAX_ID			0x1D
#define DP2MIN_ID			0x1E		
#define DP2MAX_ID			0x1F
#define TMMIN_ID			0x20		
#define TMMAX_ID			0x21
#define RHMIN_ID			0x22		
#define RHMAX_ID			0x23
#define TMUNT_ID			0x2E
#define DP1CAL_ID			0x30
#define DP2CAL_ID			0x31
#define TMCAL_ID			0x32
#define RHCAL_ID			0x33
#define SFVER_ID			0x34
#define BZRON_ID			0x35		
#define BZROFF_ID			0x36
#define CAL_FPWD_ID			0x37		
#define CAL_CPWD_ID			0x38
#define CPWD_ID				0x39
#define ACK_TIMER_ID		0x3A
#define ACK_PW_ID			0x3B
#define ALM_ACK_ID			0x3C
#define BATPER_ID			0x3D
#define BRDSTP_ID			0x3E
#define BRDSTR_ID			0x3F
#define SRNO_ID				0x40
#define UBRT_ID				0x41
#define UDBT_ID				0x42
#define UPRT_ID				0x43
#define USTB_ID				0x44
#define RAM_ALL_ID			0x45
#define RAM_IND_ID			0x46
#define FLASH24_IND_ID		0x47
#define REALTIME_VAL_ID		0x48
#define RDLG_DT_ID			0x49
#define RDLG_CNT_ID			0x4A	
#define DATETIME_ID			0x4B
#define FLASH24_CUR_IND_ID	0x4C
#define FCPWD_ID			0x4D
#define SET_DPARA_PWD_ID	0x4E
#define SCROLL_TIME_ID		0x4F
#define EXT_FLASH_ERASE_ID	0x50
#define DFLT_RTC_ID			0x51
#define DFLT_CAL_ID			0x52
#define MINMAXMEAN_IND_ID	0x53
#define MEAN_HR_ID			0x54
#define BACKLIT_ID			0x55
#define TM_RH_SCAN_TIME_ID	0x56
#define BIG_FONT_LED_SET_ID	0x57
#define MENB_ID				0x58
#define DOOR_SENSE_POLARITY_ID		0x5A
#define DOOR_SENSE_TIME_ID			0x5B
#define DP1_ALM_SENSE_TIME_ID		0x5C
#define DP2_ALM_SENSE_TIME_ID		0x5D
#define LCD_BRIGHT_CNT_ID			0x5E
#define DP_SW_FACT_ID				0x5F
#define RELAY_CNTL_ID				0x60
#define AUTO_SENT_INTERVAL_ID		0x69
#define XBEE_RST_INTERVAL_ID		0x6A
#define XBEE_MAC_ADDR_ID			0x6B
#define DEVICES_IN_GROUP_ID			0x6C
#define XBEE_SELF_MAC_ADDR_ID		0x6D
#define DP_LIMIT_ID					0x6E

#define CORR_RTC_DATA_ID			0x99	
#define PUT_DFU_ID					0xAA	
//************************************************************************/
// UART PARAMETER
//************************************************************************/

#define BAUD_1200		0
#define BAUD_2400		1
#define BAUD_4800		2
#define BAUD_9600		3
#define BAUD_14400		4
#define BAUD_19200		5
#define BAUD_28800		6 
#define BAUD_38400		7
#define BAUD_57600		8
#define BAUD_115200		9

#define DATABIT_5		0
#define DATABIT_6		1
#define DATABIT_7		2
#define DATABIT_8		3

#define PARITY_NONE		0
#define PARITY_EVEN		1
#define PARITY_ODD		2

#define STOP_BIT_1		0
#define STOP_BIT_2		1

//************************************************************************/
// EEPROM LOCATION FOR PARAMETERS
//************************************************************************/
#define DFU_NUMBER_LOGIC		0
#define FIRST_BOOT_CHECK		(DFU_NUMBER_LOGIC+2)
#define DISP_PARA_SELECT		(FIRST_BOOT_CHECK+1)
#define DUMMY_ADDR				(DISP_PARA_SELECT+2)
#define DP1_UP_ALM_ON			(DUMMY_ADDR+1)
#define DP1_UP_ALM_OFF			(DP1_UP_ALM_ON+2)
#define DP1_LO_ALM_ON			(DP1_UP_ALM_OFF+2)
#define DP1_LO_ALM_OFF			(DP1_LO_ALM_ON+2)
#define DP2_UP_ALM_ON			(DP1_LO_ALM_OFF+2)
#define DP2_UP_ALM_OFF			(DP2_UP_ALM_ON+2)
#define DP2_LO_ALM_ON			(DP2_UP_ALM_OFF+2)
#define DP2_LO_ALM_OFF			(DP2_LO_ALM_ON+2)
#define DP1_CAL_CNT				(DP2_LO_ALM_OFF+2)
#define DP1_CAL_CNT_C			(DP1_CAL_CNT+2)
#define DP2_CAL_CNT				(DP1_CAL_CNT_C+2)
#define DP2_CAL_CNT_C			(DP2_CAL_CNT+2)
#define TEMP_UP_ALM_ON			(DP2_CAL_CNT_C+2)
#define TEMP_UP_ALM_OFF			(TEMP_UP_ALM_ON+2)
#define TEMP_LO_ALM_ON			(TEMP_UP_ALM_OFF+2)
#define TEMP_LO_ALM_OFF			(TEMP_LO_ALM_ON+2)
#define TEMP_CAL_CNT			(TEMP_LO_ALM_OFF+2)
#define TEMP_CAL_CNT_C			(TEMP_CAL_CNT+2)
#define TEMP_UNIT				(TEMP_CAL_CNT_C+2)
#define RH_UP_ALM_ON			(TEMP_UNIT+1)
#define RH_UP_ALM_OFF			(RH_UP_ALM_ON+2)
#define RH_LO_ALM_ON			(RH_UP_ALM_OFF+2)
#define RH_LO_ALM_OFF			(RH_LO_ALM_ON+2)
#define RH_CAL_CNT				(RH_LO_ALM_OFF+2)
#define RH_CAL_CNT_C			(RH_CAL_CNT+2)
#define LAST_DP1_ALRM_STAT		(RH_CAL_CNT_C+2)
#define LAST_DP2_ALRM_STAT		(LAST_DP1_ALRM_STAT+1)
#define LAST_TM_ALRM_STAT		(LAST_DP2_ALRM_STAT+1)
#define LAST_RH_ALRM_STAT		(LAST_TM_ALRM_STAT+1)
#define DEVICE_ID				(LAST_RH_ALRM_STAT+1)
#define BUZZER_ON_TIME			(DEVICE_ID+1)
#define BUZZER_OFF_TIME			(BUZZER_ON_TIME+2)
#define LOG_INTERVAL			(BUZZER_OFF_TIME+2)
#define UART_BAUDRATE			(LOG_INTERVAL+2)
#define UART_DATBITS			(UART_BAUDRATE+1)
#define UART_PARITY				(UART_DATBITS+1)
#define UART_STOPBIT			(UART_PARITY+1)
#define DP1_MAXIMUM				(UART_STOPBIT+1)
#define DP1_MINIMUM				(DP1_MAXIMUM+4)
#define DP2_MAXIMUM				(DP1_MINIMUM+4)
#define DP2_MINIMUM				(DP2_MAXIMUM+4)
#define TEMP_MAXIMUM			(DP2_MINIMUM+4)
#define TEMP_MINIMUM			(TEMP_MAXIMUM+4)
#define RH_MAXIMUM				(TEMP_MINIMUM+4)
#define RH_MINIMUM				(RH_MAXIMUM+4)
#define CUSTOMER_PASSWORD		(RH_MINIMUM+4)
#define FAC_CUSTOMER_PASSWORD	(CUSTOMER_PASSWORD+2)
#define ACK_TIMER				(FAC_CUSTOMER_PASSWORD+2)
#define ACK_PWD_IND				(ACK_TIMER+2)
#define ACK_PASSWORD			(ACK_PWD_IND+1)
#define DEVICE_SR_NO			(ACK_PASSWORD+30)
#define CURR_LOG_IND_RDLC		(DEVICE_SR_NO+16)
#define FLSH_OVF_IND			(CURR_LOG_IND_RDLC+2)
#define CURR_LOG_IND			(FLSH_OVF_IND+1)
#define CURR_LOG24_IND_RDLC		(CURR_LOG_IND+400)
#define CURR_LOG24_IND			(CURR_LOG24_IND_RDLC+1)
#define PARA_SCROLL_TIME		(CURR_LOG24_IND+2)
#define RTC_SET_FLAG_ADDR		(PARA_SCROLL_TIME+1)
#define DP1_CAL_DATE_ADDR		(RTC_SET_FLAG_ADDR+1)
#define DP2_CAL_DATE_ADDR		(DP1_CAL_DATE_ADDR+12)
#define TM_CAL_DATE_ADDR		(DP2_CAL_DATE_ADDR+12)
#define RH_CAL_DATE_ADDR		(TM_CAL_DATE_ADDR+12)
#define DP1_CAL_CERT_ADDR		(RH_CAL_DATE_ADDR+12)
#define DP2_CAL_CERT_ADDR		(DP1_CAL_CERT_ADDR+15)
#define TM_CAL_CERT_ADDR		(DP2_CAL_CERT_ADDR+15)
#define RH_CAL_CERT_ADDR		(TM_CAL_CERT_ADDR+15)
#define DP1_USER_CAL_DATE_IND_ADDR	(RH_CAL_CERT_ADDR+15)
#define DP2_USER_CAL_DATE_IND_ADDR	(DP1_USER_CAL_DATE_IND_ADDR+1)
#define TM_USER_CAL_DATE_IND_ADDR	(DP2_USER_CAL_DATE_IND_ADDR+1)
#define RH_USER_CAL_DATE_IND_ADDR	(TM_USER_CAL_DATE_IND_ADDR+1)
#define DP1_USER_CAL_DATE_ADDR	(RH_USER_CAL_DATE_IND_ADDR+1)
#define DP2_USER_CAL_DATE_ADDR	(DP1_USER_CAL_DATE_ADDR+100)
#define TM_USER_CAL_DATE_ADDR	(DP2_USER_CAL_DATE_ADDR+100)
#define RH_USER_CAL_DATE_ADDR	(TM_USER_CAL_DATE_ADDR+100)
#define DP1_CAL_VAL_F_ADDR		(RH_USER_CAL_DATE_ADDR+100)
#define DP2_CAL_VAL_F_ADDR		(DP1_CAL_VAL_F_ADDR+2)
#define TM_CAL_VAL_F_ADDR		(DP2_CAL_VAL_F_ADDR+2)
#define RH_CAL_VAL_F_ADDR		(TM_CAL_VAL_F_ADDR+2)
#define DP1_CAL_VAL_C_ADDR		(RH_CAL_VAL_F_ADDR+2)
#define DP2_CAL_VAL_C_ADDR		(DP1_CAL_VAL_C_ADDR+2)
#define TM_CAL_VAL_C_ADDR		(DP2_CAL_VAL_C_ADDR+2)
#define RH_CAL_VAL_C_ADDR		(TM_CAL_VAL_C_ADDR+2)
#define RTC_SET_FLAG1_ADDR		(RH_CAL_VAL_C_ADDR+2)
#define RTC_SET_FLAG2_ADDR		(RTC_SET_FLAG1_ADDR+1)
#define CORRUPT_RTC_IND_ADDR	(RTC_SET_FLAG2_ADDR+1)
#define CORRUPT_RTC_DATA_ADDR	(CORRUPT_RTC_IND_ADDR+1)
#define MIN_MAX_LOG_IND_ADDR	(CORRUPT_RTC_DATA_ADDR+150)
#define BACKLIT_ON_OFF_ADDR		(MIN_MAX_LOG_IND_ADDR+5)
#define TM_RH_SCAN_TIME_ADDR	(BACKLIT_ON_OFF_ADDR+1)
#define DP1_LED_SETTING_ADDR	(TM_RH_SCAN_TIME_ADDR+1)
#define DP2_LED_SETTING_ADDR	(DP1_LED_SETTING_ADDR+1)
#define TM_LED_SETTING_ADDR		(DP2_LED_SETTING_ADDR+1)
#define RH_LED_SETTING_ADDR		(TM_LED_SETTING_ADDR+1)
#define MASTER_ENABLE_ADDR		(RH_LED_SETTING_ADDR+1)
#define DOOR_SENSE_POLARITY_ADDR		(MASTER_ENABLE_ADDR+1)
#define DOOR_SENSE_TIME_ADDR			(DOOR_SENSE_POLARITY_ADDR+1)
#define DP1_ALM_SENSE_TIME_ADDR			(DOOR_SENSE_TIME_ADDR+1)
#define DP2_ALM_SENSE_TIME_ADDR			(DP1_ALM_SENSE_TIME_ADDR+1)
#define DP_AUTO_CAL_FLAG				(DP2_ALM_SENSE_TIME_ADDR+1)
#define LCD_BRIGHT_CNT_ADDR				(DP_AUTO_CAL_FLAG+1)
#define DP_SW_FACT_ADDR					(LCD_BRIGHT_CNT_ADDR+1)
#define DP_FACT_ENB_ADDR				(DP_SW_FACT_ADDR+10)
#define RELAY_STAT_ADDR					(DP_FACT_ENB_ADDR+2)
#define AUTO_SENT_INTERVAL_ADDR			(RELAY_STAT_ADDR+1)
#define XBEE_RST_INTERVAL_ADDR			(AUTO_SENT_INTERVAL_ADDR+1)
#define BROADCAST_ENB_ADDR				(XBEE_RST_INTERVAL_ADDR+2)
#define DEVICES_IN_GROUP_ADDR			(BROADCAST_ENB_ADDR+1)
#define XBEE_MAC_ADDR					(DEVICES_IN_GROUP_ADDR+1)
#define DP_LIMIT_ADDR					(XBEE_MAC_ADDR+6)


//************************************************************************/
// SMALL FONT SEGMENT LCD CONSTANT DEFINATIONS
//************************************************************************/
#define RH_MIN_on			final_buffer[7] |= BIT0;
#define RH_MIN_ALM_on		final_buffer[7] |= BIT1;
#define RH_UNIT_on 			final_buffer[7] |= BIT6;
#define RH_LOGO_ALM_on 		{final_buffer[28] |= BIT2;final_buffer[29] |= BIT2;final_buffer[30] |= BIT2;final_buffer[31] |= BIT2;}
#define RH_LOGO_on 			{final_buffer[28] |= BIT3;final_buffer[29] |= BIT3;final_buffer[30] |= BIT3;final_buffer[31] |= BIT3;}

//-------------------------------------------------
#define TM_MIN_on			final_buffer[15] |= BIT2;
#define TM_MIN_ALM_on		final_buffer[15] |= BIT3;
#define TM_UNIT_C_on 		final_buffer[27] |= BIT1;
#define TM_UNIT_F_on 		final_buffer[28] |= BIT1;
#define TM_LOGO_ALM_on 			{final_buffer[23] |= BIT2;final_buffer[24] |= BIT2;final_buffer[25] |= BIT2;final_buffer[26] |= BIT2;final_buffer[27] |= BIT2;}
#define TM_LOGO_on 		{final_buffer[23] |= BIT3;final_buffer[24] |= BIT3;final_buffer[25] |= BIT3;final_buffer[26] |= BIT3;final_buffer[27] |= BIT3;}

//-------------------------------------------------
#define DP_UNIT_on 			{final_buffer[20] |= BIT6;final_buffer[21] |= BIT6;}
#define DP_MIN_on			final_buffer[31] |= BIT4;
#define DP_MIN_ALM_on		final_buffer[31] |= BIT5;
#define DP_LOGO_ALM_on 		{final_buffer[16] |= BIT6;final_buffer[17] |= BIT6;final_buffer[18] |= BIT6;final_buffer[19] |= BIT6;}
#define DP_LOGO_on 			{final_buffer[16] |= BIT7;final_buffer[17] |= BIT7;final_buffer[18] |= BIT7;final_buffer[19] |= BIT7;}
	
//-------------------------------------------------
#define RTC_BC1_on 			{final_buffer[23] |= BIT1; final_buffer[24] |= BIT1;}
#define RTC_COL_on			final_buffer[23] |= BIT0;
#define RTC_AM_on 			final_buffer[25] |= BIT1;
#define RTC_PM_on 			final_buffer[26] |= BIT1;

//-------------------------------------------------
#define MIN_on 				final_buffer[4] |= BIT6;
#define MAX_on 				final_buffer[9] |= BIT6;
#define MEAN_on 			final_buffer[10] |= BIT6;
#define SET_on 				final_buffer[6] |= BIT6;
#define ID_on 				final_buffer[5] |= BIT6;
#define ACK_on 				final_buffer[8] |= BIT6;
#define LOGO_on 			{final_buffer[0] |= BIT6;final_buffer[1] |= BIT6;final_buffer[2] |= BIT6;final_buffer[3] |= BIT6;}

#define DOOR_on 			{final_buffer[22] |= BIT6; final_buffer[23] |= BIT6;}
#define DOOR_SYM_on 		{final_buffer[29] |= BIT1; final_buffer[30] |= BIT1;}
#define DOOR_ALM_on 		{final_buffer[22] |= BIT7; final_buffer[23] |= BIT7;}

//-------------------------------------------------
#define OPEN			1
#define CLOSE			0

//Display Mode ---------------------------------------------------------------------------------------------
#define NORMAL_MODE			0
#define PROG_MODE			1
#define DP_AUTO_CAL_MODE	2
//----------------------------------------------------------------------------------------------------------

#define PROG_CNT			5
#define DEBOUNCE			5
//**********************************************************************************************************
//											VARIABLE DECLARATION
//**********************************************************************************************************
volatile static struct bits
{
	uint8_t AM_PM_Flag : 1;	
	uint8_t Sec_blink_flag : 1;	
	uint8_t RTCChangeOccure : 1;
	uint8_t sec_flag : 1;	
	uint8_t msec500_flag : 1;	
	uint8_t msec250_flag : 1;
	uint8_t msec100_flag : 1;
	uint8_t msec50_flag : 1;
	uint8_t buzzerStart : 1;
	uint8_t UARTChanged : 1;	
	uint8_t keybit : 1;
	uint8_t cal_mode : 1;
	uint8_t DP1_NC : 1;
	uint8_t DP2_NC : 1;
	uint8_t RH_TEMP_NC : 1;
	uint8_t msgRcvOK : 1;
	uint8_t paraIdNotValid : 1;
	uint8_t Log_Enb_Dis : 1;
	uint8_t FactoryCalibrationOn : 1;
	uint8_t CustmerCalibrationOn : 1;
	uint8_t SetACKPwd : 2;
	uint8_t brodcastEnb : 1;
	uint8_t alarmAutorestore : 1;
	uint8_t led_toggle : 1;
	uint8_t DP1Log : 1;
	uint8_t DP2Log : 1;
	uint8_t TMLog : 1;
	uint8_t RHLog : 1;
	uint8_t FlashReadCmd : 1;
	uint8_t Flash24ReadCmd : 1;
	uint8_t RamReadCmd : 1;
	uint8_t RamAllReadCmd : 1;
	uint8_t logtransferStart : 1;
	uint8_t EraseFlash : 1;
	uint8_t usb_sense_flag : 1;
	uint8_t usb_sense_flag1 : 1;
	uint8_t resetMinMax : 1;
	uint8_t logDataflag : 1;
	uint8_t blink : 1;
	uint8_t batteryPerBlink : 1;
	uint8_t resetDevice : 1;
	//uint8_t rtcCorrupt : 1;
	//uint8_t rtcValid : 1;
	uint8_t mec500_blink_flag : 1;
	uint8_t writeMinMax : 1;
	uint8_t MinMaxMeanLogReadCmd : 1;
	uint8_t MeanHrLogReadCmd : 1;
	uint8_t noData : 1;
	uint8_t doorStatus : 1;
	uint8_t AlarmLED : 1;
	uint8_t buzzeralert : 1;
	uint8_t autoSendResponse : 1;
	uint8_t triggerXbeeReset : 1;
	uint8_t DP1_limit : 2;
	uint8_t DP2_limit : 2;
	
}b={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

struct lcdbits
{
	uint8_t Sym_LOGO : 1;	
	
	uint8_t Sym_DOOR : 1;	
	uint8_t Sym_DOOR_SYM : 1;	
	uint8_t Sym_DOOR_ALM : 1;	
	
	uint8_t Sym_MIN : 1;	
	uint8_t Sym_MAX : 1;	
	uint8_t Sym_MEAN : 1;	
	uint8_t Sym_SET : 1;
	uint8_t Sym_ID : 1;	
	uint8_t Sym_ACK : 1;	
	
	uint8_t Sym_RTC_AM : 1;	
	uint8_t Sym_RTC_PM : 1;	
	uint8_t Sym_RTC_BC1 : 1;	
	uint8_t Sym_RTC_COL : 1;	
	
	uint8_t Sym_DP_LOGO : 1;
	uint8_t Sym_DP_LOGO_ALM : 1;
	uint8_t Sym_DP_UNIT : 1;
	uint8_t Sym_DP_MIN : 1;
	uint8_t Sym_DP_MIN_ALM : 1;
	
	uint8_t Sym_RH_LOGO : 1;
	uint8_t Sym_RH_LOGO_ALM : 1;
	uint8_t Sym_RH_UNIT : 1;
	uint8_t Sym_RH_MIN : 1;
	uint8_t Sym_RH_MIN_ALM : 1;
	
	uint8_t Sym_TM_LOGO : 1;
	uint8_t Sym_TM_LOGO_ALM : 1;
	uint8_t Sym_TM_UNIT_C : 1;
	uint8_t Sym_TM_UNIT_F : 1;
	uint8_t Sym_TM_MIN : 1;
	uint8_t Sym_TM_MIN_ALM : 1;
	
}lcd={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


uint8_t rtcCorrupt=0;
uint8_t rtcValid=0;
uint8_t RTCCorruptDataInd=0;

uint8_t final_buffer[32];
uint8_t disp_buffer[NO_DIGIT];
uint8_t data[NO_DIGIT];
	
uint8_t seg_code[]=
{
	0x3F,  //code for 0
	0x06,  //code for 1
	0x5B,  //code for 2
	0x4F,  //code for 3
	0x66,  //code for 4
	0x6D,  //code for 5
	0x7D,  //code for 6
	0x07,  //code for 7
	0x7F,  //code for 8
	0x6F,  //code for 9
	0xBF,  //code for 0.
	0x86,  //code for 1.
	0xDB,  //code for 2.
	0xCF,  //code for 3.
	0xE6,  //code for 4.
	0xED,  //code for 5.
	0xFD,  //code for 6.
	0x87,  //code for 7.
	0xFF,  //code for 8.
	0xEF,  //code for 9.
	0x00,  //code for BLANK
	0x40,  //code for -
	0x39,  //code for C
	0x77,  //code for A
	0x38,  //code for L
	0x5E,  //code for d
	0x54,  //code for n
	0x79,  //code for E
	0x7C,  //code for B
	0x78,  //code for t
	0x50,  //code for r
	0x73,  //code for P
	0x30,  //code for I
	0x71,  //code for F
	0x3E,  //code for U
	0x76,  //code for H
	0x15,  //code for M
	0x6E,  //code for small y
	0x2A,  //code for V
	0xC0   //code for -.
};

uint8_t i=0,j=0,k=0;
uint8_t prog_para_cnt=0,Normal_para_cnt=0,autoCal_para_cnt=0,para_cnt1=TEMP_DISPLAY,Lastpara_cnt;					
uint8_t key_para;			
uint8_t count;
uint8_t disp_digit;
uint8_t *front_ptr;
uint8_t key_count;
uint8_t para_cal=0;

uint8_t DeviceID=0;
unsigned short Buzzer_ON_Time=0,Buzzer_OFF_Time=0,LogInterval=0;
unsigned short RH_Upper_Alm_ON=0,RH_Upper_Alm_OFF=0,RH_Lower_Alm_ON=0,RH_Lower_Alm_OFF=0;
signed short DP1_Upper_Alm_ON=0,DP1_Upper_Alm_OFF=0,DP1_Lower_Alm_ON=0,DP1_Lower_Alm_OFF=0;
signed short DP2_Upper_Alm_ON=0,DP2_Upper_Alm_OFF=0,DP2_Lower_Alm_ON=0,DP2_Lower_Alm_OFF=0;
signed short TM_Upper_Alm_ON=0,TM_Upper_Alm_OFF=0,TM_Lower_Alm_ON=0,TM_Lower_Alm_OFF=0;
//signed short DP1_Cal_Count=0,DP2_Cal_Count=0,TM_Cal_Count=0,RH_Cal_Count=0;
//signed short DP1_Cal_Count_C=0,DP2_Cal_Count_C=0,TM_Cal_Count_C=0,RH_Cal_Count_C=0;
//signed short AutocalCnt=0;
uint8_t TM_Unit=0;
float DP1_Max=0,DP1_Min=0,DP1_Mean=0;
float DP2_Max=0,DP2_Min=0,DP2_Mean=0;
float TM_Max=0,TM_Min=0,TM_Mean=0;
float RH_Max=0,RH_Min=0,RH_Mean=0;
uint8_t UART_BaudRate=3,UART_DataBits=3,UART_Parity=0,UART_StopBit=0;
signed short dummy=0,dummy1=0;
uint8_t key_up_count=0,key_dn_count=0;
unsigned short buzzerOnTime=0,buzzerOffTime=0;
unsigned short ConfiguPassword=0;
uint8_t test[20];
uint8_t Temp_RTC_ARR[5]={0};
//uint8_t gu8_DPAutoCalFlag=0,gu8_DPAutoCalTimer=0;
unsigned short gu16_DPAutoCalTimer10Sec[2] = {0},gu16_DPAutoCalTimer5Min[2] = {0};
unsigned char gu8_DPAutoCalDoorCnt[2] = {0};

struct RTCData
{
	unsigned short year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	
}rtc,rtc1,rtc2,rtc3;

const unsigned short int __mon_yday[2][13] =
{
	/* Normal years.  */
	{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
	/* Leap years.   */
	{ 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }
};

const int _ytab[2][12] =
{
	{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
	{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

#define LEAPYEAR(year)          (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)          (LEAPYEAR(year) ? 366 : 365)


union
{
	unsigned long currentEpochTime;
	uint8_t cept[4];
}ep,ep1;

unsigned long testEpochTime1;

uint8_t Raw_pressure_cnt_ind1=0;
signed short Raw_pressure_cnt1[RAW_DP_CNT_IND];

uint8_t Raw_pressure_cnt_ind2=0;
signed short Raw_pressure_cnt2[RAW_DP_CNT_IND];

float Dpressure1=0.0,Dpressure2=0.0;
float LastDpressure1=0.0,LastDpressure2=0.0;

uint8_t FirstTimeCheck=0;	
uint8_t progTimeout=0;

uint8_t prog_cnt=0;
uint8_t mode=NORMAL_MODE;
	
uint8_t keybyte=0;
uint8_t debounce=DEBOUNCE;

float tempfloat=0.0,tempfloat1=0.0,tempfloat2=0.0,f32_dp_sw_factor[5]={0},f32_dp_limit[5]={0};
short tempshort=0;
uint8_t tempchar=0;
uint8_t a1=0,a2=0,a3=0;
uint8_t b1=0,b2=0,b3=0;
uint8_t c1=0,c2=0,c3=0;
unsigned short us1=0,us2=0,us3=0;
signed short ss1=0,ss2=0,ss3=0;
unsigned long ul1=0,ul2=0,ul3=0;
//signed long sl1=0,sl2=0,sl3=0;

uint8_t Buffer1[100]={0};
uint8_t RAMBuffer[RAM_BUF_SIZE]={0};
uint8_t RxBuffer1[RX_IND_MAX]={0};
uint8_t RxBuffer[RX_IND_MAX]={0};
uint8_t TxBuffer[TX_IND_MAX]={0};
uint8_t XbeeRxBuffer[XBEE_RX_IND_MAX]={0};
	
volatile uint8_t RxInd=0,XbeeRxInd=0,RxTimeout=0;
volatile uint8_t crcVal=0;

unsigned short CustPassword=0,FactCustPassword=0;

uint8_t PCCalibrationTimer=0,gu8_doorSensingTimer=0,gu8_Dp1AlarmSensingTimer=0,gu8_Dp2AlarmSensingTimer=0;
uint8_t gu8_Dp1AlarmSensingTime=0,gu8_Dp2AlarmSensingTime=0,gu8_doorSensingTime=0,gu8_doorSensingPolarity=0,gu8_LCDBrigthnessCnt=0;
uint8_t restoreFactoryCalibrationTimer=0,DPAutoCalModeTimer=0,MinMaxMeanModeTimer=0,MeanHrModeTimer=0,DPAutoCalTimer=0,ProgModeTimer=0,gu8_MinMaxClearTimer=0;

uint8_t AckPwdInd=0;
unsigned short AckTimer=0,AckPwd[NO_OF_ACKPWD];
uint8_t BatteryPercentageSample[3]={0};
uint8_t BatSampleInd=0;
unsigned short BatteryPercentage=0;
unsigned long StartBroadcastTimer=0,AlarmAckTimer=0;
unsigned short logTimer=0; 
//unsigned long logTimer=0; 
//uint8_t FlashlogTimer=60;
unsigned long CurrentLogInd = 0;
unsigned short CurrentLog24Ind = 0;
unsigned short CurrentLogIndReadLoc=0;
uint8_t CurrentLog24IndReadLoc=0;

unsigned short ADC_sample=0;

uint8_t DP_StartUpTimer=0,TMRH_StartUpTimer=0,gu8_restartTimer=0;

unsigned short RAMBufferInd=0,RAMBufferLog=0;
uint8_t FlashOVFByte=0;
uint8_t rxMode=0;

unsigned long LowEpoch=0,MidEpoch=0,MidEpoch1=0,HighEpoch=0,MidLogInd=0;
unsigned long InitLogInd=0,LastLogInd=0,StartEpoch=0,EndEpoch=0,StartEpochTime=0,EndEpochTime=0,TotalLog=0,StartLogInd=0,EndLogInd=0;

unsigned short logtransfer=0, gu16_XbeeRstInterval=0;
unsigned long gu32_triggerXbeeResetTimer = 0;
signed short su16_dp_sw_factor[5]={0};
unsigned short u16_dp_limit[3]={0};
volatile unsigned long templong=0;
volatile unsigned short flash24_StartInd=0,flash24_EndInd=0;
volatile unsigned short NoOf24Log=0;
volatile uint8_t DP1_Alrm_ON=0,DP2_Alrm_ON=0,TM_Alrm_ON=0,RH_Alrm_ON=0;
volatile uint8_t LastDP1_Alrm_ON=0,LastDP2_Alrm_ON=0,LastTM_Alrm_ON=0,LastRH_Alrm_ON=0;
volatile uint8_t last_sec=0,last_min=0,last_hr=0,current_sec=0,current_min=0,current_hr=0;
volatile uint8_t ParaScrollTime=0;
uint8_t rtcCorruptionCheckCnt=INIT_RTC_CORRUPTION_CHECK_CNT;
uint8_t gu8_BackLitOnOff=0;
uint8_t gu8_TM_RH_ScanTime=0;
uint8_t gu8_DP1_LEDBlinkForPara=0,gu8_DP2_LEDBlinkForPara=0,gu8_TM_LEDBlinkForPara=0,gu8_RH_LEDBlinkForPara=0;
uint8_t gu8_masterEnable=0,gu8_dp_sw_enb=0,gu8_rly_stat=0;
//volatile uint8_t RTCSetFlag=0;

uint8_t MinMaxMeanReadParaType=0;
uint8_t dispMinMaxMeanLogInd=0,dispLogInd=0;
uint8_t min_max_mean_page_disp_cnt=0,mean_hr_page_disp_cnt=0;

uint8_t MeanHourLogInd=0;
float HourDP1_Mean=0.0,HourDP2_Mean=0.0,HourTM_Mean=0.0,HourRH_Mean=0.0;
uint8_t HrDP1SampleInd=0,HrDP2SampleInd=0,HrTMSampleInd=0,HrRHSampleInd=0;
	
uint8_t DP1_UserCalDateInd=0,DP2_UserCalDateInd=0,TM_UserCalDateInd=0,RH_UserCalDateInd=0;	
signed short DP1_Cal_Value_F=0,DP2_Cal_Value_F=0,TM_Cal_Value_F=0,RH_Cal_Value_F=0;
signed short DP1_Cal_Value_C=0,DP2_Cal_Value_C=0,TM_Cal_Value_C=0,RH_Cal_Value_C=0;
float DP1_Cal_float_Value_F=0,DP2_Cal_float_Value_F=0,TM_Cal_float_Value_F=0,RH_Cal_float_Value_F=0;
float DP1_Cal_float_Value_C=0,DP2_Cal_float_Value_C=0,TM_Cal_float_Value_C=0,RH_Cal_float_Value_C=0;
float RealDpressure1=0.0,RealDpressure2=0.0,RealtemperatureC=0.0,RealtemperatureF=0.0,RealhumidityRH=0.0;

uint8_t glbSrcPort=0;

uint8_t comport=0;
uint8_t gu8ar_SrNumber[16]={0};
uint8_t gu8arr_XbeeMac[NO_OF_XBEE_MAC][XBEE_MAC_SIZE]={0};
uint8_t gu8arr_XbeeSelfMac[XBEE_MAC_SIZE]={'0'};

uint8_t *p_SrNumber;
unsigned long gu32_SrNumber;
uint8_t gu8_AutoSentTimeout=60,gu8_AutoSentInterval=DEFAULT_AUTO_SENT_INTERVAL,gu8_DeviceInGroup=DEFAULT_DEVICES_IN_GROUP,gu8_AutoSentTimer=0,gu8_groupID=0,gu8_broadcast=0,gu8_Mac2ValidTimer = 0;

uint8_t gu8_deviceIDChangeTryTimer=0, gu8_deviceIDChangeTry=0;

// Kalman filter structure
typedef struct {
	float Q1;       // Process noise covariance
	float R1;       // Measurement noise covariance
	float X1;       // State estimate
	float P1;       // Estimate covariance
	float K1;       // Kalman gain
} KalmanFilter;
static KalmanFilter Kalman[4] = {0};	

//************************************************************************************************************************************
//													FUNCTION PROTOTYPES
//************************************************************************************************************************************
void InitSystemClock(void);
void Init_InternalRTC(void);
void Init_GPIO(void);
void Init_Timer0(void);
void InitADC(void);
void Init_DMA(void);
void ReadADCForBatteryVoltage(void);
void ServePCMsg(uint8_t SrcPort);

void delay(unsigned int);
void Init_variables(void);

void AllSegment(uint8_t);
void Check_RTC(void);
void ReadDiffPressure1(void);
void ReadDiffPressure2(void);

void DisableUnusedModules(void);
void InitLEDController(void);
void disp_value(void);
void conv_value(void);
void chartostr(unsigned short,uint8_t *,uint8_t);
void convert_float(float,uint8_t*,uint8_t);
void convert_char(unsigned short,uint8_t*,uint8_t);
void convert_long(unsigned long,uint8_t*,uint8_t);
//-----------------------------------------------------------------------------------------------------------------------------------------
void check_key(void);
void keyboard(void);
void CheckUpDnKey(void);
//-----------------------------------------------------------------------------------------------------------------------------------------
void boot_data(void);
void SecondTick(void);
void TMUnitChange(void);
void StartBuzzer(void);
void StopBuzzer(void);
void AutoSendDataResponse(uint8_t SrcPort);
//----------------------------------------------------------------------------------------------------------------------------
void Kalman_Init(KalmanFilter *kf, float qr, float rs, float initial_estimate);
float Kalman_Update(KalmanFilter *kf, float measurement);
//----------------------------------------------------------------------------------------------------------------------------
static inline int leapyear (long int year);
unsigned long  ydhms_diff (long int year1, long int yday1, int hour1, int min1, int sec1,int year0, int yday0, int hour0, int min0, int sec0);
unsigned long get_epoch_time(struct RTCData);
void get_date_time(struct RTCData* t1,unsigned long epoch);
//-------------------RS485 ROUTINE-------------------------------------------------------------------------------
void InitUSART(USART_t * usart, PORT_t * port, uint8_t rxpin, uint8_t txpin, uint8_t ubrr, uint8_t noOfbits, uint8_t parity, uint8_t stopbit);
//void Init_USARTC0(unsigned short Baudrate,uint8_t Databits,uint8_t Parity,uint8_t Stopbit);
//void Init_USARTC1(unsigned short Baudrate,uint8_t Databits,uint8_t Parity,uint8_t Stopbit);
void opstr(uint8_t portNo, char *str);
void opchar(uint8_t portNo,uint8_t str);
void print_float(uint8_t portNo,float,uint8_t*,uint8_t);
void print_short(uint8_t portNo,long,uint8_t*,uint8_t);
void print_Hex(uint8_t portNo,uint8_t);
void SendToUART(uint8_t port,uint8_t *str,unsigned short NoOfBytes);
short findValue(uint8_t *ptr,uint8_t NoOfDigit);
uint8_t fillValue(uint8_t *ptr,long value);
uint8_t CalCRC(uint8_t *ptr,unsigned short NoOfByte);
uint8_t find_Checksum(unsigned short Count,uint8_t *msg);
void FillRamBuffer(uint8_t,uint8_t,unsigned short);
void SetTxmode(uint8_t txmode,uint8_t *buffer,unsigned short bytes);
void SendToSlave(void);
uint32_t ascii2hex(uint8_t *data, uint8_t NoOfdigit);
void SetMAC2Xbee(uint8_t *mac,uint8_t ReadSelfMac);
uint8_t changeNibbles(uint8_t byte);
//****************************************************************************************************************************************

// Initialize the Kalman filter
void Kalman_Init(KalmanFilter *kf, float qr, float rs, float initial_estimate) 
{
	kf->Q1 = qr;          // Process noise covariance
	kf->R1 = rs;          // Measurement noise covariance
	kf->X1 = initial_estimate;  // Initial estimate
	kf->P1 = 1.0;        // Initial estimate covariance
}

// Update the Kalman filter with a new measurement
float Kalman_Update(KalmanFilter *kf, float measurement) 
{
	// Prediction update
	kf->P1 += kf->Q1;

	// Measurement update
	kf->K1 = kf->P1 / (kf->P1 + kf->R1);
	kf->X1 += kf->K1 * (measurement - kf->X1);
	kf->P1 *= (1 - kf->K1);

	return kf->X1;
}


/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	InitSystemClock();	
	DisableUnusedModules();
	Init_GPIO();
	
	//wdt_enable(WDTO_8S);
	
	//-------------------------------------------------------
	//Initialize Timer0
	//-------------------------------------------------------
	//#ifndef BOARD_5_13
	//Init_Timer0();
	//#endif
	
	BUZZER_ON;
	XBEE_RST_LOW;
	//-------------------------------------------------------
	//Boot Data from Internal EEPROM
	//-------------------------------------------------------
	boot_data();
	
	//-------------------------------------------------------
	//Initialize Variables
	//-------------------------------------------------------
	Init_variables();
	
	//-------------------------------------------------------
	//Initialize I2C for RTC, SHT25
	//-------------------------------------------------------
	I2C_Init();	
	//Init_DS1307();
	
	//-------------------------------------------------------
	//Initialize I2C2 for DP2/7SEG DISPLAY
	//-------------------------------------------------------
	I2C2_Init();
	
	//-------------------------------------------------------
	//Initialize I2C4 for DP1
	//-------------------------------------------------------
	I2C4_Init();
	
	//-------------------------------------------------------
	//Initialize Internal LCD Module
	//-------------------------------------------------------
	if(gu16_parameterWord & ENABLE_LCD)
	{
		InitLEDController();
	}
	
	//-------------------------------------------------------
	//Initialize USART
	//-------------------------------------------------------
	//Init_USARTC0(UART_BaudRate,UART_DataBits,UART_Parity,UART_StopBit);
	//Init_USARTC1(BAUD_57600,DATABIT_8,PARITY_NONE,STOP_BIT_1);
	InitUSART(&USARTC0, &PORTC, PIN2_bm, PIN3_bm, UART_BaudRate,UART_DataBits,UART_Parity,UART_StopBit);
	//InitUSART(&USARTC1, &PORTC, PIN6_bm, PIN7_bm, BAUD_57600, DATABIT_8, PARITY_NONE, STOP_BIT_1);
	//-------------------------------------------------------
	//POWER ON LOG
	//-------------------------------------------------------
	us1=CurrentLog24Ind;
	FillRamBuffer(POWER_UP_LOG,0,0xFFFF);
	
	//-------------------------------------------------------
	//Initialize RTC
	//-------------------------------------------------------
	Init_InternalRTC();
	
	//-------------------------------------------------------
	//Initialize DMA for UART Data transfer
	//-------------------------------------------------------
	Init_DMA();
	
	//-------------------------------------------------------
	//Initialize ADC
	//-------------------------------------------------------
	#ifdef ENABLE_BATTERY_DISPLAY
	InitADC();
	#endif
	
	#ifdef ENABLE_PRINTF
		
		// --- Reset sensor by command ---
		error |= SHT2x_SoftReset();

		// --- Read the sensors serial number (64bit) ---
		error |= SHT2x_GetSerialNumber(SerialNumber_SHT2x);
		
		// --- Set Resolution e.g. RH 10bit, Temp 13bit ---
		error |= SHT2x_ReadUserRegister(&userRegister);  //get actual user reg
		userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
		error |= SHT2x_WriteUserRegister(&userRegister); //write changed user reg
		
		opstr(0,"\r\n//*****************************//");
		opstr(0,"\r\n// Shree Aerodynamic Products  //");
		opstr(0,"\r\n//*****************************//");
		
		/*memset(&Buffer1,'0',sizeof(Buffer1));
		Buffer1[12]=0;
		
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)DP1_CAL_DATE_ADDR,6);
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)DP1_CAL_CERT_ADDR,15);
		opstr(0,"\r\nDP1 Cal Date :");
		opstr(0,(char*)&Buffer1[0]);		
		opstr(0,"\r\nDP1 Cal Certi :");
		opstr(0,(char*)&Buffer1[0]);
		
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)DP2_CAL_DATE_ADDR,12);
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)DP2_CAL_CERT_ADDR,15);
		opstr(0,"\r\nDP2 Cal Date :");
		opstr(0,(char*)&Buffer1[0]);
		opstr(0,"\r\nDP2 Cal Certi :");
		opstr(0,(char*)&Buffer1[0]);
		
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)TM_CAL_DATE_ADDR,12);
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)TM_CAL_CERT_ADDR,15);
		opstr(0,"\r\nTM Cal Date :");
		opstr(0,(char*)&Buffer1[0]);
		opstr(0,"\r\nTM Cal Certi :");
		opstr(0,(char*)&Buffer1[0]);
		
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)RH_CAL_DATE_ADDR,12);
		eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)RH_CAL_CERT_ADDR,15);
		opstr(0,"\r\nRH Cal Date :");
		opstr(0,(char*)&Buffer1[0]);
		opstr(0,"\r\nRH Cal Certi :");
		opstr(0,(char*)&Buffer1[0]);
		*/
		
		opstr(0,"\r\nSHT25 Sr.No. :");
		for(uint8_t i=0;i<8;i++)print_Hex(0,SerialNumber_SHT2x[i]);
		
		opstr(0,"\r\nCustomer password:");
		print_float(0,CustPassword,test,0);
		
		opstr(0,"\r\nFactory Customer password:");
		print_float(0,FactCustPassword,test,0);
		
		opstr(0,"\r\nAck parameter:");
		opstr(0,"\r\nAck Timer:");
		print_float(0,AckTimer,test,0);
		
		opstr(0,"\r\nAck password Counter:");
		print_float(0,AckPwdInd,test,0);
		
		opstr(0,"\r\n");
		
		for(uint8_t i=0;i<NO_OF_ACKPWD;i++)
		{
			print_float(0,AckPwd[i],test,0);
			opstr(0,"\r\n");
		}
		
		opstr(0,"\r\nDP1 Min  :");	print_float(0,DP1_Min,test,1);		opstr(0,"  Max:");	print_float(0,DP1_Max,test,1);	
		opstr(0,"\r\nDP2 Min  :");	print_float(0,DP2_Min,test,1);		opstr(0,"  Max:");	print_float(0,DP2_Max,test,1);	
		opstr(0,"\r\nTM Min   :");	print_float(0,TM_Min,test,1);		opstr(0,"  Max:");	print_float(0,TM_Max,test,1);	
		opstr(0,"\r\nRH Min   :");	print_float(0,RH_Min,test,1);		opstr(0,"  Max:");	print_float(0,RH_Max,test,1);
		
		opstr(0,"\r\nLog Interval :");
		print_float(0,LogInterval,test,0);		
		
		opstr(0,"\r\n       L_ON     L_OFF     U_OFF    U_ON");
		opstr(0,"\r\nDP1   :");
		
		print_float(0,(float)DP1_Lower_Alm_ON/10,test,1);	opstr(0,"      ");
		print_float(0,(float)DP1_Lower_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)DP1_Upper_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)DP1_Upper_Alm_ON/10,test,1);
		
		opstr(0,"\r\nDP2   :");
		
		print_float(0,(float)DP2_Lower_Alm_ON/10,test,1);	opstr(0,"      ");
		print_float(0,(float)DP2_Lower_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)DP2_Upper_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)DP2_Upper_Alm_ON/10,test,1);
		
		opstr(0,"\r\nTM    :");
		
		print_float(0,(float)TM_Lower_Alm_ON/10,test,1);	opstr(0,"      ");
		print_float(0,(float)TM_Lower_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)TM_Upper_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)TM_Upper_Alm_ON/10,test,1);
		
		opstr(0,"\r\nRH    :");
		
		print_float(0,(float)RH_Lower_Alm_ON/10,test,1);	opstr(0,"      ");
		print_float(0,(float)RH_Lower_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)RH_Upper_Alm_OFF/10,test,1);	opstr(0,"      ");
		print_float(0,(float)RH_Upper_Alm_ON/10,test,1);
		
		opstr(0,"\r\nFactory Cal Value  :");
		print_float(0,DP1_Cal_Value_F,test,0);		opstr(0,"         ");
		print_float(0,DP2_Cal_Value_F,test,0);		opstr(0,"         ");
		print_float(0,TM_Cal_Value_F,test,0);		opstr(0,"         ");
		print_float(0,RH_Cal_Value_F,test,0);		opstr(0,"         ");
		
		opstr(0,"\r\nCustomer Cal Value :");
		print_float(0,DP1_Cal_Value_C,test,0);		opstr(0,"         ");
		print_float(0,DP2_Cal_Value_C,test,0);		opstr(0,"         ");
		print_float(0,TM_Cal_Value_C,test,0);		opstr(0,"         ");
		print_float(0,RH_Cal_Value_C,test,0);		opstr(0,"         ");
		
		opstr(0,"\r\nTM Unit:");
		print_float(0,TM_Unit,test,0);	
		
		opstr(0,"\r\nDevice ID:");
		print_float(0,DeviceID,test,0);		
		
		opstr(0,"\r\nBUZZER ON Time:");
		print_float(0,Buzzer_ON_Time,test,0);		
		opstr(0,"   OFF Time:");
		print_float(0,Buzzer_OFF_Time,test,0);	
		
		opstr(0,"\r\nUART:");
		print_float(0,UART_BaudRate,test,0);		opstr(0,"  ");
		print_float(0,UART_DataBits,test,0);		opstr(0,"  ");
		print_float(0,UART_Parity,test,0);			opstr(0,"  ");
		print_float(0,UART_StopBit,test,0);			opstr(0,"  ");
		
		opstr(0,"\r\nUser Log Index:");
		print_float(0,CurrentLogInd,test,0);		opstr(0,"  ");
		print_float(0,FlashOVFByte,test,0);			opstr(0,"  ");
		
		opstr(0,"\r\n24Hr Log Index:");
		print_float(0,CurrentLog24Ind,test,0);		opstr(0,"  ");
		
		opstr(0,"\r\n");
		
	#endif
	
	logTimer = LogInterval;
	b.brodcastEnb=0;	
	logtransfer=0;
	b.logtransferStart=0;
	DP_StartUpTimer=S_STABLE_TIME;
	TMRH_StartUpTimer=S_STABLE_TIME;
	b.resetDevice=0;
	
	RS485_TX0_DIS;
	RS485_TX1_DIS;
	BUZZER_OFF;
	XBEE_RST_HIGH;
	
	// Start USB stack to authorize VBus monitoring
	//udc_start();
	
	// Turn Interrupts on.
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();			//Global Interrupt Enable
	
	//WHITE_BLIT_ON;
	
	#ifndef DISABLE_DOOR_SENSING

	if((!DOOR_SENSE && gu8_doorSensingPolarity) || (DOOR_SENSE && !gu8_doorSensingPolarity))
	{
		b.doorStatus=OPEN;
		gu8_dp_sw_enb = 1;
	}
	else
	{
		b.doorStatus=CLOSE;
		gu8_dp_sw_enb = 0;
	}
		
	#endif

	SetMAC2Xbee(&gu8arr_XbeeMac[0][0],1);
	
	Kalman_Init(&Kalman[0], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalman[1], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalman[2], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalman[3], 0.01, 0.1, 0.0);  // Initialize with default values

 	while(1)
 	{	
		if(b.resetDevice) 	
		{
			while(1);
		}	
						
		//Serve Watchdog Timer
		wdt_reset();
		
		if(b.autoSendResponse)
		{
			AutoSendDataResponse(1);
			b.autoSendResponse = false;
		}
		
		if(b.triggerXbeeReset)
		{
			_delay_ms(25);
			XBEE_RST_LOW;
			_delay_ms(1);
			XBEE_RST_HIGH;
			_delay_ms(25);
			
			b.triggerXbeeReset=false;
		}
		
		//1 Second Tick ====================================================
		if(b.sec_flag)
		{
			SecondTick();

			//====================================================	
			if(gu16_parameterWord & ENABLE_RTC)
			{
				Check_RTC();	//Check RTC
			}
			//====================================================
			if(AlarmAckTimer)
			{
				StopBuzzer();
			}
			//====================================================
			/*if((!DP1_Alrm_ON) && (!DP2_Alrm_ON) && (!TM_Alrm_ON) && (!RH_Alrm_ON) && (b.doorStatus==CLOSE))
			{
				if(b.alarmAutorestore)
				{
					AlarmAckTimer=0;
					
					StopBuzzer();
					
					b.alarmAutorestore=0;
				}
			}
			else
			{
				b.alarmAutorestore=1;
			}*/
			
			if((DP1_Alrm_ON!=NO_ALARM)||(DP2_Alrm_ON!=NO_ALARM)||(TM_Alrm_ON!=NO_ALARM)||(RH_Alrm_ON!=NO_ALARM)||(b.doorStatus==OPEN))
			{	
				if(b.buzzeralert==0)
				{	
					gu8_doorSensingTimer++;
					if(gu8_doorSensingTimer>=gu8_doorSensingTime)
					{
						gu8_doorSensingTimer=0;
						if(gu16_parameterWord & ENABLE_ALERT) StartBuzzer();
						b.buzzeralert=1;
					}	
				}
			}
			else
			{
				if(b.buzzeralert==1)
				{
					StopBuzzer();
					gu8_doorSensingTimer=0;
					b.buzzeralert=0;
				}	
			}	
			
			b.sec_flag=0;
		}
		
		// Check for any RS485 Command ================================================
		if(b.msgRcvOK)
		{
			ServePCMsg(comport);
		}
		else
		{
			if(b.RamReadCmd)
			{
				if(NoOf24Log && !b.logtransferStart)
				{
					cli();
					TxBuffer[0]=0xFD;
					TxBuffer[1]=RxBuffer[1];
					TxBuffer[2]=RxBuffer[2];
					TxBuffer[3]=0x00;
					if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
					if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
					if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
					if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
					TxBuffer[4]=RxBuffer[3];
					TxBuffer[5]=RxBuffer[4];

					tempshort = (flash24_StartInd * LOG_SIZE) + RAM_FILL_START;
					memcpy(&TxBuffer[6],&RAMBuffer[tempshort],LOG_SIZE);
					
					TxBuffer[69]=CalCRC(&TxBuffer[1],68);
					TxBuffer[70]=0xFC;

					if(flash24_StartInd)
					{
						flash24_StartInd--;
					}
					else
					{
						flash24_StartInd=29;
					}
					
					if(!glbSrcPort)
					{
						b.logtransferStart=1;
					}
					else if(glbSrcPort==1)
					{
						b.logtransferStart=1;
					}
					else
					{
						b.logtransferStart=0;
					}
					
					SetTxmode(glbSrcPort,TxBuffer,71);
					sei();

					/*logtransfer++;
					if(logtransfer>flash24_EndInd)
					{
						b.RamReadCmd=0;
						glbSrcPort=0;
					}
					*/
					NoOf24Log--;
					if(!NoOf24Log)
					{
						b.RamReadCmd=0;
						glbSrcPort=0;
					}
				}
			}
			else if(b.RamAllReadCmd)
			{
				cli();
				SendToUART(glbSrcPort,&RAMBuffer[0],1507);
				
				b.RamAllReadCmd=0;
				glbSrcPort=0;
				sei();
			}
		}
		
		//====================================================
		#ifdef ENABLE_KEY_LOGIC
			check_key();		//Check Keyboard
			if(b.msec_flag)
			{
				CheckUpDnKey();		//Check UP and Down key
			}
		#endif
		
		if(b.msec250_flag)
		{
			Read_SHT25();
			
			b.msec250_flag=0;
		}
		
		if(b.msec50_flag)
		{
			//Read Differential Pressure -----------------------------------------
			if(gu16_parameterWord & ENABLE_DP1)
			{
				ReadDiffPressure1();
			}
			else
			{
				b.DP1_NC = 0;
				
				Dpressure1=0.0;

				DP1_Max=0.0;
				DP1_Min=0.0;

				DP1_Alrm_ON=NO_ALARM;
				
				DP_StartUpTimer=0;
			}
	
			if(gu16_parameterWord & ENABLE_DP2)
			{
				ReadDiffPressure2();
			}
			else
			{
				b.DP2_NC = 0;
				
				Dpressure2=0.0;

				DP2_Max=0.0;
				DP2_Min=0.0;

				DP2_Alrm_ON=NO_ALARM;
				
				DP_StartUpTimer=0;
			}
			
			b.msec50_flag=0;
		}
		//====================================================
		if(gu16_parameterWord & ENABLE_LCD)
		{		
			conv_value();
			disp_value();	
		}
 	}//END OF WHILE LOOP
}

void SetMAC2Xbee(uint8_t *mac,uint8_t ReadSelfMac)
{
	uint8_t buffer[5]={0};
	
	_delay_ms(1000);
	//-------------------------
	opstr(0,"+++");
	_delay_ms(1000);
	//-------------------------
	opstr(0,"ATDH");
	SendToUART(0,&mac[0],8);
	opchar(0,'\r');
	_delay_ms(50);
	//-------------------------
	opstr(0,"ATDL");
	SendToUART(0,&mac[8],8);
	opchar(0,'\r');
	_delay_ms(50);
	//-------------------------
	if(ReadSelfMac==1)
	{
		memset(gu8arr_XbeeSelfMac,'0',XBEE_MAC_SIZE);
		memset(XbeeRxBuffer,0,XBEE_RX_IND_MAX);
		XbeeRxInd=0;
		opstr(0,"ATSH?\r");
		_delay_ms(50);
		memcpy(&gu8arr_XbeeSelfMac[2],&XbeeRxBuffer[0],6);
		//-------------------------
		memset(XbeeRxBuffer,0,XBEE_RX_IND_MAX);
		XbeeRxInd=0;
		opstr(0,"ATSL?\r");
		_delay_ms(50);
		memcpy(&gu8arr_XbeeSelfMac[8],&XbeeRxBuffer[0],8);
		//-------------------------
		chartostr(DeviceID,&buffer[0],3);
		opstr(0,"ATBISAP");
		SendToUART(0,&buffer[0],3);
		opchar(0,'-');
		SendToUART(0,&gu8ar_SrNumber[8],8);
		opchar(0,'\r');
		_delay_ms(50);
	}
	//-------------------------
	opstr(0,"ATWR\r");
	_delay_ms(50);
	//-------------------------
	opstr(0,"ATCN\r");
	_delay_ms(50);
	//-------------------------
	
	//SendToUART(1,gu8arr_XbeeSelfMac,XBEE_MAC_SIZE);
}
	
void check_key(void)
{
	keybyte=0;

	if(!PARA_SELECT_KEY) keybyte |= BIT1;

	if((!keybyte)&&(!b.keybit))
	{
		if(debounce)
		{
			debounce--;
			if(!debounce)
			{
				b.keybit=1;
				debounce=DEBOUNCE;
			}
		}
	}	
	else if((keybyte)&&(b.keybit))
	{
		if(debounce)
		{
			debounce--;
			if(!debounce)
			{
				b.keybit=0;
				keyboard();
				debounce=DEBOUNCE;
			}
		}
	}
	else debounce=DEBOUNCE;

	if(UP_KEY) key_up_count=0;
	if(DN_KEY) key_dn_count=0;
}

void CheckUpDnKey(void)
{
	uint8_t i=0;
	
	if(!UP_KEY && !DN_KEY)
	{
		if(mode==DP_AUTO_CAL_MODE)
		{
			progTimeout=60;
			
			DPAutoCalTimer++;
			if(DPAutoCalTimer > 10)
			{
				DPAutoCalTimer=0;
				
				switch(autoCal_para_cnt)
				{
					case 0:
						
						DP1_Cal_Value_C = (RealDpressure1 - DP1_Cal_float_Value_F)*10.0;
						//DP1_Cal_Value_C = RealDpressure1*10.0;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
						DP1_Cal_float_Value_C = (float)DP1_Cal_Value_C/10.0;
						
						/*AutocalCnt = (signed short)(Dpressure1 * 10.0);
						DP1_Cal_Count_C -= AutocalCnt;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT_C,DP1_Cal_Count_C);
						*/
						
					break;
				
					case 1:
				
						DP2_Cal_Value_C = (RealDpressure2 - DP2_Cal_float_Value_F)*10.0;
						//DP2_Cal_Value_C = RealDpressure2*10.0;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
						DP2_Cal_float_Value_C = (float)DP2_Cal_Value_C/10.0;
						
						/*AutocalCnt = (signed short)(Dpressure2 * 10.0);
						DP2_Cal_Count_C -= AutocalCnt;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT_C,DP2_Cal_Count_C);
						*/
						
					break;
				}
				
				for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
				data[4] = D;
				data[5] = 0;
				data[6] = N;
				disp_value();
				
				_delay_ms(4000);
				
				mode=NORMAL_MODE;
				Normal_para_cnt=0;
				autoCal_para_cnt=0;
				progTimeout=0;
			}
		}
		else
		{
			restoreFactoryCalibrationTimer++;
			if(restoreFactoryCalibrationTimer > 20)
			{
				restoreFactoryCalibrationTimer=0;
			
				DP1_Cal_Value_C=0;
				DP1_Cal_float_Value_C = 0.0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
				
				DP2_Cal_Value_C=0;
				DP2_Cal_float_Value_C = 0.0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
				
				TM_Cal_Value_C=0;
				TM_Cal_float_Value_C = 0.0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
				
				RH_Cal_Value_C=0;
				RH_Cal_float_Value_C = 0.0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
				
				//DP1_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT_C,DP1_Cal_Count_C);
			
				//DP2_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT_C,DP2_Cal_Count_C);
			
				//TM_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT_C,TM_Cal_Count_C);
			
				//RH_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT_C,RH_Cal_Count_C);	
				Normal_para_cnt=0;
				progTimeout=0;

				for(uint8_t i=0;i<NO_DIGIT;i++) data[i]=BLANK;
				data[1] = F;
				data[2] = A;
				data[3] = C;
				
				data[4] = C;
				data[5] = A;
				data[6] = L;
				disp_value();
				
				_delay_ms(4000);
				
			}
		}
	}
	else if(!PROG_ENT_KEY)
	{
		ProgModeTimer++;
		if(ProgModeTimer > 10)
		{
			ProgModeTimer=0;
			
			if(mode==NORMAL_MODE)
			{
				mode=PROG_MODE;
				prog_para_cnt=0;
				
				Normal_para_cnt=0;
				b.RTCChangeOccure=0;
				b.UARTChanged=0;
				Lastpara_cnt=0;
				progTimeout=60;
				b.SetACKPwd=0;
			}
			else if(mode==PROG_MODE)
			{
				mode=NORMAL_MODE;
				progTimeout=0;
				
				for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
				data[4] = N;
				data[5] = 0;
				data[6] = r;
				disp_value();
				
				_delay_ms(4000);
			}
		}
	}
	else if(!UP_KEY)
	{
		if(key_up_count<20)key_up_count++;
	
		if(key_up_count<10)			dummy++;
		else if(key_up_count<20)	dummy+=10;
		else						dummy+=100;
	
		//print_short(1,key_up_count,test,3);		opstr(1,"\r\n");
		
		switch(mode)
		{
			case NORMAL_MODE: 
			
				progTimeout=60;
			
				if(!b.SetACKPwd)
				{
					Normal_para_cnt++;

					if(Normal_para_cnt==5)
					{
						if((DP1_Alrm_ON) || (DP2_Alrm_ON) || (TM_Alrm_ON) || (RH_Alrm_ON))
						{
							dummy1=0;
							dummy=0;
						}
						else
						{
							Normal_para_cnt=0;
							progTimeout=0;
							b.SetACKPwd=0;
						}
					}
					
					if(Normal_para_cnt>5)
					{
						Normal_para_cnt=0;
						progTimeout=0;
						b.SetACKPwd=0;
					}
				}
				else if(b.SetACKPwd==1)
				{
					//b.SetACKPwd=1;
					if(dummy>15)dummy=15;
					dummy1=dummy;
				}
				else
				{
					if(dummy>999)dummy=999;
				}
			
			break;
			
			case DP_AUTO_CAL_MODE:
			
				progTimeout=60;
				autoCal_para_cnt=1;
				
			break;
			
			case PROG_MODE:
			
				if(!PARA_SELECT_KEY)
				{
					//opstr(0,"\r\nProg Mode + Up Key + Para_Select key pressed\r\n");
					prog_para_cnt++;
					
					if((prog_para_cnt==4) && !(gu16_parameterWord & ENABLE_DP2))
					{
						prog_para_cnt=8;
					}
					
					if((prog_para_cnt==8) && !(gu16_parameterWord & ENABLE_TEMP))
					{
						prog_para_cnt=13;
					}
					
					if((prog_para_cnt==13) && !(gu16_parameterWord & ENABLE_RH))
					{
						prog_para_cnt=17;
					}
					
					if((prog_para_cnt==29) && (!(gu16_parameterWord & ENABLE_TEMP) && !(gu16_parameterWord & ENABLE_RH)))
					{
						prog_para_cnt=1;
					}

					if(prog_para_cnt==30)
					{
						if((dummy == CustPassword) || (dummy == FactCustPassword))
						{
							prog_para_cnt=30;
							
							if((prog_para_cnt==30) && !(gu16_parameterWord & ENABLE_TEMP))
							{
								prog_para_cnt=31;
							}
							else
							{
								b.cal_mode=1;
							}
							
							if((prog_para_cnt==31) && !(gu16_parameterWord & ENABLE_RH))
							{
								prog_para_cnt=1;
							}
							else
							{
								b.cal_mode=1;
							}
						}
						else
						{
							prog_para_cnt=1;
						}
					}
					
					if(prog_para_cnt>31)
					{
						prog_para_cnt=1;
					}
					
					dummy=0;
					
					Lastpara_cnt=prog_para_cnt;
					//----------------------------------------------------------------------
					switch(prog_para_cnt)
					{
						case 1:		dummy = DeviceID;				break;
						case 2:		dummy = gu8_BackLitOnOff;		break;
						case 3:		dummy = gu8_TM_RH_ScanTime;		break;
						case 4:		dummy = DP2_Upper_Alm_ON; 		break;
						case 5:		dummy = DP2_Upper_Alm_OFF; 		break;
						case 6:		dummy = DP2_Lower_Alm_OFF;  	break;
						case 7:		dummy = DP2_Lower_Alm_ON;  		break;
						case 8:		dummy = TM_Upper_Alm_ON;  		break;
						case 9:		dummy = TM_Upper_Alm_OFF; 		break;
						case 10:	dummy = TM_Lower_Alm_OFF; 		break;
						case 11:	dummy = TM_Lower_Alm_ON;  		break;
						case 12:	dummy = TM_Unit;				break;
						case 13:	dummy = RH_Upper_Alm_ON;  		break;
						case 14:	dummy = RH_Upper_Alm_OFF;  		break;
						case 15:	dummy = RH_Lower_Alm_OFF;  		break;
						case 16:	dummy = RH_Lower_Alm_ON;  		break;
						case 17:
							Temp_RTC_ARR[0] = rtc.hour;
							dummy = rtc.hour;
						break;
						case 18:
							Temp_RTC_ARR[1] = rtc.minute;
							dummy = rtc.minute;
						break;
						case 19:
							Temp_RTC_ARR[2] = rtc.day;
							dummy = rtc.day;
						break;
						case 20:
							Temp_RTC_ARR[3] = rtc.month;
							dummy = rtc.month;
						break;
						case 21:
							Temp_RTC_ARR[4] = rtc.year;
							dummy = rtc.year;
						break;
						case 22:	dummy = Buzzer_ON_Time;	  		break;
						case 23:	dummy = Buzzer_OFF_Time;	  	break;
						case 24:	dummy = LogInterval;			break;
						case 25:	dummy = UART_BaudRate;		  	break;
						case 26:	dummy = UART_DataBits;			break;
						case 27:	dummy = UART_Parity;		 	break;
						case 28:	dummy = UART_StopBit;	 		break;
						case 29:	dummy = 0;						break;
						case 30:
						
							if(b.RH_TEMP_NC)
							{
								dummy = 0;
							}
							else
							{
								if(!TM_Unit)
								{
									dummy = (RealtemperatureC - TM_Cal_float_Value_F)*10.0;
								}
								else
								{
									dummy = (RealtemperatureF - TM_Cal_float_Value_F)*10.0;
								}
								dummy1 = dummy;
							}
						
						break;
						case 31:
							if(b.RH_TEMP_NC)
							{
								dummy = 0;
							}
							else
							{
								dummy = (RealhumidityRH - RH_Cal_float_Value_F)*10.0;
								dummy1 = dummy;
							}
						break;
					}
				}
				else
				{
					progTimeout=60;
					
					switch(prog_para_cnt)
					{
						case 1:		if(dummy>250)dummy=250;										break;
						case 2:		if(dummy>1)dummy=1;											break;
						case 3:		if(dummy>20)dummy=20;										break;
						case 4:		if(dummy>DEFAUT_DP2_MIN*10.0)dummy=DEFAUT_DP2_MIN*10.0;		break;
						case 5:		if(dummy>DP2_Upper_Alm_ON) dummy=(DP2_Upper_Alm_ON-1);		break;
						case 6:		if(dummy>DP2_Upper_Alm_OFF)dummy=(DP2_Upper_Alm_OFF-1);		break;
						case 7:		if(dummy>DP2_Lower_Alm_OFF)dummy=(DP2_Lower_Alm_OFF-1);		break;
						case 8:
							if(!TM_Unit)
							{
								if(dummy>DEFAUT_TEMP_C_MIN*10.0)dummy=DEFAUT_TEMP_C_MIN*10.0;
							}
							else
							{
								if(dummy>DEFAUT_TEMP_F_MIN*10.0)dummy=DEFAUT_TEMP_F_MIN*10.0;
							}
						break;
						case 9:		if(dummy>TM_Upper_Alm_ON)dummy=(TM_Upper_Alm_ON-1);		break;
						case 10:	if(dummy>TM_Upper_Alm_OFF)dummy=(TM_Upper_Alm_OFF-1);	break;
						case 11:	if(dummy>TM_Lower_Alm_OFF)dummy=(TM_Lower_Alm_OFF-1);	break;
						case 12:	dummy=1;												break;
						case 13:	if(dummy>DEFAUT_RH_MIN*10.0)dummy=DEFAUT_RH_MIN*10.0;	break;
						case 14:	if(dummy>RH_Upper_Alm_ON)dummy=(RH_Upper_Alm_ON-1);		break;
						case 15:	if(dummy>RH_Upper_Alm_OFF)dummy=(RH_Upper_Alm_OFF-1);	break;
						case 16:	if(dummy>RH_Lower_Alm_OFF)dummy=(RH_Lower_Alm_OFF-1);	break;
						case 17: 	if(dummy>23)dummy=23; 	b.RTCChangeOccure = 1;			break;
						case 18: 	if(dummy>59)dummy=59; 	b.RTCChangeOccure = 1;			break;
						case 19: 	if(dummy>31)dummy=31; 	b.RTCChangeOccure = 1;			break;
						case 20: 	if(dummy>12)dummy=12; 	b.RTCChangeOccure = 1;			break;
						case 21: 	if(dummy>99)dummy=99;	b.RTCChangeOccure = 1;			break;
						case 22:	if(dummy>60)dummy=60;									break;
						case 23:	if(dummy>960)dummy=960;									break;
						case 24:	if(dummy>MAX_LOG_INTERVAL)dummy=MAX_LOG_INTERVAL;		break;
						case 25:	if(dummy>9)dummy=9;		b.UARTChanged = 1;				break;
						case 26:	if(dummy>3)dummy=3;		b.UARTChanged = 1;				break;
						case 27:	if(dummy>2)dummy=2;		b.UARTChanged = 1;				break;
						case 28:	if(dummy>1)dummy=1;		b.UARTChanged = 1;				break;
						case 29:	if(dummy>999)dummy=999;									break;
						case 30:
							if(b.RH_TEMP_NC)
							{
								dummy=0;
							}
							else
							{
								if(!TM_Unit)
								{
									if(dummy>(DEFAUT_TEMP_C_MIN*10))dummy=(DEFAUT_TEMP_C_MIN*10);
								}
								else
								{
									if(dummy>(DEFAUT_TEMP_F_MIN*10))dummy=(DEFAUT_TEMP_F_MIN*10);
								}
							}
						break;
						case 31:
							if(b.RH_TEMP_NC)
							{
								dummy=0;
							}
							else
							{
								if(dummy>(DEFAUT_RH_MIN*10))dummy=(DEFAUT_RH_MIN*10);	 		
							}
						break;
					}
				}
			break;
		}
	}
	else if(!DN_KEY)
	{
		if(key_dn_count<20)key_dn_count++;
	
		if(key_dn_count<10)			dummy--;
		else if(key_dn_count<20)	dummy-=10;
		else						dummy-=100;
	
		switch(mode)
		{
			case NORMAL_MODE: 
			
				progTimeout=60;
				
				if(!b.SetACKPwd)
				{
					
				}
				else if(b.SetACKPwd==1)
				{
					if(dummy<1)dummy=1;
					dummy1=dummy;
				}
				else 
				{
					if(dummy<0)dummy=0;
				}
				
			break;
			
			case DP_AUTO_CAL_MODE:
			
				/*progTimeout=60;
				
				autoCal_para_cnt=1;
				
				*/
				
			break;
			
			case PROG_MODE:

				if(!PARA_SELECT_KEY)
				{
					//opstr(0,"\r\nProg Mode + Down Key + Para_Select key pressed\r\n");
					if(prog_para_cnt)prog_para_cnt--;
					
					if(!prog_para_cnt)prog_para_cnt=29;
					
					if(prog_para_cnt>29)
					{
						prog_para_cnt=29;
						b.cal_mode=0;
					}
					
					if((prog_para_cnt==16) && !(gu16_parameterWord & ENABLE_RH))
					{
						prog_para_cnt=12;
					}
					
					if((prog_para_cnt==12) && !(gu16_parameterWord & ENABLE_TEMP))
					{
						prog_para_cnt=7;
					}
					
					if((prog_para_cnt==7) && !(gu16_parameterWord & ENABLE_DP2))
					{
						prog_para_cnt=3;
					}
					
					dummy=0;
					
					Lastpara_cnt=prog_para_cnt;
					//----------------------------------------------------------------------
					switch(prog_para_cnt)
					{
						case 1:		dummy = DeviceID;				break;
						case 2:		dummy = gu8_BackLitOnOff;		break;
						case 3:		dummy = gu8_TM_RH_ScanTime;		break;
						case 4:		dummy = DP2_Upper_Alm_ON; 		break;
						case 5:		dummy = DP2_Upper_Alm_OFF; 		break;
						case 6:		dummy = DP2_Lower_Alm_OFF;  	break;
						case 7:		dummy = DP2_Lower_Alm_ON;  		break;
						case 8:		dummy = TM_Upper_Alm_ON;  		break;
						case 9:		dummy = TM_Upper_Alm_OFF; 		break;
						case 10:	dummy = TM_Lower_Alm_OFF; 		break;
						case 11:	dummy = TM_Lower_Alm_ON;  		break;
						case 12:	dummy = TM_Unit;				break;
						case 13:	dummy = RH_Upper_Alm_ON;  		break;
						case 14:	dummy = RH_Upper_Alm_OFF;  		break;
						case 15:	dummy = RH_Lower_Alm_OFF;  		break;
						case 16:	dummy = RH_Lower_Alm_ON;  		break;
						case 17:
							Temp_RTC_ARR[0] = rtc.hour;
							dummy = rtc.hour;
						break;
						case 18:
							Temp_RTC_ARR[1] = rtc.minute;
							dummy = rtc.minute;
						break;
						case 19:
							Temp_RTC_ARR[2] = rtc.day;
							dummy = rtc.day;
						break;
						case 20:
							Temp_RTC_ARR[3] = rtc.month;
							dummy = rtc.month;
						break;
						case 21:
							Temp_RTC_ARR[4] = rtc.year;
							dummy = rtc.year;
						break;
						case 22:	dummy = Buzzer_ON_Time;	  		break;
						case 23:	dummy = Buzzer_OFF_Time;	  	break;
						case 24:	dummy = LogInterval;			break;
						case 25:	dummy = UART_BaudRate;		  	break;
						case 26:	dummy = UART_DataBits;			break;
						case 27:	dummy = UART_Parity;		 	break;
						case 28:	dummy = UART_StopBit;	 		break;
						case 29:	dummy = 0;						break;
					}
				}
				else
				{
					progTimeout=60;
					
					switch(prog_para_cnt)
					{
						case 1:		if(dummy<1)dummy=1;										break;
						case 2:		if(dummy<0)dummy=0;										break;
						case 3:		if(dummy<5)dummy=5;										break;
						case 4:		if(dummy<DP2_Upper_Alm_OFF)dummy=(DP2_Upper_Alm_OFF+1);	break;
						case 5:		if(dummy<DP2_Lower_Alm_OFF)dummy=(DP2_Lower_Alm_OFF+1);	break;
						case 6:		if(dummy<DP2_Lower_Alm_ON)dummy=(DP2_Lower_Alm_ON+1);	break;
						case 7:		if(dummy<DEFAUT_DP2_MAX*10.0)dummy=DEFAUT_DP2_MAX*10.0;	break;
						case 8:		if(dummy<TM_Upper_Alm_OFF)dummy=(TM_Upper_Alm_OFF+1);	break;
						case 9:		if(dummy<TM_Lower_Alm_OFF)dummy=(TM_Lower_Alm_OFF+1);	break;
						case 10:	if(dummy<TM_Lower_Alm_ON)dummy=(TM_Lower_Alm_ON+1);		break;
						case 11:	if(dummy<DEFAUT_TEMP_C_MAX*10.0)dummy=DEFAUT_TEMP_C_MAX*10.0;		break;
						case 12:	dummy=0;												break;
						case 13:	if(dummy<RH_Upper_Alm_OFF)dummy=(RH_Upper_Alm_OFF+1);	break;
						case 14:	if(dummy<RH_Lower_Alm_OFF)dummy=(RH_Lower_Alm_OFF+1);	break;
						case 15:	if(dummy<RH_Lower_Alm_ON)dummy=(RH_Lower_Alm_ON+1);		break;
						case 16:	if(dummy<0)dummy=DEFAUT_RH_MAX*10.0;					break;
						case 17: 	if(dummy<0)dummy=0; 	b.RTCChangeOccure = 1;			break;
						case 18: 	if(dummy<0)dummy=0; 	b.RTCChangeOccure = 1;			break;
						case 19: 	if(dummy<1)dummy=1; 	b.RTCChangeOccure = 1;			break;
						case 20: 	if(dummy<1)dummy=1; 	b.RTCChangeOccure = 1;			break;
						case 21: 	if(dummy<0)dummy=0;		b.RTCChangeOccure = 1;			break;
						case 22:	if(dummy<0)dummy=0;										break;
						case 23:	if(dummy<0)dummy=0;										break;
						case 24:	if(dummy<MIN_LOG_INTERVAL)dummy=MIN_LOG_INTERVAL;		break;
						case 25:	if(dummy<3)dummy=3;		b.UARTChanged = 1;				break;
						case 26:
						case 27:
						case 28:	if(dummy<0)dummy=0;		b.UARTChanged = 1;				break;
						case 29:	if(dummy<0)dummy=0;										break;
						case 30:
							if(b.RH_TEMP_NC)
							{
								dummy=0;
							}
							else
							{
								if(!TM_Unit)
								{
									if(dummy<(DEFAUT_TEMP_C_MAX*10))dummy=(DEFAUT_TEMP_C_MAX*10);
								}
								else
								{
									if(dummy<(DEFAUT_TEMP_F_MAX*10))dummy=(DEFAUT_TEMP_F_MAX*10);
								}
							}
						break;
						case 31:
							if(b.RH_TEMP_NC)
							{
								dummy=0;
							}
							else
							{
								if(dummy<(DEFAUT_RH_MAX*10))dummy=(DEFAUT_RH_MAX*10);
							}
						break;
					}
				}
				
			break;
		}
	}
	else if(!PARA_SELECT_KEY)
	{
		if(mode==NORMAL_MODE)
		{
			DPAutoCalModeTimer++;
			if(DPAutoCalModeTimer > 20)
			{
				DPAutoCalModeTimer=0;
				
				mode=DP_AUTO_CAL_MODE;
				
				progTimeout=60;
			}
		}
	}
	else
	{
		restoreFactoryCalibrationTimer=0;
		DPAutoCalModeTimer=0;
		DPAutoCalTimer=0;
		ProgModeTimer=0;
		MinMaxMeanModeTimer=0;
		MeanHrModeTimer=0;
		gu8_MinMaxClearTimer=0;
	}
}
//**********************************************************************************************************************************************/
void keyboard(void)
{	
	switch(keybyte)
	{
		/*case PROG_ENT:
		
			for(uint8_t i=0;i<NO_DIGIT;i++) data[i]=BLANK;
			
			if(mode==NORMAL_MODE)
			{
				mode=PROG_MODE;
				prog_para_cnt=0;
				Normal_para_cnt=0;
				b.RTCChangeOccure=0;
				b.UARTChanged=0;
				Lastpara_cnt=0;
				progTimeout=60;
				b.SetACKPwd=0;
			}
			else
			{
				mode=NORMAL_MODE;
				progTimeout=0;
			}
		
		break;
		*/
		
		case PARA_SELECT:
		
			switch(mode)
			{
				case NORMAL_MODE:
				
					if(Normal_para_cnt==5)
					{
						if(!b.SetACKPwd)
						{
							if((DP1_Alrm_ON) || (DP2_Alrm_ON) || (TM_Alrm_ON) || (RH_Alrm_ON))
							{
								b.SetACKPwd=1;
								dummy1=0;
							}
						}
						else if(b.SetACKPwd==1)
						{
							if((DP1_Alrm_ON) || (DP2_Alrm_ON) || (TM_Alrm_ON) || (RH_Alrm_ON))
							{
								b.SetACKPwd=2;
								dummy=0;
							}
						}
						else
						{
							if(AckPwdInd)
							{
								//for(uint8_t i=0;i<NO_OF_ACKPWD;i++)
								{
									if(AckPwd[dummy1-1]==dummy)
									{
										AlarmAckTimer=(unsigned long)AckTimer * 60;										
										FillRamBuffer(ALM_ACK_LOG,dummy1,AckPwd[dummy1-1]);
										
										dummy1=0;
									}
								}
							}
							else
							{
								if(dummy==FACT_ACK_PWD)
								{
									AlarmAckTimer=(unsigned long)AckTimer * 60;
									FillRamBuffer(ALM_ACK_LOG,0,FACT_ACK_PWD);
								}
							}
							
							Normal_para_cnt=0;
							b.SetACKPwd=0;
						}
					}
					
				break;
				
				case PROG_MODE: 
				
					progTimeout=60;
				
					cli();			//Global Interrupt Disable
					
					switch(Lastpara_cnt)
					{
						case 1:
							if(DeviceID != dummy)
							{
								DeviceID = dummy;
								gu8_groupID = ((DeviceID - 1)/gu8_DeviceInGroup)+1;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICE_ID,DeviceID);
							}
						break;
						case 2:
							if(gu8_BackLitOnOff != dummy)
							{
								gu8_BackLitOnOff = dummy;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BACKLIT_ON_OFF_ADDR,gu8_BackLitOnOff);
								
								if(!gu8_BackLitOnOff)
								{
									//WHITE_BLIT_OFF;
								}
								else
								{
									//WHITE_BLIT_ON;
								}
							}
						break;
						case 3:
							if(gu8_TM_RH_ScanTime != dummy)
							{
								gu8_TM_RH_ScanTime = dummy;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_RH_SCAN_TIME_ADDR,gu8_TM_RH_ScanTime);
							}
						break;
						case 4:
							if(DP2_Upper_Alm_ON != dummy)
							{
								DP2_Upper_Alm_ON = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_ON,DP2_Upper_Alm_ON);
							}
						break;
						case 5:
							if(DP2_Upper_Alm_OFF != dummy)
							{
								DP2_Upper_Alm_OFF = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_OFF,DP2_Upper_Alm_OFF);
							}
						break;
						case 6:
							if(DP2_Lower_Alm_OFF != dummy)
							{
								DP2_Lower_Alm_OFF = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_OFF,DP2_Lower_Alm_OFF);
							}
						break;
						case 7:
							if(DP2_Lower_Alm_ON != dummy)
							{
								DP2_Lower_Alm_ON = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_ON,DP2_Lower_Alm_ON);
							}
						break;								
						case 8:
							if(TM_Upper_Alm_ON != dummy)
							{
								TM_Upper_Alm_ON = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
							}
						break;
						case 9:
							if(TM_Upper_Alm_OFF != dummy)
							{
								TM_Upper_Alm_OFF = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_OFF,TM_Upper_Alm_OFF);
							}
						break;
						case 10:
							if(TM_Lower_Alm_OFF != dummy)
							{
								TM_Lower_Alm_OFF = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_OFF,TM_Lower_Alm_OFF);
							}
						break;
						case 11:
							if(TM_Lower_Alm_ON != dummy)
							{
								TM_Lower_Alm_ON = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_ON,TM_Lower_Alm_ON);
							}
						break;
						case 12:
							if(TM_Unit != dummy)
							{
								TM_Unit = dummy;
								TMUnitChange();
							}
						break;
						case 13:
							if(RH_Upper_Alm_ON != dummy)
							{
								RH_Upper_Alm_ON = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_ON,RH_Upper_Alm_ON);
							}
						break;
						case 14:
							if(RH_Upper_Alm_OFF != dummy)
							{
								RH_Upper_Alm_OFF = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_OFF,RH_Upper_Alm_OFF);
							}
						break;
						case 15:
							if(RH_Lower_Alm_OFF != dummy)
							{
								RH_Lower_Alm_OFF = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_OFF,RH_Lower_Alm_OFF);
							}
						break;
						case 16:
							if(RH_Lower_Alm_ON != dummy)
							{
								RH_Lower_Alm_ON = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_ON,RH_Lower_Alm_ON);
							}
						break;
						case 17:
							if(Temp_RTC_ARR[0] != dummy)
							{
								Temp_RTC_ARR[0] = dummy;
								b.RTCChangeOccure=1;
							}
						break;
						case 18:
							if(Temp_RTC_ARR[1] != dummy)
							{
								Temp_RTC_ARR[1] = dummy;
								b.RTCChangeOccure=1;
							}
						break;
						case 19:
							if(Temp_RTC_ARR[2] != dummy)
							{
								Temp_RTC_ARR[2] = dummy;
								b.RTCChangeOccure=1;
							}
						break;
						case 20:
							if(Temp_RTC_ARR[3] != dummy)
							{
								Temp_RTC_ARR[3] = dummy;
								b.RTCChangeOccure=1;
							}
						break;
						case 21:
							if(Temp_RTC_ARR[4] != dummy)
							{
								Temp_RTC_ARR[4] = dummy;
								b.RTCChangeOccure=1;
							}
						break;
						case 22:
							if(Buzzer_ON_Time != dummy)
							{
								Buzzer_ON_Time = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_ON_TIME,Buzzer_ON_Time);
							
								if(!Buzzer_ON_Time)
								{
									//b.buzzerStart=NO;
									BUZZER_OFF;
									buzzerOnTime=0;
									buzzerOffTime=0;
								}
								else
								{
									if(b.buzzerStart==YES)
									{
										buzzerOnTime=Buzzer_ON_Time;
										buzzerOffTime=0;
										BUZZER_ON;
									}
								}
							}
						break;
						case 23:
							if(Buzzer_OFF_Time != dummy)
							{
								Buzzer_OFF_Time = dummy;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_OFF_TIME,Buzzer_OFF_Time);
							}
						break;
						case 24:
							if(LogInterval != dummy)
							{
								LogInterval = dummy;
								logTimer = LogInterval;
								//FlashlogTimer=60;
								//logTimer=(unsigned long)LogInterval*60;
								eeprom_busy_wait();  eeprom_write_word ((unsigned int*)LOG_INTERVAL,LogInterval);
							}
						break;
						case 25:
							if(UART_BaudRate != dummy)
							{
								UART_BaudRate = dummy;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_BAUDRATE,UART_BaudRate);
							}
						break;
						case 26:
							if(UART_DataBits != dummy)
							{
								UART_DataBits = dummy;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_DATBITS,UART_DataBits);
							}
						break;
						case 27:
							if(UART_Parity != dummy)
							{
								UART_Parity = dummy;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_PARITY,UART_Parity);
							}
						break;
						case 28:
							if(UART_StopBit != dummy)
							{
								UART_StopBit = dummy;
								eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_STOPBIT,UART_StopBit);
							}
						break;
						case 30:
							if(!b.RH_TEMP_NC)
							{
								if(dummy1 != dummy)
								{
									if(!TM_Unit)
									{
										ss1 = (RealtemperatureC - TM_Cal_float_Value_F)*10.0;
										TM_Cal_Value_C = ss1 - dummy;
										eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
										TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
									}
									else
									{
										ss1 = (RealtemperatureF - TM_Cal_float_Value_F)*10.0;
										TM_Cal_Value_C = ss1 - dummy;
										TM_Cal_Value_C = ((float)TM_Cal_Value_C * 1.8) + 32.0;
										eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
										
										TM_Cal_Value_C = (TM_Cal_Value_C-320) / 1.8;
										TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
									}
								}
							}
							//if(TM_Cal_Count_C != dummy)
							//{
							//	TM_Cal_Count_C = dummy;
							//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT_C,TM_Cal_Count_C);
							//}
						break;
						case 31:
							
							if(!b.RH_TEMP_NC)
							{
								if(dummy1 != dummy)
								{
									ss1 = (RealhumidityRH - RH_Cal_float_Value_F)*10.0;
									RH_Cal_Value_C = ss1 - dummy;
									eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
									RH_Cal_float_Value_C = (float)RH_Cal_Value_C/10.0;
								}
							}
							//if(RH_Cal_Count_C != dummy)
							//{
							//	RH_Cal_Count_C = dummy;
							//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT_C,RH_Cal_Count_C);
							//}
						break;
					}
					
					if(b.RTCChangeOccure==1)
					{
						rtc2.day = Temp_RTC_ARR[2];
						rtc2.month = Temp_RTC_ARR[3];
						rtc2.year = Temp_RTC_ARR[4];
						rtc2.year += 2000;	
						rtc2.hour = Temp_RTC_ARR[0];
						rtc2.minute = Temp_RTC_ARR[1];
						rtc2.second = 0;
					
						ep.currentEpochTime = get_epoch_time(rtc2);
						
						b.RTCChangeOccure=0;
					}
					
					if(b.UARTChanged==1)
					{
						//Init_USARTC0(UART_BaudRate,UART_DataBits,UART_Parity,UART_StopBit);
						InitUSART(&USARTC0, &PORTC, PIN2_bm, PIN3_bm, UART_BaudRate,UART_DataBits,UART_Parity,UART_StopBit);
						b.UARTChanged=0;
					}
					
					sei();			//Global Interrupt Enable
					//----------------------------------------------------------------------
					prog_para_cnt++;
					
					if((prog_para_cnt==4) && !(gu16_parameterWord & ENABLE_DP2))
					{
						prog_para_cnt=8;
					}
					
					if((prog_para_cnt==8) && !(gu16_parameterWord & ENABLE_TEMP))
					{
						prog_para_cnt=13;
					}
					
					if((prog_para_cnt==13) && !(gu16_parameterWord & ENABLE_RH))
					{
						prog_para_cnt=17;
					}
					
					if((prog_para_cnt==29) && (!(gu16_parameterWord & ENABLE_TEMP) && !(gu16_parameterWord & ENABLE_RH)))
					{
						prog_para_cnt=1;
					}

					if(prog_para_cnt==30)
					{
						if((dummy == CustPassword) || (dummy == FactCustPassword))
						{
							prog_para_cnt=30;
							
							if((prog_para_cnt==30) && !(gu16_parameterWord & ENABLE_TEMP))
							{
								prog_para_cnt=31;
							}
							else
							{
								b.cal_mode=1;
							}
							
							if((prog_para_cnt==31) && !(gu16_parameterWord & ENABLE_RH))
							{
								prog_para_cnt=1;
							}
							else
							{
								b.cal_mode=1;
							}
						}
						else
						{
							prog_para_cnt=1;
						}
					}
					
					if(prog_para_cnt>31)
					{
						prog_para_cnt=1;
					}
					
					dummy=0;
					
					Lastpara_cnt=prog_para_cnt;
					//----------------------------------------------------------------------
					switch(prog_para_cnt)
					{
						case 1:		dummy = DeviceID;				break;
						case 2:		dummy = gu8_BackLitOnOff;		break;
						case 3:		dummy = gu8_TM_RH_ScanTime;		break;
						case 4:		dummy = DP2_Upper_Alm_ON; 		break;
						case 5:		dummy = DP2_Upper_Alm_OFF; 		break;
						case 6:		dummy = DP2_Lower_Alm_OFF;  	break;
						case 7:		dummy = DP2_Lower_Alm_ON;  		break;
						case 8:		dummy = TM_Upper_Alm_ON;  		break;
						case 9:		dummy = TM_Upper_Alm_OFF; 		break;
						case 10:	dummy = TM_Lower_Alm_OFF; 		break;
						case 11:	dummy = TM_Lower_Alm_ON;  		break;
						case 12:	dummy = TM_Unit;				break;
						case 13:	dummy = RH_Upper_Alm_ON;  		break;
						case 14:	dummy = RH_Upper_Alm_OFF;  		break;
						case 15:	dummy = RH_Lower_Alm_OFF;  		break;
						case 16:	dummy = RH_Lower_Alm_ON;  		break;
						case 17:
							Temp_RTC_ARR[0] = rtc.hour;
							dummy = rtc.hour;
						break;
						case 18:
							Temp_RTC_ARR[1] = rtc.minute;
							dummy = rtc.minute;
						break;
						case 19:
							Temp_RTC_ARR[2] = rtc.day;
							dummy = rtc.day;
						break;
						case 20:
							Temp_RTC_ARR[3] = rtc.month;
							dummy = rtc.month;
						break;
						case 21:
							Temp_RTC_ARR[4] = rtc.year;
							dummy = rtc.year;
						break;
						case 22:	dummy = Buzzer_ON_Time;	  		break;
						case 23:	dummy = Buzzer_OFF_Time;	  	break;
						case 24:	dummy = LogInterval;			break;
						case 25:	dummy = UART_BaudRate;		  	break;
						case 26:	dummy = UART_DataBits;			break;
						case 27:	dummy = UART_Parity;		 	break;
						case 28:	dummy = UART_StopBit;	 		break;
						case 29:	dummy = 0;						break;
						case 30:
						
							if(b.RH_TEMP_NC)
							{
								dummy = 0;
							}
							else
							{
								if(!TM_Unit)
								{
									dummy = (RealtemperatureC - TM_Cal_float_Value_F)*10.0;
								}
								else
								{
									dummy = (RealtemperatureF - TM_Cal_float_Value_F)*10.0;
								}
								dummy1 = dummy;
							}	
										
						break;
						case 31:
							if(b.RH_TEMP_NC)
							{
								dummy = 0;
							}
							else
							{
								dummy = (RealhumidityRH - RH_Cal_float_Value_F)*10.0;
								dummy1 = dummy;
							}
						break;
					}
					
				break;
			}
		break;
		default:
		break;
	}
}

void StartBuzzer(void)
{
	if(b.buzzerStart==NO)
	{
		b.buzzerStart=YES;
		buzzerOnTime=Buzzer_ON_Time;
		buzzerOffTime=0;
		
		if(buzzerOnTime)
		{
			BUZZER_ON;
		}
	}
}

void StopBuzzer(void)
{
	b.buzzerStart=NO;
	BUZZER_OFF;
	buzzerOnTime=0;
	buzzerOffTime=0;
}

void SendToSlave(void)
{
	memset(&Buffer1[0],0,sizeof(Buffer1));
	
	Buffer1[0]=63;
	
	Buffer1[1]=DeviceID;
	
	Buffer1[2]=BatteryPercentage;
	
	Buffer1[3]=rtcValid;
	Buffer1[4]=rtc.hour;
	Buffer1[5]=rtc.minute;
	Buffer1[6]=rtc.second;
	Buffer1[7]=rtc.day;
	Buffer1[8]=rtc.month;
	Buffer1[9]=rtc.year;
	
	Buffer1[10]=0;
	if(b.DP1_NC) 	Buffer1[10] |= DP1_FAULTY;
	if(b.DP2_NC) 	Buffer1[10] |= DP2_FAULTY;
	if(b.RH_TEMP_NC)Buffer1[10] |= RH_TEMP_FAULTY;
	
	memcpy(&Buffer1[11],(uint8_t*)&Dpressure1,4);
	memcpy(&Buffer1[15],(uint8_t*)&Dpressure2,4);
	memcpy(&Buffer1[19],(uint8_t*)&temperatureC,4);
	memcpy(&Buffer1[23],(uint8_t*)&humidityRH,4);
	memcpy(&Buffer1[27],(uint8_t*)&DP1_Min,4);
	memcpy(&Buffer1[31],(uint8_t*)&DP1_Max,4);
	memcpy(&Buffer1[35],(uint8_t*)&DP2_Min,4);
	memcpy(&Buffer1[39],(uint8_t*)&DP2_Max,4);
	memcpy(&Buffer1[43],(uint8_t*)&TM_Min,4);
	memcpy(&Buffer1[47],(uint8_t*)&TM_Max,4);
	memcpy(&Buffer1[51],(uint8_t*)&RH_Min,4);
	memcpy(&Buffer1[55],(uint8_t*)&RH_Max,4);
	
	if(gu16_parameterWord & ENABLE_DP1)
	{
		Buffer1[59] = DP1_Alrm_ON;
	}

	if(gu16_parameterWord & ENABLE_DP2)
	{
		Buffer1[60] = DP2_Alrm_ON;
	}

	if(gu16_parameterWord & ENABLE_TEMP)
	{
		Buffer1[61] = TM_Alrm_ON;
		
		if(TM_Unit)
		{
			Buffer1[61] |= 0x80;
		}
	}

	if(gu16_parameterWord & ENABLE_RH)
	{
		Buffer1[62] = RH_Alrm_ON;
	}
	
	Buffer1[63]=find_Checksum(63,&Buffer1[0]);
	
	SendToUART(0,&Buffer1[0],64);
	
	/*Buffer1[0]=0xFD;
	Buffer1[1]=0x00;
	Buffer1[2]=0x01;
	
	Buffer1[3]=DeviceID;
	
	Buffer1[4]=BatteryPercentage;
	
	Buffer1[5]=rtcValid;
	Buffer1[6]=rtc.hour;
	Buffer1[7]=rtc.minute;
	Buffer1[8]=rtc.second;
	Buffer1[9]=rtc.day;
	Buffer1[10]=rtc.month;
	Buffer1[11]=rtc.year;
	
	Buffer1[12]=0;
	if(b.DP1_NC) 	Buffer1[12] |= DP1_FAULTY;
	if(b.DP2_NC) 	Buffer1[12] |= DP2_FAULTY;
	if(b.RH_TEMP_NC)Buffer1[12] |= RH_TEMP_FAULTY;
	
	memcpy(&Buffer1[13],(uint8_t*)&Dpressure1,4);
	memcpy(&Buffer1[17],(uint8_t*)&Dpressure2,4);
	memcpy(&Buffer1[21],(uint8_t*)&temperatureC,4);
	memcpy(&Buffer1[25],(uint8_t*)&humidityRH,4);
	memcpy(&Buffer1[29],(uint8_t*)&DP1_Min,4);
	memcpy(&Buffer1[33],(uint8_t*)&DP1_Max,4);
	memcpy(&Buffer1[37],(uint8_t*)&DP2_Min,4);
	memcpy(&Buffer1[41],(uint8_t*)&DP2_Max,4);
	memcpy(&Buffer1[45],(uint8_t*)&TM_Min,4);
	memcpy(&Buffer1[49],(uint8_t*)&TM_Max,4);
	memcpy(&Buffer1[53],(uint8_t*)&RH_Min,4);
	memcpy(&Buffer1[57],(uint8_t*)&RH_Max,4);
	
	if(gu16_parameterWord & ENABLE_DP1)
	{
		Buffer1[61] = DP1_Alrm_ON;
	}

	if(gu16_parameterWord & ENABLE_DP2)
	{
		Buffer1[62] = DP2_Alrm_ON;
	}

	if(gu16_parameterWord & ENABLE_TEMP)
	{
		Buffer1[63] = TM_Alrm_ON;
		
		if(TM_Unit)
		{
			Buffer1[63] |= 0x80;
		}
	}

	if(gu16_parameterWord & ENABLE_RH)
	{
		Buffer1[64] = RH_Alrm_ON;
	}
	
	Buffer1[65]=CalCRC(&Buffer1[1],64);
	Buffer1[66]=0xFC;
	
	SendToUART(1,&Buffer1[0],67);
	*/
}

//------------------------------------------------------------------------------
uint8_t find_Checksum(unsigned short Count,uint8_t *msg)
{
	unsigned long Total=0;
	unsigned short i=0;
	
	for (i=0; i<Count; i++)
	{
		Total += (unsigned long)*(msg + i);
	}
	Total = Total & 0xFF;
	Total = ((~Total) + 1) & 0xFF;
	
	return (uint8_t)(Total);
}

void InitSystemClock(void)
{	
	//Set Clock to Internal 32MHz
	OSC.CTRL = OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));
	
	//Write CCP to change clock option otherwise change will not happens -----------------------------
	CCP = CCP_IOREG_gc;	
	asm("LDI R24,0x01"); //For switching to 32MHz internal RC. Modify 0x01 as required
	asm("STS 0x0040,R24");
	//------------------------------------------------------------------------------------------------
	
	OSC.CTRL &= ~OSC_RC2MEN_bm;
			
	//Select Clock Source for RTC to Internal 32.768 KHz
	CLK.RTCCTRL = (CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm);
}

void FillRamBuffer(uint8_t logtype,uint8_t userID,unsigned short password)
{
	if((gu16_parameterWord & ENABLE_RTC) && !DP_StartUpTimer && !TMRH_StartUpTimer && rtcValid)
	{
		//unsigned short i=0;
		
		RAMBufferInd = RAM_FILL_START + (RAMBufferLog * LOG_SIZE);
		//i=RAMBufferInd;
	
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&ep.currentEpochTime,4);
		RAMBufferInd += 4;
	
		RAMBuffer[RAMBufferInd++]=DeviceID;
		RAMBuffer[RAMBufferInd++]=logtype;
		RAMBuffer[RAMBufferInd++]=userID;
		RAMBuffer[RAMBufferInd++]=password & 0x00FF;
		RAMBuffer[RAMBufferInd++]=password >> 8;
	
		RAMBuffer[RAMBufferInd]=0;
		if(b.DP1_NC) 	RAMBuffer[RAMBufferInd] |= DP1_FAULTY;
		if(b.DP2_NC) 	RAMBuffer[RAMBufferInd] |= DP2_FAULTY;
		if(b.RH_TEMP_NC)RAMBuffer[RAMBufferInd] |= RH_TEMP_FAULTY;
		RAMBufferInd++;
	
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&Dpressure1,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&Dpressure2,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&temperatureC,4);		RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&humidityRH,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&DP1_Min,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&DP1_Max,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&DP2_Min,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&DP2_Max,4);			RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&TM_Min,4);				RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&TM_Max,4);				RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&RH_Min,4);				RAMBufferInd += 4;
		memcpy(&RAMBuffer[RAMBufferInd],(uint8_t*)&RH_Max,4);				RAMBufferInd += 4;
	
		if(gu16_parameterWord & ENABLE_DP1)
		{
			if(!DP1_Alrm_ON) 
			{
				if(logtype==DP1_ALM_RESTORE_LOG)
				{
					if(LastDP1_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd++] = 1;
					else if(LastDP1_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd++] = 2;
					else								 RAMBuffer[RAMBufferInd++] = 0;
				}
				else
				{
					RAMBuffer[RAMBufferInd++] = 0;
				}
			}
			else
			{
				if(LastDP1_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd++] = 1;
				else if(LastDP1_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd++] = 2;
				else								 RAMBuffer[RAMBufferInd++] = 0;
				
				//if(DP_Alrm_ON==UPPER_ALARM) RAMBuffer[RAMBufferInd++] = 1;
				//else						  RAMBuffer[RAMBufferInd++] = 2;
			}
		}
		else
		{
			RAMBuffer[RAMBufferInd++] = 0;
		}
		
		if(gu16_parameterWord & ENABLE_DP2)
		{
			if(!DP2_Alrm_ON)
			{
				if(logtype==DP2_ALM_RESTORE_LOG)
				{
					if(LastDP2_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd++] = 1;
					else if(LastDP2_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd++] = 2;
					else								 RAMBuffer[RAMBufferInd++] = 0;
				}
				else
				{
					RAMBuffer[RAMBufferInd++] = 0;
				}
			}
			else
			{
				if(LastDP2_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd++] = 1;
				else if(LastDP2_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd++] = 2;
				else								 RAMBuffer[RAMBufferInd++] = 0;
			
				//if(DP_Alrm_ON==UPPER_ALARM) RAMBuffer[RAMBufferInd++] = 1;
				//else						  RAMBuffer[RAMBufferInd++] = 2;
			}
		}
		else
		{
			RAMBuffer[RAMBufferInd++] = 0;
		}
	
		if(gu16_parameterWord & ENABLE_TEMP)
		{
			if(!TM_Alrm_ON) 
			{
				if(logtype==TM_ALM_RESTORE_LOG)
				{
					if(LastTM_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd] = 1;
					else if(LastTM_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd] = 2;
					else								 RAMBuffer[RAMBufferInd] = 0;
				}
				else
				{
					RAMBuffer[RAMBufferInd] = 0;
				}
			}
			else
			{
				if(LastTM_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd] = 1;
				else if(LastTM_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd] = 2;
				else								 RAMBuffer[RAMBufferInd] = 0;
				
				//if(TM_Alrm_ON==UPPER_ALARM) RAMBuffer[RAMBufferInd] = 1;
				//else						  RAMBuffer[RAMBufferInd] = 2;
			}
		
			if(TM_Unit)
			{
				RAMBuffer[RAMBufferInd] |= 0x80;
			}
		
			RAMBufferInd++;
		}
		else
		{
			RAMBuffer[RAMBufferInd++] = 0;
		}
	
		if(gu16_parameterWord & ENABLE_RH)
		{
			if(!RH_Alrm_ON) 
			{
				if(logtype==RH_ALM_RESTORE_LOG)
				{
					if(LastRH_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd++] = 1;
					else if(LastRH_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd++] = 2;
					else								 RAMBuffer[RAMBufferInd++] = 0;
				}
				else
				{
					RAMBuffer[RAMBufferInd++] = 0;
				}
			}
			else
			{
				if(LastRH_Alrm_ON==UPPER_ALARM)		 RAMBuffer[RAMBufferInd++] = 1;
				else if(LastRH_Alrm_ON==LOWER_ALARM) RAMBuffer[RAMBufferInd++] = 2;
				else								 RAMBuffer[RAMBufferInd++] = 0;
				
				//if(RH_Alrm_ON==UPPER_ALARM) RAMBuffer[RAMBufferInd++] = 1;
				//else						  RAMBuffer[RAMBufferInd++] = 2;
			}
		}
		else
		{
			RAMBuffer[RAMBufferInd++] = 0;
		}
	
		RAMBufferLog++;
		if(RAMBufferLog > 29) RAMBufferLog=0;
	}
}

void Init_DMA(void)
{
	DMA_CTRL = DMA_RESET_bm;
	DMA_CTRL = DMA_ENABLE_bm;
	DMA_INTFLAGS = 0x00;

	DMA_CH0_CTRLB = DMA_CH_TRNINTLVL_HI_gc | DMA_CH_ERRINTLVL_HI_gc;
	DMA_CH0_CTRLA = DMA_CH_BURSTLEN_1BYTE_gc;
	DMA_CH0_ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA_CH0_TRIGSRC = DMA_CH_TRIGSRC_USARTC0_DRE_gc;
	DMA_CH0_TRFCNT = 100;
	DMA_CH0_REPCNT = 0;
	DMA_CH0_SRCADDR0 = (unsigned short)RAMBuffer;
	DMA_CH0_SRCADDR1 = (unsigned short)RAMBuffer>>8;
	DMA_CH0_DESTADDR0 = &USARTC0.DATA;
	DMA_CH0_DESTADDR1 = (unsigned short)&USARTC0.DATA >> 8;
	//DMA_CH0_CTRLA |= DMA_ENABLE_bm | DMA_CH_SINGLE_bm;
	//DMA_CH0_CTRLA |= DMA_CH_TRFREQ_bm;
	
	//DMA_CTRL = 0x00;
}

ISR(DMA_CH0_vect)
{	
	DMA_CH0_CTRLB |= DMA_CH_TRNIF_bm | DMA_CH_ERRIF_bm;
	
	b.logtransferStart=0;
	
	if(!b.FlashReadCmd && !b.Flash24ReadCmd && !b.MinMaxMeanLogReadCmd && !b.MeanHrLogReadCmd && !b.RamReadCmd && !b.RamAllReadCmd)
	{
		if(comport==0)
		{
			while(!(USARTC0_STATUS & USART_DREIF_bm));
			while(!(USARTC0_STATUS & USART_TXCIF_bm));
			
			_delay_ms(200);
			
			RS485_TX0_DIS;	
		}
		else if(comport==1)
		{
			while(!(USARTC1_STATUS & USART_DREIF_bm));
			while(!(USARTC1_STATUS & USART_TXCIF_bm));
			
			_delay_ms(200);
			
			RS485_TX1_DIS;
		}
	}		
}

void AutoSendDataResponse(uint8_t SrcPort)
{
	uint8_t j=0;
	
	Buffer1[0]=0xFD;
	Buffer1[1]=DeviceID;
	Buffer1[2]=PARA_WITH_ALM_READ_CMD;
	Buffer1[3]=0x00;
	if(b.paraIdNotValid) 	Buffer1[3] |= INVALID_PARA;
	if(b.DP1_NC) 			Buffer1[3] |= DP1_FAULTY;
	if(b.DP2_NC) 			Buffer1[3] |= DP2_FAULTY;
	if(b.RH_TEMP_NC) 		Buffer1[3] |= RH_TEMP_FAULTY;
	
	j=4;
	
	if(b.DP1_NC) 			
	{
		Buffer1[j++] |= DP1_FAULTY;
	}
	else
	{
		Buffer1[j++]=0;
		
		//Fill DP value
		templong = Dpressure1*100;
		j += fillValue(&Buffer1[j],templong);	
	}
	
	Buffer1[j++] = 0xEE;	//Field Separator
	
	if(b.DP2_NC)
	{
		Buffer1[j++] |= DP2_FAULTY;
	}
	else
	{
		Buffer1[j++]=0;
		
		//Fill DP value
		templong = Dpressure2*100;
		j += fillValue(&Buffer1[j],templong);
	}
	
	Buffer1[j++] = 0xEE;	//Field Separator
	
	if(b.RH_TEMP_NC) 			
	{
		Buffer1[j++] |= RH_TEMP_FAULTY;
	}
	else
	{
		Buffer1[j++]=0;
		
		//Fill Temp value
		if(!TM_Unit)
		{
			tempshort = temperatureC*100;
		}
		else
		{
			tempshort = temperatureF*100;
		}
		
		j += fillValue(&Buffer1[j],tempshort);
	}
	
	Buffer1[j++] = 0xEE;	//Field Separator
	
	if(b.RH_TEMP_NC) 			
	{
		Buffer1[j++] |= RH_TEMP_FAULTY;
	}
	else
	{
		Buffer1[j++]=0;
		
		//Fill Temp value
		tempshort = humidityRH*100;
		j += fillValue(&Buffer1[j],tempshort);
	}
	
	Buffer1[j++] = 0xEE;	//Field Separator

	if(gu16_parameterWord & ENABLE_DP1)
	{
		if(!DP1_Alrm_ON)
		{
			Buffer1[j++] = 0;
		}
		else
		{
			if(DP1_Alrm_ON==UPPER_ALARM) Buffer1[j++] = 1;
			else						  Buffer1[j++] = 2;
		}
	}
	else
	{
		Buffer1[j++] = 0;
	}
	
	if(gu16_parameterWord & ENABLE_DP2)
	{
		if(!DP2_Alrm_ON)
		{
			Buffer1[j++] = 0;
		}
		else
		{
			if(DP2_Alrm_ON==UPPER_ALARM) Buffer1[j++] = 1;
			else						  Buffer1[j++] = 2;
		}
	}
	else
	{
		Buffer1[j++] = 0;
	}
	
	if(gu16_parameterWord & ENABLE_TEMP)
	{
		if(!TM_Alrm_ON)
		{
			Buffer1[j++] = 0;
		}
		else
		{
			if(TM_Alrm_ON==UPPER_ALARM) Buffer1[j++] = 1;
			else						  Buffer1[j++] = 2;
		}
	}
	else
	{
		Buffer1[j++] = 0;
	}
	
	if(gu16_parameterWord & ENABLE_RH)
	{
		if(!RH_Alrm_ON)
		{
			Buffer1[j++] = 0;
		}
		else
		{
			if(RH_Alrm_ON==UPPER_ALARM) Buffer1[j++] = 1;
			else						  Buffer1[j++] = 2;
		}
	}
	else
	{
		Buffer1[j++] = 0;
	}
	
	Buffer1[j++] = 0xEE;	//Field Separator
	
	#ifdef DISABLE_DOOR_SENSING

		//Door Close
		Buffer1[j++] = 0x0F;
		Buffer1[j++] = 0x0F;
	
	#else
		
	if(b.doorStatus==OPEN)
	{
		//Door Open
		Buffer1[j++] = 0x0E;
		Buffer1[j++] = 0x0E;	
	}
	else
	{
		//Door Close
		Buffer1[j++] = 0x0F;
		Buffer1[j++] = 0x0F;
	}
		
	#endif
	
	Buffer1[j++] = 0xEE;	//Field Separator
	
	memcpy(&Buffer1[j],&gu8ar_SrNumber[8],8);
	j+=8;
	
	Buffer1[j++] = 0xEE;	//Field Separator
	
	memcpy(&Buffer1[j],(uint8_t*)&ep.currentEpochTime,4);
	j+=4;
	
	Buffer1[j]=CalCRC(&Buffer1[1],j-1);
	j++;
	
	Buffer1[j++]=0xFC;
	
	SendToUART(SrcPort,&Buffer1[0],j);
	//SetTxmode(SrcPort,Buffer1,j);
}
void ServePCMsg(uint8_t SrcPort)
{
	#ifdef DEBUG_RCV_CMD
		opstr(0,"\r\nMsg OK");
	#endif
	uint8_t index=0,j=0,lu8_sendResponse=0,lu8_GroupDelay=0,m=0;
	
	b.paraIdNotValid=0;
	
	if(gu8_Mac2ValidTimer) gu8_Mac2ValidTimer=60;
	
	if(RxBuffer[2]==PARA_WITH_ALM_READ_CMD)
	{	
		if(RxInd==6)
		{
			if((RxBuffer[3]==0x00) || (RxBuffer[3]==gu8_groupID))
			{
				lu8_sendResponse = 1;
				
				//if(gu8_groupID>1)
				//{
					//for(m=0;m<gu8_groupID;m++)
					//{
						//_delay_ms(100);
					//}
				//}
				
				lu8_GroupDelay = (DeviceID - 1)%gu8_DeviceInGroup;
				
				if(lu8_GroupDelay)
				{
					for(m=0;m<lu8_GroupDelay;m++)
					{
						_delay_ms(1);
					}
				}
			}
			else
			{
				lu8_sendResponse = 0;
			}
		}
		else
		{	
			lu8_sendResponse = 1;
		}
		
		if(lu8_sendResponse==1)
		{
			RxBuffer[0]=0xFD;
			
			RxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	RxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			RxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			RxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		RxBuffer[3] |= RH_TEMP_FAULTY;
			
			j=4;
			
			if(b.DP1_NC) 			
			{
				RxBuffer[j++] |= DP1_FAULTY;
			}
			else
			{
				RxBuffer[j++]=0;
				
				//Fill DP value
				templong = Dpressure1*100;
				j += fillValue(&RxBuffer[j],templong);	
			}
			
			RxBuffer[j++] = 0xEE;	//Field Separator
			
			if(b.DP2_NC)
			{
				RxBuffer[j++] |= DP2_FAULTY;
			}
			else
			{
				RxBuffer[j++]=0;
				
				//Fill DP value
				templong = Dpressure2*100;
				j += fillValue(&RxBuffer[j],templong);
			}
			
			RxBuffer[j++] = 0xEE;	//Field Separator
			
			if(b.RH_TEMP_NC) 			
			{
				RxBuffer[j++] |= RH_TEMP_FAULTY;
			}
			else
			{
				RxBuffer[j++]=0;
				
				//Fill Temp value
				if(!TM_Unit)
				{
					tempshort = temperatureC*100;
				}
				else
				{
					tempshort = temperatureF*100;
				}
				
				j += fillValue(&RxBuffer[j],tempshort);
			}
			
			RxBuffer[j++] = 0xEE;	//Field Separator
			
			if(b.RH_TEMP_NC) 			
			{
				RxBuffer[j++] |= RH_TEMP_FAULTY;
			}
			else
			{
				RxBuffer[j++]=0;
				
				//Fill Temp value
				tempshort = humidityRH*100;
				j += fillValue(&RxBuffer[j],tempshort);
			}
			
			RxBuffer[j++] = 0xEE;	//Field Separator

			if(gu16_parameterWord & ENABLE_DP1)
			{
				if(!DP1_Alrm_ON)
				{
					RxBuffer[j++] = 0;
				}
				else
				{
					if(DP1_Alrm_ON==UPPER_ALARM) RxBuffer[j++] = 1;
					else						  RxBuffer[j++] = 2;
				}
			}
			else
			{
				RxBuffer[j++] = 0;
			}
			
			if(gu16_parameterWord & ENABLE_DP2)
			{
				if(!DP2_Alrm_ON)
				{
					RxBuffer[j++] = 0;
				}
				else
				{
					if(DP2_Alrm_ON==UPPER_ALARM) RxBuffer[j++] = 1;
					else						  RxBuffer[j++] = 2;
				}
			}
			else
			{
				RxBuffer[j++] = 0;
			}
			
			if(gu16_parameterWord & ENABLE_TEMP)
			{
				if(!TM_Alrm_ON)
				{
					RxBuffer[j++] = 0;
				}
				else
				{
					if(TM_Alrm_ON==UPPER_ALARM) RxBuffer[j++] = 1;
					else						  RxBuffer[j++] = 2;
				}
			}
			else
			{
				RxBuffer[j++] = 0;
			}
			
			if(gu16_parameterWord & ENABLE_RH)
			{
				if(!RH_Alrm_ON)
				{
					RxBuffer[j++] = 0;
				}
				else
				{
					if(RH_Alrm_ON==UPPER_ALARM) RxBuffer[j++] = 1;
					else						  RxBuffer[j++] = 2;
				}
			}
			else
			{
				RxBuffer[j++] = 0;
			}
			
			RxBuffer[j++] = 0xEE;	//Field Separator
			
			#ifdef DISABLE_DOOR_SENSING

				//Door Close
				RxBuffer[j++] = 0x0F;
				RxBuffer[j++] = 0x0F;
			
			#else
				
			if(b.doorStatus==OPEN)
			{
				//Door Open
				RxBuffer[j++] = 0x0E;
				RxBuffer[j++] = 0x0E;	
			}
			else
			{
				//Door Close
				RxBuffer[j++] = 0x0F;
				RxBuffer[j++] = 0x0F;
			}
				
			#endif
			
			RxBuffer[j++] = 0xEE;	//Field Separator
			
			memcpy(&RxBuffer[j],&gu8ar_SrNumber[8],8);
			j+=8;
			
			RxBuffer[j++] = 0xEE;	//Field Separator
			
			memcpy(&RxBuffer[j],(uint8_t*)&ep.currentEpochTime,4);
			j+=4;
			
			RxBuffer[j]=CalCRC(&RxBuffer[1],j-1);
			j++;
			
			RxBuffer[j++]=0xFC;
		
			SendToUART(SrcPort,RxBuffer,j);
			//SetTxmode(SrcPort,RxBuffer,j);
		}
	}
	else if(RxBuffer[2]==PARA_WRITE_CMD)
	{
		#ifdef DEBUG_RCV_CMD
			opstr(0,"\r\nWrite Cmd");
		#endif
		
		switch(RxBuffer[3])
		{
			case ACK_PW_ID:	
			case ALM_ACK_ID:			tempshort = findValue(&RxBuffer[5],RxInd-7);	break;
			case XBEE_MAC_ADDR_ID:		break;
			case SET_DPARA_PWD_ID:		break;
			case SRNO_ID:		  		break;
			case BRDSTR_ID:		  		break;
			case BRDSTP_ID:		  		break;
			case DATETIME_ID:	  		break;
			case EXT_FLASH_ERASE_ID:	break;
			case DFLT_RTC_ID:			break;
			case DFLT_CAL_ID:			break;
			case CORR_RTC_DATA_ID:		break;
			case BIG_FONT_LED_SET_ID:	break;
			case DP1CAL_ID:
			case DP2CAL_ID:
			case TMCAL_ID:
			case RHCAL_ID:
			
				tempshort = findValue(&RxBuffer[4],5);
				
			break;
			
			case DP_SW_FACT_ID:
			
				tempshort = findValue(&RxBuffer[6],5);
				if(RxBuffer[5]=='-') tempshort *= (-1);	
				
			break;
			
			case DP_LIMIT_ID:
			
				tempshort = findValue(&RxBuffer[5],5);
			
			break;
			
			default: 
				tempshort = findValue(&RxBuffer[4],RxInd-6);
			break;
		}
			
		#ifdef DEBUG_RCV_CMD
			opstr(0,"\r\nValue:");
			print_float(0,tempshort,test,0);	
		#endif

		cli();
		
		switch(RxBuffer[3])
		{
			case BIG_FONT_LED_SET_ID:
			
				switch(RxBuffer[4])
				{
					case TEMP_DISPLAY:
					
						gu8_TM_LEDBlinkForPara=RxBuffer[5];
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_LED_SETTING_ADDR,gu8_TM_LEDBlinkForPara);
					
					break;
					
					case RH_DISPLAY:
					
						gu8_RH_LEDBlinkForPara=RxBuffer[5];
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RH_LED_SETTING_ADDR,gu8_RH_LEDBlinkForPara);
					
					break;
					
					case DP1_DISPLAY:
					
						gu8_DP1_LEDBlinkForPara=RxBuffer[5];
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_LED_SETTING_ADDR,gu8_DP1_LEDBlinkForPara);
					
					break;
					
					case DP2_DISPLAY:
					
						gu8_DP2_LEDBlinkForPara=RxBuffer[5];
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_LED_SETTING_ADDR,gu8_DP2_LEDBlinkForPara);
					
					break;
				}
			
			break;
			
			case SET_DPARA_PWD_ID:
			
				us2=0;	//password
				us1 = RxBuffer[4]-'0';		us1 *= 1000;			us2 += us1;		us1 = 0;
				us1 = RxBuffer[5]-'0';		us1 *= 100;				us2 += us1;		us1 = 0;
				us1 = RxBuffer[6]-'0';		us1 *= 10;				us2 += us1;		us1 = 0;
				us1 = RxBuffer[7]-'0';								us2 += us1;		us1 = 0;
				
				if(us2 == FACTORY_PARASET_PWD)
				{
					us3=0;	//Display Parameter bit pattern
					us1 = RxBuffer[8]-'0';		us1 *= 10000;			us3 += us1;		us1 = 0;
					us1 = RxBuffer[9]-'0';		us1 *= 1000;			us3 += us1;		us1 = 0;
					us1 = RxBuffer[10]-'0';		us1 *= 100;				us3 += us1;		us1 = 0;
					us1 = RxBuffer[11]-'0';		us1 *= 10;				us3 += us1;		us1 = 0;
					us1 = RxBuffer[12]-'0';								us3 += us1;		us1 = 0;
					
					gu16_parameterWord=us3;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DISP_PARA_SELECT,gu16_parameterWord);

					//wdt_enable(WDTO_2S);
					//b.resetDevice=1;
					gu8_restartTimer = 3;
					
					//CPU_CCP=0xD8;
					//RST_CTRL=RST_SWRST_bm;
				}
			
			break;
			
			case DATETIME_ID:
			
				a1 = (RxBuffer[4]-'0');
				a1 *= 10;
				a2 = (RxBuffer[5]-'0');
				a1 += a2;
				rtc2.day = a1;//HEX2BCD(a1);

				a1 = (RxBuffer[6]-'0');
				a1 *= 10;
				a2 = (RxBuffer[7]-'0');
				a1 += a2;
				rtc2.month = a1;//HEX2BCD(a1);

				a1 = (RxBuffer[8]-'0');
				a1 *= 10;
				a2 = (RxBuffer[9]-'0');
				a1 += a2;
				rtc2.year = a1;//HEX2BCD(a1);
				rtc2.year += 2000;	//Year
				
				a1 = (RxBuffer[10]-'0');
				a1 *= 10;
				a2 = (RxBuffer[11]-'0');
				a1 += a2;
				rtc2.hour = a1;//HEX2BCD(a1);

				a1 = (RxBuffer[12]-'0');
				a1 *= 10;
				a2 = (RxBuffer[13]-'0');
				a1 += a2;
				rtc2.minute = a1;//HEX2BCD(a1);

				a1 = (RxBuffer[14]-'0');
				a1 *= 10;
				a2 = (RxBuffer[15]-'0');
				a1 += a2;
				rtc2.second = a1;//HEX2BCD(a1);

				ep.currentEpochTime = get_epoch_time(rtc2);
				rtcValid=1;

			break;
			case DP1UAON_ID:	
				ss1 = (tempshort/10);
				if((ss1<=1000) && (ss1>= DP1_Upper_Alm_OFF))
				{
					DP1_Upper_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_UP_ALM_ON,DP1_Upper_Alm_ON); 
				}
			break;
			case DP1UAOFF_ID:
				ss1 = (tempshort/10);
				if((ss1<=DP1_Upper_Alm_ON) && (ss1>= DP1_Lower_Alm_OFF))
				{
					DP1_Upper_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_UP_ALM_OFF,DP1_Upper_Alm_OFF); 
				}
			break;
			case DP1LAON_ID:	
				ss1 = (tempshort/10);
				if((ss1<=DP1_Lower_Alm_OFF) && (ss1>= -1000))	
				{
					DP1_Lower_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_LO_ALM_ON,DP1_Lower_Alm_ON); 
				}
			break;
			case DP1LAOFF_ID:	
				ss1 = (tempshort/10);
				if((ss1<=DP1_Upper_Alm_OFF) && (ss1>= DP1_Lower_Alm_ON))	
				{
					DP1_Lower_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_LO_ALM_OFF,DP1_Lower_Alm_OFF); 
				}
			break;
			
			case DP2UAON_ID:
				ss1 = (tempshort/10);
				if((ss1<=1000) && (ss1>= DP2_Upper_Alm_OFF))
				{
					DP2_Upper_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_ON,DP2_Upper_Alm_ON);
				}
			break;
			case DP2UAOFF_ID:
				ss1 = (tempshort/10);
				if((ss1<=DP2_Upper_Alm_ON) && (ss1>= DP2_Lower_Alm_OFF))
				{
					DP2_Upper_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_OFF,DP2_Upper_Alm_OFF);
				}
			break;
			case DP2LAON_ID:
				ss1 = (tempshort/10);
				if((ss1<=DP2_Lower_Alm_OFF) && (ss1>= -1000))
				{
					DP2_Lower_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_ON,DP2_Lower_Alm_ON);
				}
			break;
			case DP2LAOFF_ID:
				ss1 = (tempshort/10);
				if((ss1<=DP2_Upper_Alm_OFF) && (ss1>= DP2_Lower_Alm_ON))
				{
					DP2_Lower_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_OFF,DP2_Lower_Alm_OFF);
				}
			break;
			
			case TMUAON_ID:	
				ss1 = (tempshort/10);
				if(!TM_Unit)
				{
					if((ss1<=1250) && (ss1>= TM_Upper_Alm_OFF))
					{
						TM_Upper_Alm_ON=ss1;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
					}
				}
				else
				{
					if((ss1<=2570) && (ss1>= TM_Upper_Alm_OFF))
					{
						TM_Upper_Alm_ON=ss1;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
					}
				}
			break;
			case TMUAOFF_ID:
				ss1 = (tempshort/10);
				if((ss1<=TM_Upper_Alm_ON) && (ss1>= TM_Lower_Alm_OFF))		
				{
					TM_Upper_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_OFF,TM_Upper_Alm_OFF);
				}
			break;
			case TMLAON_ID:		
				ss1 = (tempshort/10);
				if((ss1<=TM_Lower_Alm_OFF) && (ss1>= -400))
				{
					TM_Lower_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_ON,TM_Lower_Alm_ON);
				}
			break;
			case TMLAOFF_ID:	
				ss1 = (tempshort/10);
				if((ss1<=TM_Upper_Alm_OFF) && (ss1>= TM_Lower_Alm_ON))	
				{
					TM_Lower_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_OFF,TM_Lower_Alm_OFF);
				}
			break;
			case RHUAON_ID:		
				ss1 = (tempshort/10);
				if((ss1<=1000) && (ss1>= RH_Upper_Alm_OFF))
				{
					RH_Upper_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_ON,RH_Upper_Alm_ON); 
				}
			break;
			case RHUAOFF_ID:	
				ss1 = (tempshort/10);	
				if((ss1<=RH_Upper_Alm_ON) && (ss1>= RH_Lower_Alm_OFF))
				{
					RH_Upper_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_OFF,RH_Upper_Alm_OFF); 
				}
			break;
			case RHLAON_ID:		
				ss1 = (tempshort/10);
				if((ss1<=RH_Lower_Alm_OFF) && (ss1>= 0))
				{
					RH_Lower_Alm_ON=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_ON,RH_Lower_Alm_ON); 
				}
			break;
			case RHLAOFF_ID:	
				ss1 = (tempshort/10);
				if((ss1<=RH_Upper_Alm_OFF) && (ss1>= RH_Lower_Alm_ON))
				{
					RH_Lower_Alm_OFF=ss1;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_OFF,RH_Lower_Alm_OFF); 
				}
			break;
			case LOGINTVAL_ID:		
				
				if((tempshort>=MIN_LOG_INTERVAL) && (tempshort<=MAX_LOG_INTERVAL))
				{
					LogInterval=tempshort;
					logTimer = LogInterval;
					//FlashlogTimer=60;
					//logTimer=(unsigned long)LogInterval*60;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)LOG_INTERVAL,LogInterval);	
				}
			break;
			case DVCID_ID:		
				
				gu8_deviceIDChangeTryTimer = 10;
				gu8_deviceIDChangeTry++;
				if(gu8_deviceIDChangeTry>1)
				{
					gu8_deviceIDChangeTry=0;
					
					DeviceID=tempshort;
					gu8_groupID = ((DeviceID - 1)/gu8_DeviceInGroup)+1;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICE_ID,DeviceID);
				}
				
			break;
			case BZRON_ID:		
				Buzzer_ON_Time=tempshort;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_ON_TIME,Buzzer_ON_Time);
				if(!Buzzer_ON_Time)
				{
					//b.buzzerStart=NO;
					BUZZER_OFF;
					buzzerOnTime=0;
					buzzerOffTime=0;
				}
				else
				{
					if(b.buzzerStart==YES)
					{
						buzzerOnTime=Buzzer_ON_Time;
						buzzerOffTime=0;
						BUZZER_ON;
					}
				}
			break;
			
			case SCROLL_TIME_ID:
				ParaScrollTime=(uint8_t)tempshort;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)PARA_SCROLL_TIME,ParaScrollTime);
			break;
			
			case TM_RH_SCAN_TIME_ID:
				gu8_TM_RH_ScanTime=(uint8_t)tempshort;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_RH_SCAN_TIME_ADDR,gu8_TM_RH_ScanTime);
			break;
			
			case EXT_FLASH_ERASE_ID:
			
				//Erase External Flash
				b.EraseFlash=1;
				
			break;
			
			case DFLT_RTC_ID:
				
				//Erase External Flash
				b.EraseFlash=1;
				
			break;
			
			case DFLT_CAL_ID:
			
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					switch(RxBuffer[4])
					{
						case '0':
					
							//DP1_Cal_Value_F=0;
							//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_F_ADDR,DP1_Cal_Value_F);

							DP1_Cal_Value_C=0;
							eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);

							//DP1_Cal_float_Value_F = 0.0;
							DP1_Cal_float_Value_C = 0.0;
						
						break;
					
						case '1':
					
							//DP2_Cal_Value_F=0;
							//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_F_ADDR,DP2_Cal_Value_F);
						
							DP2_Cal_Value_C=0;
							eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
						
							//DP2_Cal_float_Value_F = 0.0;
							DP2_Cal_float_Value_C = 0.0;
					
						break;
					
						case '2':
					
							//TM_Cal_Value_F=0;
							//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
						
							TM_Cal_Value_C=0;
							eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
						
							//TM_Cal_float_Value_F = 0.0;
							TM_Cal_float_Value_C = 0.0;
					
						break;
					
						case '3':
					
							//RH_Cal_Value_F=0;
							//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_F_ADDR,RH_Cal_Value_F);
						
							RH_Cal_Value_C=0;
							eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
						
							//RH_Cal_float_Value_F = 0.0;
							RH_Cal_float_Value_C = 0.0;
					
						break;
					}
				}
			break;

			case BZROFF_ID:		
				Buzzer_OFF_Time=tempshort;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_OFF_TIME,Buzzer_OFF_Time);
			break;
			case CAL_FPWD_ID:		
				if(tempshort==FACTORY_PASSWORD)
				{
					b.FactoryCalibrationOn=1;
					b.CustmerCalibrationOn=0;
					PCCalibrationTimer=60;
				}
			break;
			
			case CAL_CPWD_ID:		
				if((tempshort==CustPassword) || (tempshort==FactCustPassword) || (tempshort==989))
				{	
					b.FactoryCalibrationOn=0;
					b.CustmerCalibrationOn=1;
					PCCalibrationTimer=60;
					
					/*if(tempshort==989)
					{
						gu8_dp_sw_enb = 0;
					}
					else
					{
						gu8_dp_sw_enb = 1;
					}
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP_FACT_ENB_ADDR,gu8_dp_sw_enb);
					*/
				}

			break;
			case CPWD_ID:		
				if(tempshort<=999)
				{
					CustPassword=tempshort;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)CUSTOMER_PASSWORD,CustPassword);
				}
			break;
			case FCPWD_ID:
				if(tempshort<=9999)
				{
					FactCustPassword=tempshort;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)FAC_CUSTOMER_PASSWORD,FactCustPassword);
				}
			break;
			
			case DP_SW_FACT_ID:
			
				if(b.CustmerCalibrationOn==1)
				{
					index=RxBuffer[4]-'0';
					
					if(index<MAX_SUPPORTED_DP)
					{	
						su16_dp_sw_factor[index]=tempshort;
						f32_dp_sw_factor[index]=(float)su16_dp_sw_factor[index]/100.0;
						eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_SW_FACT_ADDR+(index*2)),su16_dp_sw_factor[index]);
					}
				}	
				
			break;

			case DP_LIMIT_ID:
			
				index=RxBuffer[4]-'0';
				
				if(index<MAX_SUPPORTED_DP)
				{
					u16_dp_limit[index]=tempshort;
					f32_dp_limit[index]=(float)u16_dp_limit[index]/10.0;
					eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_LIMIT_ADDR+(index*2)),u16_dp_limit[index]);
				}
			
			break;
			
			case DP1CAL_ID:
				
				if(b.FactoryCalibrationOn==1)
				{
					//DP1_Cal_Count=tempshort;
					//DP1_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT,DP1_Cal_Count);
					
					DP1_Cal_Value_F = RealDpressure1*10.0;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_F_ADDR,DP1_Cal_Value_F);
					DP1_Cal_float_Value_F = (float)DP1_Cal_Value_F/10.0;
					
					DP1_Cal_Value_C = 0;
					DP1_Cal_float_Value_C = 0;
					
					su16_dp_sw_factor[0]=0;
					f32_dp_sw_factor[0]=0.0;
					eeprom_busy_wait();  eeprom_write_word((unsigned int*)DP_SW_FACT_ADDR,su16_dp_sw_factor[0]);
					
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//DP1_Cal_Count_C=tempshort;
					
					DP1_Cal_Value_C = (RealDpressure1 - DP1_Cal_float_Value_F)*10.0;
					DP1_Cal_float_Value_C = (float)DP1_Cal_Value_C/10.0;
				}
				
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT_C,DP1_Cal_Count_C);
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
					
					DP1_Max = DEFAUT_DP1_MAX;
					DP1_Min = DEFAUT_DP1_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Max,(uint8_t*)DP1_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Min,(uint8_t*)DP1_MINIMUM,4);
				}
				
				/*if(b.FactoryCalibrationOn==1)
				{
					//DP1_Cal_Count=tempshort;
					//DP1_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT,DP1_Cal_Count);
					
					DP1_Cal_Value_F = RealDpressure1*10.0;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_F_ADDR,DP1_Cal_Value_F);
					DP1_Cal_float_Value_F = (float)DP1_Cal_Value_F/10.0;
					
					DP1_Cal_Value_C = 0;
					DP1_Cal_float_Value_C = 0;
					
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)DP1_CAL_DATE_ADDR,12);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[21],(uint8_t*)DP1_CAL_CERT_ADDR,15);
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//DP1_Cal_Count_C=tempshort;
					
					DP1_Cal_Value_C = (RealDpressure1 - DP1_Cal_float_Value_F)*10.0;
					DP1_Cal_float_Value_C = (float)DP1_Cal_Value_C/10.0;
						
					Buffer1[0] = findValue(&RxBuffer[9],2);
					Buffer1[1] = findValue(&RxBuffer[11],2);
					Buffer1[2] = findValue(&RxBuffer[13],2);
								
					if(!Buffer1[0] && !Buffer1[1] && !Buffer1[2])
					{	
						if(!DP1_UserCalDateInd) a1=NO_OF_USER_CAL_DATE-1;
						else a1=DP1_UserCalDateInd-1;
					
						us1 = DP1_USER_CAL_DATE_ADDR + (a1 * 6);
						eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)us1,6);
						
						if(memcmp(&Buffer1[0],&RxBuffer[9],6))
						{
							us1 = DP1_USER_CAL_DATE_ADDR + (DP1_UserCalDateInd * 6);
							eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)us1,6);
						
							DP1_UserCalDateInd++;
							if(DP1_UserCalDateInd>=NO_OF_USER_CAL_DATE) DP1_UserCalDateInd=0;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_USER_CAL_DATE_IND_ADDR,DP1_UserCalDateInd);
						}
					}
				}
				
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT_C,DP1_Cal_Count_C);
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
					
					DP1_Max = DEFAUT_DP1_MAX;
					DP1_Min = DEFAUT_DP1_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Max,(uint8_t*)DP1_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Min,(uint8_t*)DP1_MINIMUM,4);
				}*/
				
			break;
			
			case DP2CAL_ID:
			
				if(b.FactoryCalibrationOn==1)
				{
					//DP2_Cal_Count=tempshort;
					//DP2_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT,DP2_Cal_Count);
					
					DP2_Cal_Value_F = RealDpressure2*10.0;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_F_ADDR,DP2_Cal_Value_F);
					DP2_Cal_float_Value_F = (float)DP2_Cal_Value_F/10.0;
					
					DP2_Cal_Value_C = 0;
					DP2_Cal_float_Value_C = 0;
					
					su16_dp_sw_factor[1]=0;
					f32_dp_sw_factor[1]=0.0;
					eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_SW_FACT_ADDR+2),su16_dp_sw_factor[1]);
					
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//DP2_Cal_Count_C=tempshort;
					
					DP2_Cal_Value_C = (RealDpressure2 - DP2_Cal_float_Value_F)*10.0;
					DP2_Cal_float_Value_C = (float)DP2_Cal_Value_C/10.0;
				}
				
				//opstr(1,"\r\nDP2_Cal_F:");
				//print_float(1,DP2_Cal_float_Value_F,test,1);
				//opstr(1,"\r\n");
				//
				//opstr(1,"\r\nDP2_Cal_C:");
				//print_float(1,DP2_Cal_float_Value_C,test,1);
				//opstr(1,"\r\n");
			
				//opstr(SrcPort,"\r\nDP2_Cal_float_Value_F:");
				//print_float(SrcPort,DP2_Cal_float_Value_F,test,1);
				//opstr(SrcPort,"\r\n");
				//
				//opstr(SrcPort,"\r\nDP2_Cal_float_Value_C:");
				//print_float(SrcPort,DP2_Cal_float_Value_C,test,1);
				//opstr(SrcPort,"\r\n");
			
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT_C,DP2_Cal_Count_C);
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
					
					DP2_Max = DEFAUT_DP2_MAX;
					DP2_Min = DEFAUT_DP2_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Max,(uint8_t*)DP2_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Min,(uint8_t*)DP2_MINIMUM,4);
				}
				
				/*if(b.FactoryCalibrationOn==1)
				{
					//DP2_Cal_Count=tempshort;
					//DP2_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT,DP2_Cal_Count);
					
					DP2_Cal_Value_F = RealDpressure2*10.0;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_F_ADDR,DP2_Cal_Value_F);
					DP2_Cal_float_Value_F = (float)DP2_Cal_Value_F/10.0;
					
					DP2_Cal_Value_C = 0;
					DP2_Cal_float_Value_C = 0;
									
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)DP2_CAL_DATE_ADDR,12);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[21],(uint8_t*)DP2_CAL_CERT_ADDR,15);
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//DP2_Cal_Count_C=tempshort;
					
					DP2_Cal_Value_C = (RealDpressure2 - DP2_Cal_float_Value_F)*10.0;
					DP2_Cal_float_Value_C = (float)DP2_Cal_Value_C/10.0;
					
					Buffer1[0] = findValue(&RxBuffer[9],2);
					Buffer1[1] = findValue(&RxBuffer[11],2);
					Buffer1[2] = findValue(&RxBuffer[13],2);
					
					if(!Buffer1[0] && !Buffer1[1] && !Buffer1[2])
					{
						if(!DP2_UserCalDateInd) a1=14;
						else a1=DP2_UserCalDateInd-1;
					
						us1 = DP2_USER_CAL_DATE_ADDR + (a1 * 6);
						eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)us1,6);
					
						if(memcmp(&Buffer1[0],&RxBuffer[9],6))
						{
							us1 = DP2_USER_CAL_DATE_ADDR + (DP2_UserCalDateInd * 6);
							eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)us1,6);
						
							DP2_UserCalDateInd++;
							if(DP2_UserCalDateInd>14) DP2_UserCalDateInd=0;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_USER_CAL_DATE_IND_ADDR,DP2_UserCalDateInd);
						}
					}
				}
			
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT_C,DP2_Cal_Count_C);
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
					
					DP2_Max = DEFAUT_DP2_MAX;
					DP2_Min = DEFAUT_DP2_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Max,(uint8_t*)DP2_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Min,(uint8_t*)DP2_MINIMUM,4);
				}*/
				
			break;
			case TMCAL_ID:
				
				if(b.FactoryCalibrationOn==1)
				{
					//TM_Cal_Count=tempshort;
					//TM_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT,TM_Cal_Count);
					
					if(!TM_Unit)
					{
						ss1 = RealtemperatureC*10.0;
						TM_Cal_Value_F = ss1 - tempshort;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
						TM_Cal_float_Value_F = (float)TM_Cal_Value_F/10.0;
					}
					else
					{
						ss1 = RealtemperatureF*10.0;
						TM_Cal_Value_F = ss1 - tempshort;
						TM_Cal_Value_F = ((float)TM_Cal_Value_F * 1.8) + 32.0;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
						
						TM_Cal_Value_F = (TM_Cal_Value_F-320) / 1.8;
						TM_Cal_float_Value_F = (float)TM_Cal_Value_F/10.0;
					}
										
					TM_Cal_Value_C = 0;
					TM_Cal_float_Value_C = 0;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//TM_Cal_Count_C=tempshort;
					
					if(!TM_Unit)
					{
						ss1 = (RealtemperatureC - TM_Cal_float_Value_F)*10.0;
						TM_Cal_Value_C = ss1 - tempshort;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
						TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
					}
					else
					{
						ss1 = (RealtemperatureF - TM_Cal_float_Value_F)*10.0;
						TM_Cal_Value_C = ss1 - tempshort;
						TM_Cal_Value_C = ((float)TM_Cal_Value_C * 1.8) + 32.0;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
						
						TM_Cal_Value_C = (TM_Cal_Value_C-320) / 1.8;
						TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
					}
				}
				
				//opstr(SrcPort,"\r\nTM_Cal_float_Value_F:");
				//print_float(SrcPort,TM_Cal_float_Value_F,test,1);
				//opstr(SrcPort,"\r\n");
				
				//opstr(SrcPort,"\r\nTM_Cal_float_Value_C:");
				//print_float(SrcPort,TM_Cal_float_Value_C,test,1);
				//opstr(SrcPort,"\r\n");
				
				
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT_C,TM_Cal_Count_C);

					TM_Max = DEFAUT_TEMP_C_MAX;
					TM_Min = DEFAUT_TEMP_C_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Max,(uint8_t*)TEMP_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Min,(uint8_t*)TEMP_MINIMUM,4);
				}
				
				/*if(b.FactoryCalibrationOn==1)
				{
					//TM_Cal_Count=tempshort;
					//TM_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT,TM_Cal_Count);
					
					if(!TM_Unit)
					{
						ss1 = RealtemperatureC*10.0;
						TM_Cal_Value_F = ss1 - tempshort;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
						TM_Cal_float_Value_F = (float)TM_Cal_Value_F/10.0;
					}
					else
					{
						ss1 = RealtemperatureF*10.0;
						TM_Cal_Value_F = ss1 - tempshort;
						TM_Cal_Value_F = ((float)TM_Cal_Value_F * 1.8) + 32.0;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
						
						TM_Cal_Value_F = (TM_Cal_Value_F-320) / 1.8;
						TM_Cal_float_Value_F = (float)TM_Cal_Value_F/10.0;
					}
										
					TM_Cal_Value_C = 0;
					TM_Cal_float_Value_C = 0;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
					
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)TM_CAL_DATE_ADDR,12);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[21],(uint8_t*)TM_CAL_CERT_ADDR,15);
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//TM_Cal_Count_C=tempshort;
					
					if(!TM_Unit)
					{
						ss1 = (RealtemperatureC - TM_Cal_float_Value_F)*10.0;
						TM_Cal_Value_C = ss1 - tempshort;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
						TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
					}
					else
					{
						ss1 = (RealtemperatureF - TM_Cal_float_Value_F)*10.0;
						TM_Cal_Value_C = ss1 - tempshort;
						TM_Cal_Value_C = ((float)TM_Cal_Value_C * 1.8) + 32.0;
						eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
						
						TM_Cal_Value_C = (TM_Cal_Value_C-320) / 1.8;
						TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
					}
					
					Buffer1[0] = findValue(&RxBuffer[9],2);
					Buffer1[1] = findValue(&RxBuffer[11],2);
					Buffer1[2] = findValue(&RxBuffer[13],2);
					
					if(!Buffer1[0] && !Buffer1[1] && !Buffer1[2])
					{
						if(!TM_UserCalDateInd) a1=14;
						else a1=TM_UserCalDateInd-1;
					
						us1 = TM_USER_CAL_DATE_ADDR + (a1 * 6);
						eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)us1,6);
					
						if(memcmp(&Buffer1[0],&RxBuffer[9],6))
						{
							us1 = TM_USER_CAL_DATE_ADDR + (TM_UserCalDateInd * 6);
							eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)us1,6);
						
							TM_UserCalDateInd++;
							if(TM_UserCalDateInd>14) TM_UserCalDateInd=0;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_USER_CAL_DATE_IND_ADDR,TM_UserCalDateInd);
						}
					}
				}
				
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT_C,TM_Cal_Count_C);

					TM_Max = DEFAUT_TEMP_C_MAX;
					TM_Min = DEFAUT_TEMP_C_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Max,(uint8_t*)TEMP_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Min,(uint8_t*)TEMP_MINIMUM,4);
				}*/
				
			break;
			case RHCAL_ID:
				
				if(b.FactoryCalibrationOn==1)
				{
					ss1 = RealhumidityRH*10.0;
					RH_Cal_Value_F = ss1 - tempshort;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_F_ADDR,RH_Cal_Value_F);
					RH_Cal_float_Value_F = (float)RH_Cal_Value_F/10.0;
				
					RH_Cal_Value_C = 0;
					RH_Cal_float_Value_C = 0;
				}
				else if(b.CustmerCalibrationOn==1)
				{
					ss1 = (RealhumidityRH - RH_Cal_float_Value_F)*10.0;
					RH_Cal_Value_C = ss1 - tempshort;
					RH_Cal_float_Value_C = (float)RH_Cal_Value_C/10.0;
				}

				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT_C,RH_Cal_Count_C);
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
					
					RH_Max = DEFAUT_RH_MAX;
					RH_Min = DEFAUT_RH_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Max,(uint8_t*)RH_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Min,(uint8_t*)RH_MINIMUM,4);
				}
			
				/*if(b.FactoryCalibrationOn==1)
				{
					//RH_Cal_Count=tempshort;
					//RH_Cal_Count_C=0;
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT,RH_Cal_Count);
					
					ss1 = RealhumidityRH*10.0;
					RH_Cal_Value_F = ss1 - tempshort;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_F_ADDR,RH_Cal_Value_F);
					RH_Cal_float_Value_F = (float)RH_Cal_Value_F/10.0;
					
					RH_Cal_Value_C = 0;
					RH_Cal_float_Value_C = 0;
					
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)RH_CAL_DATE_ADDR,12);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[21],(uint8_t*)RH_CAL_CERT_ADDR,15);
				}
				else if(b.CustmerCalibrationOn==1)
				{
					//RH_Cal_Count_C=tempshort;
					
					ss1 = (RealhumidityRH - RH_Cal_float_Value_F)*10.0;
					RH_Cal_Value_C = ss1 - tempshort;
					RH_Cal_float_Value_C = (float)RH_Cal_Value_C/10.0;
					
					Buffer1[0] = findValue(&RxBuffer[9],2);
					Buffer1[1] = findValue(&RxBuffer[11],2);
					Buffer1[2] = findValue(&RxBuffer[13],2);
					
					if(!Buffer1[0] && !Buffer1[1] && !Buffer1[2])
					{
						if(!RH_UserCalDateInd) a1=14;
						else a1=RH_UserCalDateInd-1;
					
						us1 = RH_USER_CAL_DATE_ADDR + (a1 * 6);
						eeprom_read_block((uint8_t*)&Buffer1[0],(uint8_t*)us1,6);
					
						if(memcmp(&Buffer1[0],&RxBuffer[9],6))
						{
							us1 = RH_USER_CAL_DATE_ADDR + (RH_UserCalDateInd * 6);
							eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)us1,6);

							RH_UserCalDateInd++;
							if(RH_UserCalDateInd>14) RH_UserCalDateInd=0;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RH_USER_CAL_DATE_IND_ADDR,RH_UserCalDateInd);
						}
					}
				}
				
				if((b.FactoryCalibrationOn==1) || (b.CustmerCalibrationOn==1))
				{
					PCCalibrationTimer=60;
					
					//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT_C,RH_Cal_Count_C);
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
					
					RH_Max = DEFAUT_RH_MAX;
					RH_Min = DEFAUT_RH_MIN;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Max,(uint8_t*)RH_MAXIMUM,4);
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Min,(uint8_t*)RH_MINIMUM,4);
				}*/
				
			break;
			case TMUNT_ID:
				if((tempshort==0) || (tempshort==1))
				{
					if(TM_Unit!=tempshort)
					{
						TM_Unit=tempshort;
						TMUnitChange();
					}
				}
			break;
			case UBRT_ID:
				if((tempshort>=3) && (tempshort<=9))
				{
					UART_BaudRate=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_BAUDRATE,UART_BaudRate);
				}
			break;
			case UDBT_ID:
				if((tempshort>=0) && (tempshort<=3))
				{
					UART_DataBits=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_DATBITS,UART_DataBits);
				}
			break;
			case UPRT_ID:
				if((tempshort>=0) && (tempshort<=2))
				{
					UART_Parity=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_PARITY,UART_Parity);
				}
			break;
			case USTB_ID:
				if((tempshort==0) || (tempshort==1))
				{
					UART_StopBit=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_STOPBIT,UART_StopBit);
				}
			break;
			case BACKLIT_ID:
				if((tempshort==0) || (tempshort==1))
				{
					gu8_BackLitOnOff=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BACKLIT_ON_OFF_ADDR,gu8_BackLitOnOff);
					
					if(!gu8_BackLitOnOff)
					{
						//WHITE_BLIT_OFF;
					}
					else
					{
						//WHITE_BLIT_ON;
					}
				}
			break;
			
			case PUT_DFU_ID:
				if(tempshort==DFU_PASSWORD)
				{
					lcd.Sym_DOOR = 0;
					lcd.Sym_DOOR_SYM = 0;
					lcd.Sym_DOOR_ALM = 0;
					lcd.Sym_LOGO = 0;
					lcd.Sym_MIN = 0;
					lcd.Sym_MAX = 0;
					lcd.Sym_MEAN = 0;
					lcd.Sym_SET = 0;
					lcd.Sym_ID = 0;
					lcd.Sym_ACK = 0;
					lcd.Sym_RTC_AM = 0;
					lcd.Sym_RTC_PM = 0;
					lcd.Sym_RTC_BC1 = 0;
					lcd.Sym_RTC_COL = 0;

					lcd.Sym_DP_LOGO = 0;
					lcd.Sym_DP_LOGO_ALM = 0;
					lcd.Sym_DP_UNIT = 0;
					lcd.Sym_DP_MIN = 0;
					lcd.Sym_DP_MIN_ALM = 0;
					
					lcd.Sym_RH_LOGO = 0;
					lcd.Sym_RH_LOGO_ALM = 0;
					lcd.Sym_RH_UNIT = 0;
					lcd.Sym_RH_MIN = 0;
					lcd.Sym_RH_MIN_ALM = 0;
					
					lcd.Sym_TM_LOGO = 0;
					lcd.Sym_TM_LOGO_ALM = 0;
					lcd.Sym_TM_UNIT_C = 0;
					lcd.Sym_TM_UNIT_F = 0;
					lcd.Sym_TM_MIN = 0;
					lcd.Sym_TM_MIN_ALM = 0;
					
					for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
					data[4] = D;
					data[5] = F;
					data[6] = U;
					
					disp_value();
					
					eeprom_busy_wait();  	eeprom_write_word ((unsigned int*)DFU_NUMBER_LOGIC,0xABCD);
					
					gu8_restartTimer = 5;
					
					//CPU_CCP=0xD8;
					//RST_CTRL=RST_SWRST_bm;
					//wdt_enable(WDTO_2S);
					//b.resetDevice=1;
				}
			break;
			
			case MENB_ID:
				if((tempshort==0) || (tempshort==1))
				{
					gu8_masterEnable=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)MASTER_ENABLE_ADDR,gu8_masterEnable);
				}
			break;
			case DOOR_SENSE_POLARITY_ID:
				if((tempshort==0) || (tempshort==1))
				{
					gu8_doorSensingPolarity=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DOOR_SENSE_POLARITY_ADDR,gu8_doorSensingPolarity);
				}
			break;
			case DOOR_SENSE_TIME_ID:
				if(tempshort<=250)
				{
					gu8_doorSensingTime=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DOOR_SENSE_TIME_ADDR,gu8_doorSensingTime);
				}
			break;
			case LCD_BRIGHT_CNT_ID:
				if(tempshort<=63)
				{
					gu8_LCDBrigthnessCnt=tempshort;
					TM1680WriteCommand(BRIGHTNESS | gu8_LCDBrigthnessCnt);
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LCD_BRIGHT_CNT_ADDR,gu8_LCDBrigthnessCnt);
				}
			break;
			case AUTO_SENT_INTERVAL_ID:
				if((tempshort!=0) && (tempshort <= 240))
				{
					gu8_AutoSentInterval=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)AUTO_SENT_INTERVAL_ADDR,gu8_AutoSentInterval);
				}
			break;
			case DEVICES_IN_GROUP_ID:
				if((tempshort>=2) && (tempshort <= 100))
				{
					gu8_DeviceInGroup=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICES_IN_GROUP_ADDR,gu8_DeviceInGroup);
				}
			break;
			case XBEE_RST_INTERVAL_ID:
				if(tempshort <= 1440)
				{
					gu16_XbeeRstInterval=tempshort;
					gu32_triggerXbeeResetTimer = (unsigned long)gu16_XbeeRstInterval*60;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)XBEE_RST_INTERVAL_ADDR,gu16_XbeeRstInterval);
				}
			break;
			case DP1_ALM_SENSE_TIME_ID:
				if(tempshort<=250)
				{
					gu8_Dp1AlarmSensingTime=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_ALM_SENSE_TIME_ADDR,gu8_Dp1AlarmSensingTime);
				}
			break;
			case DP2_ALM_SENSE_TIME_ID:
				if(tempshort<=250)
				{
					gu8_Dp2AlarmSensingTime=tempshort;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_ALM_SENSE_TIME_ADDR,gu8_Dp2AlarmSensingTime);
				}
			break;
			case XBEE_MAC_ADDR_ID:
				
				tempchar=RxBuffer[4]-'1';
				
				if(tempchar<NO_OF_XBEE_MAC)
				{			
					memcpy(&gu8arr_XbeeMac[tempchar][0],&RxBuffer[5],XBEE_MAC_SIZE);
					eeprom_busy_wait();  eeprom_write_block(&gu8arr_XbeeMac[tempchar][0],(uint8_t*)(XBEE_MAC_ADDR+(tempchar*XBEE_MAC_SIZE)),XBEE_MAC_SIZE);
				}
				
				if(tempchar==1)
				{
					gu8_Mac2ValidTimer = 60;
				}
				
			break;
			case ACK_TIMER_ID:
				AckTimer=tempshort;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)ACK_TIMER,AckTimer);
			break;
			case ACK_PW_ID:
				
				if(RxBuffer[4]<=NO_OF_ACKPWD)
				{
					AckPwdInd=RxBuffer[4];
					tempchar=RxBuffer[4]-1;
					AckPwd[tempchar] = tempshort;
					
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)(ACK_PASSWORD+(2*tempchar)),AckPwd[tempchar]);
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)ACK_PWD_IND,AckPwdInd);
				}
										
			break;
			case ALM_ACK_ID:

				if(AckPwdInd)
				{		
					a1=RxBuffer[4]-1;
									
					if(AckPwd[a1]==tempshort)
					{
						AlarmAckTimer=(unsigned long)AckTimer * 60;
							
						FillRamBuffer(ALM_ACK_LOG,RxBuffer[4],AckPwd[a1]);
						
						break;
					}
				}
				else
				{
					if(tempshort==FACT_ACK_PWD) 
					{
						AlarmAckTimer=(unsigned long)AckTimer * 60;	
						
						FillRamBuffer(ALM_ACK_LOG,0,FACT_ACK_PWD);
					}
				}
			break;
			case SRNO_ID:
				memcpy(gu8ar_SrNumber,&RxBuffer[4],16);
				eeprom_busy_wait();  eeprom_write_block(gu8ar_SrNumber,(uint8_t*)DEVICE_SR_NO,16);
				gu32_SrNumber = ascii2hex(&gu8ar_SrNumber[8],8);
			break;	
			case BRDSTP_ID:
				b.brodcastEnb=0;
				StartBroadcastTimer=0;
				gu8_broadcast=0;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BROADCAST_ENB_ADDR,gu8_broadcast);
			break;
			case BRDSTR_ID:
				b.brodcastEnb=1;
				tempshort=findValue(&RxBuffer[4],RxInd-6);
				StartBroadcastTimer=(unsigned long)tempshort*60; 
				gu8_broadcast=1;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BROADCAST_ENB_ADDR,gu8_broadcast);
			break;
			default:
				b.paraIdNotValid=1;
			break;
		}
		
		sei();			//Global Interrupt Enable
		
		
		TxBuffer[0]=0xFD;
		TxBuffer[1]=RxBuffer[1];
		TxBuffer[2]=RxBuffer[2];
		TxBuffer[3]=0x00;
		if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
		if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
		if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
		if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
		
		for(j=4;j<RxInd;j++)TxBuffer[j]=RxBuffer[j-1];
		
		TxBuffer[RxInd-1]=CalCRC(&TxBuffer[1],RxInd-2);
		TxBuffer[RxInd]=0xFC;
		
		SetTxmode(SrcPort,TxBuffer,RxInd+1);
		
		if((RxBuffer[3]==UBRT_ID)||(RxBuffer[3]==UDBT_ID)||(RxBuffer[3]==UPRT_ID)||(RxBuffer[3]==USTB_ID))
		{
			//Init_USARTC0(UART_BaudRate,UART_DataBits,UART_Parity,UART_StopBit);
			InitUSART(&USARTC0, &PORTC, PIN2_bm, PIN3_bm, UART_BaudRate,UART_DataBits,UART_Parity,UART_StopBit);
		}	
		
		if(RxBuffer[3]==XBEE_MAC_ADDR_ID)
		{
			if(gu8_Mac2ValidTimer) SetMAC2Xbee(&gu8arr_XbeeMac[1][0],0);
		}
		
	}
	else if(RxBuffer[2]==PARA_READ_CMD)
	{
		#ifdef DEBUG_RCV_CMD
			opstr(0,"\r\nRead Cmd");
		#endif
		
		tempshort = 0;
		
		switch(RxBuffer[3])
		{
			case SEC_ID:		tempshort = rtc.second;					break;
			case MN_ID:			tempshort = rtc.minute;					break;
			case HR_ID:			tempshort = rtc.hour;					break;
			case DT_ID:			tempshort = rtc.day;					break;
			case MNTH_ID:		tempshort = rtc.month;					break;
			case YR_ID:			tempshort = (2000 + rtc.year);			break;
			case DP1UAON_ID:	tempshort = DP1_Upper_Alm_ON*10;		break;
			case DP1UAOFF_ID:	tempshort = DP1_Upper_Alm_OFF*10;		break;
			case DP1LAON_ID:	tempshort = DP1_Lower_Alm_ON*10;		break;
			case DP1LAOFF_ID:	tempshort = DP1_Lower_Alm_OFF*10;		break;
			case DP2UAON_ID:	tempshort = DP2_Upper_Alm_ON*10;		break;
			case DP2UAOFF_ID:	tempshort = DP2_Upper_Alm_OFF*10;		break;
			case DP2LAON_ID:	tempshort = DP2_Lower_Alm_ON*10;		break;
			case DP2LAOFF_ID:	tempshort = DP2_Lower_Alm_OFF*10;		break;
			case TMUAON_ID:		tempshort = TM_Upper_Alm_ON*10;			break;
			case TMUAOFF_ID:	tempshort = TM_Upper_Alm_OFF*10;		break;
			case TMLAON_ID:		tempshort = TM_Lower_Alm_ON*10;			break;
			case TMLAOFF_ID:	tempshort = TM_Lower_Alm_OFF*10;		break;
			case RHUAON_ID:		tempshort = RH_Upper_Alm_ON*10;			break;
			case RHUAOFF_ID:	tempshort = RH_Upper_Alm_OFF*10;		break;
			case RHLAON_ID:		tempshort = RH_Lower_Alm_ON*10;			break;
			case RHLAOFF_ID:	tempshort = RH_Lower_Alm_OFF*10;		break;
			case LOGINTVAL_ID:	tempshort = LogInterval;				break;
			case DVCID_ID:		tempshort = DeviceID;					break;
			case BZRON_ID:		tempshort = Buzzer_ON_Time;				break;
			case SCROLL_TIME_ID:		tempshort = ParaScrollTime;		break;
			case TM_RH_SCAN_TIME_ID:	tempshort = gu8_TM_RH_ScanTime;	break;
			case BZROFF_ID:		tempshort = Buzzer_OFF_Time;			break;
			case DP1MIN_ID:		tempshort = DP1_Min*100;				break;
			case DP1MAX_ID:		tempshort = DP1_Max*100;				break;
			case DP2MIN_ID:		tempshort = DP2_Min*100;				break;
			case DP2MAX_ID:		tempshort = DP2_Max*100;				break;
			case TMMIN_ID:
					if(!TM_Unit)
					{
						tempfloat = TM_Min;
					}
					else
					{
						tempfloat = (TM_Min * 1.8) + 32.0;
					}		
					tempshort = tempfloat*100;					
				break;
			case TMMAX_ID:		
					if(!TM_Unit)
					{
						tempfloat = TM_Max;
					}
					else
					{
						tempfloat = (TM_Max * 1.8) + 32.0;
					}
					tempshort = tempfloat*100;				
				break;
			case RHMIN_ID:		tempshort = RH_Min*100;					break;
			case RHMAX_ID:		tempshort = RH_Max*100;					break;
			case TMUNT_ID:		tempshort = TM_Unit;					break;
			case DP_SW_FACT_ID:
			
				index=RxBuffer[4]-'0';
				if(index<MAX_SUPPORTED_DP)
				{	
					tempshort = su16_dp_sw_factor[index];
				}
				
			break;
			
			case DP_LIMIT_ID:
			
				index=RxBuffer[4]-'0';
				if(index<MAX_SUPPORTED_DP)
				{
					tempshort = u16_dp_limit[index];
				}
			
			break;
			
			case DP1CAL_ID:		
					if(b.FactoryCalibrationOn==1)
					{
						PCCalibrationTimer=60;
						tempshort = DP1_Cal_Value_F;
					}
					else if(b.CustmerCalibrationOn==1)
					{
						PCCalibrationTimer=60;
						tempshort = DP1_Cal_Value_C;
					}		
					else
					{
						tempshort = 0;
					}		
					break;
			case DP2CAL_ID:
				if(b.FactoryCalibrationOn==1)
				{
					PCCalibrationTimer=60;
					tempshort = DP2_Cal_Value_F;
				}
				else if(b.CustmerCalibrationOn==1)
				{
					PCCalibrationTimer=60;
					tempshort = DP2_Cal_Value_C;
				}
				else
				{
					tempshort = 0;
				}
			break;		
			case TMCAL_ID:
				if(b.FactoryCalibrationOn==1)
				{
					PCCalibrationTimer=60;
					tempshort = TM_Cal_Value_F;
				}
				else if(b.CustmerCalibrationOn==1)
				{
					PCCalibrationTimer=60;
					tempshort = TM_Cal_Value_C;
				}
				else
				{
					tempshort = 0;
				}
			break;
			case RHCAL_ID:
				if(b.FactoryCalibrationOn==1)
				{
					PCCalibrationTimer=60;
					tempshort = RH_Cal_Value_F;
				}
				else if(b.CustmerCalibrationOn==1)
				{
					PCCalibrationTimer=60;
					tempshort = RH_Cal_Value_C;
				}
				else
				{
					tempshort = 0;
				}
			break;
			case SFVER_ID:		tempshort = SOFT_VER;					break;
			case ACK_TIMER_ID:	tempshort = AckTimer;					break;
			case CPWD_ID:		tempshort = CustPassword;				break;
			case FCPWD_ID:		tempshort = FactCustPassword;			break;
			case SET_DPARA_PWD_ID:	
				
				us2=0;	//password
				us1 = RxBuffer[4]-'0';		us1 *= 1000;			us2 += us1;		us1 = 0;
				us1 = RxBuffer[5]-'0';		us1 *= 100;				us2 += us1;		us1 = 0;
				us1 = RxBuffer[6]-'0';		us1 *= 10;				us2 += us1;		us1 = 0;
				us1 = RxBuffer[7]-'0';								us2 += us1;		us1 = 0;
				
				if(us2 == FACTORY_PARASET_PWD)
				{
					tempshort = gu16_parameterWord;			
				}
				
			break;
			
			case ACK_PW_ID:		tempshort = AckPwd[RxBuffer[4]-1];		break;
			case BATPER_ID:		tempshort = BatteryPercentage;			break;
			case UBRT_ID:		tempshort = UART_BaudRate;				break;
			case UDBT_ID:		tempshort = UART_DataBits;				break;
			case UPRT_ID:		tempshort = UART_Parity;				break;
			case USTB_ID:		tempshort = UART_StopBit;				break;
			case BACKLIT_ID:	tempshort = gu8_BackLitOnOff;			break;
			case MENB_ID:		tempshort = gu8_masterEnable;			break;
			case DOOR_SENSE_POLARITY_ID:	tempshort = gu8_doorSensingPolarity;	break;
			case DOOR_SENSE_TIME_ID:		tempshort = gu8_doorSensingTime;		break;
			case LCD_BRIGHT_CNT_ID:			tempshort = gu8_LCDBrigthnessCnt;		break;
			case AUTO_SENT_INTERVAL_ID:			tempshort = gu8_AutoSentInterval;		break;
			case DEVICES_IN_GROUP_ID:			tempshort = gu8_DeviceInGroup;			break;
			case XBEE_RST_INTERVAL_ID:			tempshort = gu16_XbeeRstInterval;		break;
			case DP1_ALM_SENSE_TIME_ID:			tempshort = gu8_Dp1AlarmSensingTime;	break;
			case DP2_ALM_SENSE_TIME_ID:			tempshort = gu8_Dp2AlarmSensingTime;	break;
			case SRNO_ID:												break;		
			case XBEE_SELF_MAC_ADDR_ID:									break;
			case XBEE_MAC_ADDR_ID:										break;
			case RAM_ALL_ID:											break;
			case RAM_IND_ID:											break;
			case FLASH24_IND_ID:										break;
			case MINMAXMEAN_IND_ID:										break;
			case MEAN_HR_ID:											break;
			case REALTIME_VAL_ID:										break;
			case RDLG_DT_ID:											break;
			case RDLG_CNT_ID:											break;
			case DATETIME_ID:											break;
			case CORR_RTC_DATA_ID:										break;
			case FLASH24_CUR_IND_ID:	tempshort = CurrentLog24Ind;	break;
			default:			b.paraIdNotValid=1;						break;
		}
			
		if(RxBuffer[3]==DATETIME_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			chartostr(rtc.day,&TxBuffer[5],2);
			chartostr(rtc.month,&TxBuffer[7],2);
			chartostr(rtc.year,&TxBuffer[9],2);
			chartostr(rtc.hour,&TxBuffer[11],2);
			chartostr(rtc.minute,&TxBuffer[13],2);
			chartostr(rtc.second,&TxBuffer[15],2);
			
			TxBuffer[17]=CalCRC(&TxBuffer[1],16);
			TxBuffer[18]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,19);
		}
		else if(RxBuffer[3]==ACK_PW_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			
			tempchar = fillValue(&TxBuffer[5],tempshort);
			
			TxBuffer[5+tempchar]=CalCRC(&TxBuffer[1],4+tempchar);	
			TxBuffer[6+tempchar]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,7+tempchar);
		}
		else if(RxBuffer[3]==CORR_RTC_DATA_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			TxBuffer[5]=RTCCorruptDataInd;
			a2=6;
			for(a1=0;a1<RTCCorruptDataInd;a1++)
			{
				us1 = CORRUPT_RTC_DATA_ADDR + (a1 * 7);
				eeprom_read_block((uint8_t*)&TxBuffer[a2],(uint8_t*)us1,7);
				a2+=7;
			}
			TxBuffer[a2]=CalCRC(&TxBuffer[1],a2-1);	
			TxBuffer[a2+1]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,a2+2);
		}
		else if(RxBuffer[3]==SRNO_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			
			memcpy(&TxBuffer[5],gu8ar_SrNumber,16);
			
			TxBuffer[21]=CalCRC(&TxBuffer[1],20);	
			TxBuffer[22]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,23);
		}
		else if(RxBuffer[3]==XBEE_MAC_ADDR_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			TxBuffer[5]=RxBuffer[4];
			
			tempchar = RxBuffer[4]-'1';
			
			memcpy(&TxBuffer[6],&gu8arr_XbeeMac[tempchar][0],XBEE_MAC_SIZE);
			
			TxBuffer[22]=CalCRC(&TxBuffer[1],21);	
			TxBuffer[23]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,24);
		}
		else if(RxBuffer[3]==XBEE_SELF_MAC_ADDR_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			
			memcpy(&TxBuffer[5],gu8arr_XbeeSelfMac,XBEE_MAC_SIZE);
			
			TxBuffer[21]=CalCRC(&TxBuffer[1],20);	
			TxBuffer[22]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,23);
		}
		else if(RxBuffer[3]==RAM_ALL_ID)
		{
			RAMBuffer[0]=0xFD;
			RAMBuffer[1]=RxBuffer[1];
			RAMBuffer[2]=RxBuffer[2];
			RAMBuffer[3]=0x00;
			if(b.paraIdNotValid) 	RAMBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			RAMBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			RAMBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		RAMBuffer[3] |= RH_TEMP_FAULTY;
			RAMBuffer[4]=RxBuffer[3];

			//Send CRC and 0xFC
			RAMBuffer[1505]=CalCRC(&RAMBuffer[1],1504);	
			RAMBuffer[1506]=0xFC;
			
			SetTxmode(SrcPort,RAMBuffer,1507);
		}
		else if(RxBuffer[3]==RAM_IND_ID)
		{
			flash24_StartInd = RxBuffer[4];
			flash24_EndInd = RxBuffer[5];
						
			b.RamReadCmd=1;
			//logtransfer=flash24_StartInd;
			b.logtransferStart=0;
				
			//if(!SrcPort)glbSrcPort=0;
			//else glbSrcPort=2;
			glbSrcPort=SrcPort;
			
			if(RAMBufferLog)
			{
				flash24_StartInd=RAMBufferLog-1;
			}
			else
			{
				flash24_StartInd=29;
			}
			
			NoOf24Log=flash24_EndInd;
				
			/*
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP_NC) 			TxBuffer[3] |= DP_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			TxBuffer[5]=RxBuffer[4];

			tempshort = (RxBuffer[4] * LOG_SIZE) + RAM_FILL_START;
			memcpy(&TxBuffer[6],&RAMBuffer[tempshort],LOG_SIZE);
			
			TxBuffer[56]=CalCRC(&TxBuffer[1],55);
			TxBuffer[57]=0xFC;
			
			SendToUART(&TxBuffer[0],58);
			*/
		}
		else if(RxBuffer[3]==FLASH24_IND_ID)
		{
			flash24_StartInd=0;
			us1 = RxBuffer[4]-'0';		us1 *= 1000;			flash24_StartInd += us1;		us1 = 0;
			us1 = RxBuffer[5]-'0';		us1 *= 100;				flash24_StartInd += us1;		us1 = 0;
			us1 = RxBuffer[6]-'0';		us1 *= 10;				flash24_StartInd += us1;		us1 = 0;
			us1 = RxBuffer[7]-'0';								flash24_StartInd += us1;		us1 = 0;
			
			flash24_EndInd=0;
			us1 = RxBuffer[8]-'0';		us1 *= 1000;			flash24_EndInd += us1;		us1 = 0;
			us1 = RxBuffer[9]-'0';		us1 *= 100;				flash24_EndInd += us1;		us1 = 0;
			us1 = RxBuffer[10]-'0';		us1 *= 10;				flash24_EndInd += us1;		us1 = 0;
			us1 = RxBuffer[11]-'0';								flash24_EndInd += us1;		us1 = 0;
			
			b.Flash24ReadCmd=1;
			//logtransfer=flash24_StartInd;
			b.logtransferStart=0;	
			
			
			NoOf24Log=flash24_EndInd;
			//if(!SrcPort)glbSrcPort=0;
			//else glbSrcPort=2;	
			glbSrcPort=SrcPort;				
		}
		else if(RxBuffer[3]==REALTIME_VAL_ID)
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			
			memcpy(&TxBuffer[5],(uint8_t*)&ep.currentEpochTime,4);
			
			TxBuffer[9]=0;
			if(b.DP1_NC) 	TxBuffer[9] |= DP1_FAULTY;
			if(b.DP2_NC) 	TxBuffer[9] |= DP2_FAULTY;
			if(b.RH_TEMP_NC)TxBuffer[9] |= RH_TEMP_FAULTY;
			
			memcpy(&TxBuffer[10],(uint8_t*)&Dpressure1,4);
			memcpy(&TxBuffer[14],(uint8_t*)&Dpressure2,4);
			memcpy(&TxBuffer[18],(uint8_t*)&temperatureC,4);
			memcpy(&TxBuffer[22],(uint8_t*)&humidityRH,4);
			memcpy(&TxBuffer[26],(uint8_t*)&DP1_Min,4);
			memcpy(&TxBuffer[30],(uint8_t*)&DP1_Max,4);
			memcpy(&TxBuffer[34],(uint8_t*)&DP2_Min,4);
			memcpy(&TxBuffer[38],(uint8_t*)&DP2_Max,4);
			memcpy(&TxBuffer[42],(uint8_t*)&TM_Min,4);
			memcpy(&TxBuffer[46],(uint8_t*)&TM_Max,4);
			memcpy(&TxBuffer[50],(uint8_t*)&RH_Min,4);
			memcpy(&TxBuffer[54],(uint8_t*)&RH_Max,4);
			
			if(gu16_parameterWord & ENABLE_DP1)
			{
				if(!DP1_Alrm_ON)
				{
					TxBuffer[58] = 0;
				}
				else
				{
					if(DP1_Alrm_ON==UPPER_ALARM) TxBuffer[58] = 1;
					else						  TxBuffer[58] = 2;
				}
			}
			else
			{
				TxBuffer[58] = 0;
			}
			
			if(gu16_parameterWord & ENABLE_DP2)
			{
				if(!DP2_Alrm_ON)
				{
					TxBuffer[59] = 0;
				}
				else
				{
					if(DP2_Alrm_ON==UPPER_ALARM) TxBuffer[59] = 1;
					else						  TxBuffer[59] = 2;
				}
			}
			else
			{
				TxBuffer[59] = 0;
			}

			if(gu16_parameterWord & ENABLE_TEMP)
			{
				if(!TM_Alrm_ON)
				{
					TxBuffer[60] = 0;
				}
				else
				{
					if(TM_Alrm_ON==UPPER_ALARM) TxBuffer[60] = 1;
					else						  TxBuffer[60] = 2;
				}
			}
			else
			{
				TxBuffer[60] = 0;
			}

			
			if(gu16_parameterWord & ENABLE_RH)
			{
				if(!RH_Alrm_ON)
				{
					TxBuffer[61] = 0;
				}
				else
				{
					if(RH_Alrm_ON==UPPER_ALARM) TxBuffer[61] = 1;
					else						  TxBuffer[61] = 2;
				}
			}
			else
			{
				TxBuffer[61] = 0;
			}

			TxBuffer[62]=CalCRC(&TxBuffer[1],61);
			TxBuffer[63]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,64);
		}
		else if(RxBuffer[3]==RDLG_DT_ID)
		{
			cli();			//Global Interrupt Disable

			rtc1.second = RxBuffer[9];		//Second
			rtc1.minute = RxBuffer[8];		//Minute
			rtc1.hour	= RxBuffer[7];		//Hour
			rtc1.day	= RxBuffer[4];		//Date
			rtc1.month	= RxBuffer[5];		//Month
			rtc1.year	= RxBuffer[6] + 2000;	//Year
			
			StartEpochTime = get_epoch_time(rtc1);
			
			rtc1.second = RxBuffer[15];		//Second
			rtc1.minute = RxBuffer[14];		//Minute
			rtc1.hour	= RxBuffer[13];		//Hour
			rtc1.day	= RxBuffer[10];		//Date
			rtc1.month	= RxBuffer[11];		//Month
			rtc1.year	= RxBuffer[12] + 2000;	//Year
			
			EndEpochTime = get_epoch_time(rtc1);

			#ifdef DEBUG_RCV_CMD
			
				print_Hex(0,StartEpochTime >> 24);
				print_Hex(0,StartEpochTime >> 16);
				print_Hex(0,StartEpochTime >> 8);
				print_Hex(0,StartEpochTime);
				opchar(0,' ');
				print_Hex(0,EndEpochTime >> 24);
				print_Hex(0,EndEpochTime >> 16);
				print_Hex(0,EndEpochTime >> 8);
				print_Hex(0,EndEpochTime);
				opstr(0,"\r\n");
					
				opstr(0,"\r\nRec Epoch:");
				print_short(0,StartEpochTime,test,10);		opstr(0,"  ");
				print_short(0,EndEpochTime,test,10);		opstr(0,"\r\n");
				
				print_Hex(0,StartEpochTime >> 24);
				print_Hex(0,StartEpochTime >> 16);
				print_Hex(0,StartEpochTime >> 8);
				print_Hex(0,StartEpochTime);
				opchar(0,' ');
				print_Hex(0,EndEpochTime >> 24);
				print_Hex(0,EndEpochTime >> 16);
				print_Hex(0,EndEpochTime >> 8);
				print_Hex(0,EndEpochTime);
				opstr(0,"\r\n");
			#endif
			
			#ifdef DEBUG_RCV_CMD
				opstr(0,"Total Log:");
				print_short(0,TotalLog,test,10);		opstr(0,"\r\n");
			#endif
			
			sei();
		}
		else if((RxBuffer[3]==DP1CAL_ID) || (RxBuffer[3]==DP2CAL_ID) || (RxBuffer[3]==TMCAL_ID) || (RxBuffer[3]==RHCAL_ID))
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			
			a1=0;
			if(tempshort<0)
			{
				a1=1;
				tempshort *= (-1);
			}
			chartostr(tempshort,&TxBuffer[5],5);	
			if(a1==1)TxBuffer[5] = '-';
			
			if(b.FactoryCalibrationOn)
			{
				switch(RxBuffer[3])
				{
					case DP1CAL_ID:			us1=DP1_CAL_DATE_ADDR;	us2=DP1_CAL_CERT_ADDR;			break;
					case DP2CAL_ID:			us1=DP2_CAL_DATE_ADDR;	us2=DP2_CAL_CERT_ADDR;			break;
					case TMCAL_ID:			us1=TM_CAL_DATE_ADDR;	us2=TM_CAL_CERT_ADDR;			break;
					case RHCAL_ID:			us1=RH_CAL_DATE_ADDR;	us2=RH_CAL_CERT_ADDR;			break;
				}
				
				eeprom_read_block((uint8_t*)&TxBuffer[10],(uint8_t*)us1,12);
				eeprom_read_block((uint8_t*)&TxBuffer[22],(uint8_t*)us2,15);
				
				TxBuffer[37]=CalCRC(&TxBuffer[1],36);
				TxBuffer[38]=0xFC;
				
				SetTxmode(SrcPort,TxBuffer,39);
			}
			else if(b.CustmerCalibrationOn)
			{
				switch(RxBuffer[3])
				{
					case DP1CAL_ID:			us1=DP1_USER_CAL_DATE_ADDR;				break;
					case DP2CAL_ID:			us1=DP2_USER_CAL_DATE_ADDR;				break;
					case TMCAL_ID:			us1=TM_USER_CAL_DATE_ADDR;				break;
					case RHCAL_ID:			us1=RH_USER_CAL_DATE_ADDR;				break;
				}
				
				eeprom_read_block((uint8_t*)&TxBuffer[10],(uint8_t*)us1,60);
				
				TxBuffer[70]=CalCRC(&TxBuffer[1],69);
				TxBuffer[71]=0xFC;
				
				SetTxmode(SrcPort,TxBuffer,72);
			}
		}
		else
		{
			TxBuffer[0]=0xFD;
			TxBuffer[1]=RxBuffer[1];
			TxBuffer[2]=RxBuffer[2];
			TxBuffer[3]=0x00;
			if(b.paraIdNotValid) 	TxBuffer[3] |= INVALID_PARA;
			if(b.DP1_NC) 			TxBuffer[3] |= DP1_FAULTY;
			if(b.DP2_NC) 			TxBuffer[3] |= DP2_FAULTY;
			if(b.RH_TEMP_NC) 		TxBuffer[3] |= RH_TEMP_FAULTY;
			TxBuffer[4]=RxBuffer[3];
			tempchar = fillValue(&TxBuffer[5],tempshort);
			
			TxBuffer[5+tempchar]=CalCRC(&TxBuffer[1],4+tempchar);	
			TxBuffer[6+tempchar]=0xFC;
			
			SetTxmode(SrcPort,TxBuffer,7+tempchar);
		}
	}
	else
	{
		//If Invalid Command received
		TxBuffer[0]=0xFD;
		TxBuffer[1]=RxBuffer[1];
		TxBuffer[2]=RxBuffer[2];
		TxBuffer[3]=INVALID_CMD;
		TxBuffer[4]=CalCRC(&TxBuffer[1],3);	
		TxBuffer[5]=0xFC;
		
		SetTxmode(SrcPort,TxBuffer,6);
	}

	b.msgRcvOK=0;
	RxInd=0;
}

void SetTxmode(uint8_t txmode,uint8_t *buffer,unsigned short bytes)
{
	if(!txmode)
	{
		_delay_ms(200);
		
		RS485_TX0_ENB;
		DMA_CH0_TRFCNT = bytes;
		DMA_CH0_SRCADDR0 = (unsigned short)buffer;
		DMA_CH0_SRCADDR1 = (unsigned short)buffer>>8;
		DMA_CH0_TRIGSRC = DMA_CH_TRIGSRC_USARTC0_DRE_gc;
		DMA_CH0_DESTADDR0 = (unsigned short)&USARTC0.DATA;
		DMA_CH0_DESTADDR1 = (unsigned short)&USARTC0.DATA >> 8;
		DMA_CH0_CTRLA |= DMA_ENABLE_bm | DMA_CH_SINGLE_bm;
		DMA_CH0_CTRLA |= DMA_CH_TRFREQ_bm;
	}
	else if(txmode==1)
	{
		_delay_ms(200);
		
		RS485_TX1_ENB;
		
		DMA_CH0_TRFCNT = bytes;
		DMA_CH0_SRCADDR0 = (unsigned short)buffer;
		DMA_CH0_SRCADDR1 = (unsigned short)buffer>>8;
		DMA_CH0_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_DRE_gc;
		DMA_CH0_DESTADDR0 = (unsigned short)&USARTC1.DATA;
		DMA_CH0_DESTADDR1 = (unsigned short)&USARTC1.DATA >> 8;
		DMA_CH0_CTRLA |= DMA_ENABLE_bm | DMA_CH_SINGLE_bm;
		DMA_CH0_CTRLA |= DMA_CH_TRFREQ_bm;
	}
	else
	{
		SendToUART(txmode,buffer,bytes);
	}
}

short findValue(uint8_t *ptr,uint8_t NoOfDigit)
{
	unsigned short Value=0,value1=1;
	uint8_t minus=0;
	
	if(*ptr=='-') 
	{
		ptr += (NoOfDigit-1);
		NoOfDigit--;
		minus=1;
	}
	else
	{
		ptr += (NoOfDigit-1);
	}
	
	
	while(NoOfDigit)
	{
		//*ptr -= '0';
		Value += ((unsigned short)(*ptr - '0') * value1);
		
		ptr--;
		NoOfDigit--;
		
		value1 *= 10;
	}
	
	if(minus) Value *= (-1);
	
	return Value;
}

uint8_t fillValue(uint8_t *ptr,long value)
{
	uint8_t x,k,minus=0;
	unsigned long temppow,tempshort;
	
	x=1;
	temppow=10;
	
	
	if(value<0)
	{
		value *= (-1);
		*ptr = '-';
		ptr++;
		minus=1;
	}
	
	tempshort=value;
	
	while(tempshort>=temppow) //to find out digit before decimal point
	{
		temppow*=10;
		x++;
	}
	ptr+=x;

	for(k=x;k>0;k--)  // to convert value before dp into BCD
	{
		ptr--;
		*ptr=(tempshort%10)+0x30;  
		tempshort/=10;
	}
	
	if(minus) x++;
	
	return x;
}

uint8_t CalCRC(uint8_t *ptr,unsigned short NoOfByte)
{
	unsigned long Total=0x00000055;
	unsigned short i=0;
	
	for (i=0;i<NoOfByte;i++)
    {
        Total += (unsigned long)*(ptr + i);
    }
	
	if(Total > 0x0000007F) Total &= 0x0000007F;

	return ((uint8_t)Total);
}

//**********************************************************************************************************
//	Timer0 related functions
//*********************************************************************************************************

void Init_Timer0(void)
{
	unsigned short temp=0;
	
	temp=(unsigned short)(2728.0*(DISP_PER/100));
	
	TCE0_CTRLB = (TC0_CCAEN_bm | TC_WGMODE_SS_gc);
	TCE0_CCA = temp;//1636;//1364;//535;//1364;
	TCE0_PER = 2728;//1070;//2728;
	TCE0_CTRLA = TC_CLKSEL_DIV1_gc;
	TCE0_INTCTRLA = 0b00000000;
}

//ISR(TCC0_OVF_vect)
//{
	////static uint8_t mcnt=0;
	//
	//TCC0_CNT = 65000;//64171;		//for 370 uSecond at 3.6864 MHz	
	//
	////TOGGLE_BACKLIT;
	////TOGGLE_BUZZER;
	////BUZZER_ON;
	//
	//TCC0_CNT = 0xE3DF;		    //for 500 mSecond at 14.7456 MHz
	//*/
//}

void ReadDiffPressure1(void)
{	
	unsigned short differanceDP=0;
	uint8_t DpError=0;
	
	Dpressure1=0.0;

	//Start Command Mode ----------------------------------
	I2C4_Start();						// Start condition
	Write_Byte_I2C4(0x51); 				// Write device address
	
	SDA4_DIR_IN;
	
	differanceDP=0;
	differanceDP = Read_Byte_I2C4(ACK);
	differanceDP<<=8;
	differanceDP |= Read_Byte_I2C4(NO_ACK);
	
	SDA4_DIR_OUT;
	
	I2C4_Stop();              // Send a STOP condition on the TWI bus.

	if((differanceDP & 0xC000) == 0xC000) DpError = 1;
	
	differanceDP &= 0x3FFF;
			
	if(DpError)
	{
		b.DP1_NC=1;
		DP1_Alrm_ON=NO_ALARM;
		
		gu8_Dp1AlarmSensingTimer=0;
		
		#ifdef DEBUG_DP1
		opstr(1,"DP1 ERROR  ");
		#endif
	}
	else
	{
		b.DP1_NC=0;
		
		RealDpressure1 = (float)differanceDP-1638;	
		RealDpressure1 *= 0.0762951094834821;//0.1496910048065919;
		RealDpressure1 -= 500;//981;
		
		float f32_temp=0;
		f32_temp = RealDpressure1;
		f32_temp -= DP1_Cal_float_Value_F;
		f32_temp -= DP1_Cal_float_Value_C;
		
		if((f32_temp > 200) && (f32_temp <= 300)) f32_temp -= 1;
		else if((f32_temp > 300) && (f32_temp <= 400)) f32_temp -= 2;
		else if((f32_temp > 400) && (f32_temp <= 500)) f32_temp -= 3;
		else if((f32_temp > 500) && (f32_temp <= 600)) f32_temp -= 4;
		else if((f32_temp > 600) && (f32_temp <= 700)) f32_temp -= 5;
		else if((f32_temp > 700) && (f32_temp <= 800)) f32_temp -= 6;
		else if(f32_temp > 800) f32_temp -= 7;
		
		Dpressure1 = f32_temp;
		
		Dpressure1 = Kalman_Update(&Kalman[0], Dpressure1);
		
		//if(!DP_CHANGE_SENSE) //ENABLE for switch
		{
			if(!gu8_dp_sw_enb)
			{
				if(Dpressure1>-1.0) Dpressure1 += f32_dp_sw_factor[0]; //MINUS LIMIT
				else Dpressure1 -= f32_dp_sw_factor[0];
			}
			else
			{
				if((Dpressure1>0.03) || (Dpressure1<-0.03))
				{
					if(Dpressure1>0.03) Dpressure1 += f32_dp_sw_factor[0];
					if(Dpressure1<-0.03) Dpressure1 -= f32_dp_sw_factor[0];
				}
			}
		}
		
		//---------------------------------------------------------------------------
		/*Raw_pressure_cnt1[Raw_pressure_cnt_ind1++] = (Dpressure1 * 10);
		if(Raw_pressure_cnt_ind1>=RAW_DP_CNT_IND) Raw_pressure_cnt_ind1=0;
		
		signed long lu32_temp=0;
		
		lu32_temp = 0;
		for(i=0;i<RAW_DP_CNT_IND;i++) lu32_temp += Raw_pressure_cnt1[i];
		lu32_temp /= RAW_DP_CNT_IND;       
		
		Dpressure1 = (float)lu32_temp/10;
		*/
		
		if((Dpressure1<1.0) && (Dpressure1>-1.0))
		{
			Dpressure1 = 0;
		}
		
		if(Dpressure1 > f32_dp_limit[0]) 
		{
			Dpressure1 = f32_dp_limit[0];
			b.DP1_limit = 1;
		}
		else if(Dpressure1 < -f32_dp_limit[0]) 
		{
			Dpressure1 = -f32_dp_limit[0];
			b.DP1_limit = 2;
		}
		else
		{
			b.DP1_limit = 0;
		}
		
		if(abs(LastDpressure1-Dpressure1)>0.1)
		{
			LastDpressure1 = Dpressure1;
		}
		else
		{
			Dpressure1 = LastDpressure1;
		}
		
		if(!DP_StartUpTimer)
		{
			//Find DP1 Min/Max
			if(Dpressure1 > DP1_Max)
			{
				DP1_Max = Dpressure1;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Max,(uint8_t*)DP1_MAXIMUM,4);
			}
						
			if(Dpressure1 < DP1_Min)
			{
				DP1_Min = Dpressure1;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Min,(uint8_t*)DP1_MINIMUM,4);
			}
					
			//Check Alarm Limit for DP ------------------------------------------------------------------------
			if(Dpressure1 > (float)DP1_Upper_Alm_ON/10.0)
			{
				if(gu8_Dp1AlarmSensingTimer<=gu8_Dp2AlarmSensingTime)
				{
					gu8_Dp1AlarmSensingTimer++;
				}
				else
				{
					DP1_Alrm_ON=UPPER_ALARM;
				
					if(!b.DP1Log)
					{
						LastDP1_Alrm_ON=DP1_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
					
						FillRamBuffer(DP1_ALM_OCCURE_LOG,0,0xFFFF);
					
						//if(gu16_parameterWord & ENABLE_ALERT) StartBuzzer();
					
						b.autoSendResponse = true;
						b.DP1Log=1;
					}
					else
					{
						if(LastDP1_Alrm_ON!=DP1_Alrm_ON)
						{
							FillRamBuffer(DP1_ALM_RESTORE_LOG,0,0xFFFF);
						
							LastDP1_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
						
							b.DP1Log=0;
						}
					}
				}
			}
			else if(Dpressure1 < (float)DP1_Lower_Alm_ON/10.0)
			{
				if(gu8_Dp1AlarmSensingTimer<=gu8_Dp2AlarmSensingTime)
				{
					gu8_Dp1AlarmSensingTimer++;
				}
				else
				{
					DP1_Alrm_ON=LOWER_ALARM;
					
					if(!b.DP1Log)
					{
						LastDP1_Alrm_ON=DP1_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
						
						FillRamBuffer(DP1_ALM_OCCURE_LOG,0,0xFFFF);

						b.autoSendResponse = true;
						b.DP1Log=1;
					}
					else
					{
						if(LastDP1_Alrm_ON!=DP1_Alrm_ON)
						{
							FillRamBuffer(DP1_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastDP1_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
							
							b.DP1Log=0;
						}
					}
				}
			}
			else if((Dpressure1 < (float)DP1_Upper_Alm_OFF/10.0) && (Dpressure1 > (float)DP1_Lower_Alm_OFF/10.0))
			{
				DP1_Alrm_ON=NO_ALARM;
				
				gu8_Dp1AlarmSensingTimer=0;
				
				if(b.DP1Log==1)
				{
					FillRamBuffer(DP1_ALM_RESTORE_LOG,0,0xFFFF);
					
					b.DP1Log=0;
					
					LastDP1_Alrm_ON=DP1_Alrm_ON;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
				}
			}
		}
	}
		
	#ifdef DEBUG_DP1				
	opstr(1," SM9541 Reading:  ");
	opstr(1,"DP1 Count: ");
	print_float(1,differanceDP,test,0);
	opstr(1,"    ");
	opstr(1,"DP1 Value: ");
	print_float(1,Dpressure1,test,2);
	opstr(1,"\r\n");
	#endif
}

void ReadDiffPressure2(void)
{
	unsigned short differanceDP=0;
	uint8_t DpError=0;
	
	Dpressure2=0.0;
	
	//Start Command Mode ----------------------------------
	I2C2_Start();						// Start condition
	Write_Byte_I2C2(0x51); 				// Write device address

	SDA2_DIR_IN;

	differanceDP=0;
	differanceDP = Read_Byte_I2C2(ACK);
	differanceDP<<=8;
	differanceDP |= Read_Byte_I2C2(NO_ACK);

	SDA2_DIR_OUT;

	I2C2_Stop();              // Send a STOP condition on the TWI bus.

	if((differanceDP & 0xC000) == 0xC000) DpError = 1;
	
	differanceDP &= 0x3FFF;
	
	if(DpError)
	{
		b.DP2_NC=1;
		DP2_Alrm_ON=NO_ALARM;
		
		gu8_Dp2AlarmSensingTimer=0;
		
		#ifdef DEBUG_DP2
		opstr(1,"DP2 ERROR  ");
		#endif
	}
	else
	{
		b.DP2_NC=0;

		RealDpressure2 = (float)differanceDP-1638;			
		RealDpressure2 *= 0.0762951094834821;//0.1496910048065919;//0762951094834821 
		RealDpressure2 -= 500;//981; pressure range  
		
		float f32_temp=0;
		f32_temp = RealDpressure2;
		f32_temp -= DP2_Cal_float_Value_F;
		f32_temp -= DP2_Cal_float_Value_C;
		
		if((f32_temp > 200) && (f32_temp <= 300)) f32_temp -= 1;
		else if((f32_temp > 300) && (f32_temp <= 400)) f32_temp -= 2;
		else if((f32_temp > 400) && (f32_temp <= 500)) f32_temp -= 3;
		else if((f32_temp > 500) && (f32_temp <= 600)) f32_temp -= 4;
		else if((f32_temp > 600) && (f32_temp <= 700)) f32_temp -= 5;
		else if((f32_temp > 700) && (f32_temp <= 800)) f32_temp -= 6;
		else if(f32_temp > 800) f32_temp -= 7;
		
		Dpressure2 = f32_temp;
		
		Dpressure2 = Kalman_Update(&Kalman[1], Dpressure2);
		
		//if(!DP_CHANGE_SENSE) //ENABLE for switch
		{
			if(!gu8_dp_sw_enb)
			{
				if(Dpressure2>-1.0) Dpressure2 += f32_dp_sw_factor[1];  //MINUS LIMIT 2
				else Dpressure2 -= f32_dp_sw_factor[1];
			}
			else
			{
				if((Dpressure2>0.03) || (Dpressure2<-0.03))
				{
					if(Dpressure2>0.03) Dpressure2 += f32_dp_sw_factor[1];
					if(Dpressure2<-0.03) Dpressure2 -= f32_dp_sw_factor[1];
				}
			}
		}

		//---------------------------------------------------------------------------
		/*Raw_pressure_cnt2[Raw_pressure_cnt_ind2++] = (Dpressure2 * 10);
		if(Raw_pressure_cnt_ind2>=RAW_DP_CNT_IND) Raw_pressure_cnt_ind2=0;
		
		signed long lu32_temp=0;
		
		lu32_temp = 0;
		for(i=0;i<RAW_DP_CNT_IND;i++) lu32_temp += Raw_pressure_cnt2[i];
		lu32_temp /= RAW_DP_CNT_IND;       
		
		Dpressure2 = (float)lu32_temp/10;
		*/
		
		if((Dpressure2<1.0) && (Dpressure2>-1.0))
		{
			Dpressure2 = 0;
		}
		
		if(Dpressure2 > f32_dp_limit[1]) 
		{
			Dpressure2 = f32_dp_limit[1];
			b.DP2_limit = 1;
		}
		else if(Dpressure2 < -f32_dp_limit[1]) 
		{
			Dpressure2 = -f32_dp_limit[1];
			b.DP2_limit = 2;
		}
		else
		{
			b.DP2_limit = 0;
		}
		
		if(abs(LastDpressure2-Dpressure2)>0.1)
		{
			LastDpressure2 = Dpressure2;
		}
		else
		{
			Dpressure2 = LastDpressure2;
		}
		
		//---------------------------------------------------------------------------
		
		if(!DP_StartUpTimer)
		{
			//Find DP Min/Max
			if(Dpressure2 > DP2_Max)
			{
				DP2_Max = Dpressure2;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Max,(uint8_t*)DP2_MAXIMUM,4);
			}
		
			if(Dpressure2 < DP2_Min)
			{
				DP2_Min = Dpressure2;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Min,(uint8_t*)DP2_MINIMUM,4);
			}
			
			//Check Alarm Limit for DP ------------------------------------------------------------------------
			if(Dpressure2 > (float)DP2_Upper_Alm_ON/10.0)
			{
				if(gu8_Dp2AlarmSensingTimer<=gu8_Dp2AlarmSensingTime)
				{
					gu8_Dp2AlarmSensingTimer++;
				}
				else
				{
					DP2_Alrm_ON=UPPER_ALARM;
					
					if(!b.DP2Log)
					{
						LastDP2_Alrm_ON=DP2_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
						
						FillRamBuffer(DP2_ALM_OCCURE_LOG,0,0xFFFF);
						
						b.autoSendResponse = true;
						b.DP2Log=1;
					}
					else
					{
						if(LastDP2_Alrm_ON!=DP2_Alrm_ON)
						{
							FillRamBuffer(DP2_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastDP2_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
							
							b.DP2Log=0;
						}
					}
				}
			}
			else if(Dpressure2 < (float)DP2_Lower_Alm_ON/10.0)
			{
				if(gu8_Dp2AlarmSensingTimer<=gu8_Dp2AlarmSensingTime)
				{
					gu8_Dp2AlarmSensingTimer++;
				}
				else
				{
					DP2_Alrm_ON=LOWER_ALARM;
					
					if(!b.DP2Log)
					{
						LastDP2_Alrm_ON=DP2_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
						
						FillRamBuffer(DP2_ALM_OCCURE_LOG,0,0xFFFF);
						
						b.autoSendResponse = true;
						b.DP2Log=1;
					}
					else
					{
						if(LastDP2_Alrm_ON!=DP2_Alrm_ON)
						{
							FillRamBuffer(DP2_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastDP2_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
							
							b.DP2Log=0;
						}
					}
				}
			}
			else if((Dpressure2 < (float)DP2_Upper_Alm_OFF/10.0) && (Dpressure2 > (float)DP2_Lower_Alm_OFF/10.0))
			{
				DP2_Alrm_ON=NO_ALARM;
				
				gu8_Dp2AlarmSensingTimer=0;
				
				if(b.DP2Log==1)
				{
					FillRamBuffer(DP2_ALM_RESTORE_LOG,0,0xFFFF);
					
					b.DP2Log=0;
					
					LastDP2_Alrm_ON=DP2_Alrm_ON;
					eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
				}
			}
		}
	}

	#ifdef DEBUG_DP2
	opstr(1," SM9541 Reading:  ");
	opstr(1,"DP2 Count: ");
	print_float(1,differanceDP,test,0);
	opstr(1,"    ");
	//opstr(0,"Temp Count: ");
	//print_float(0,tempDp,test,0);
	//opstr(0,"    ");
	opstr(1,"DP2 Value: ");
	print_float(1,Dpressure2,test,2);
	opstr(1,"\r\n\r\n\r\n");
	#endif
}

/**@brief Convert 8 digit ascii serial# into hex format
 */
uint32_t ascii2hex(uint8_t *data, uint8_t NoOfdigit)
{
    uint8_t i,val;
    uint32_t num=0;

    for(i=0; i<NoOfdigit; i++)
    {
        val = (*data - 0x30);
        //num = num + (uint32_t)((double)val * pow(10,((NoOfdigit-1)-i)));
		num = (num*10) + val;
        data++;
    }

    return num;
}
//**********************************************************************************************************
//	Internal RTC related functions
//*********************************************************************************************************

void Init_InternalRTC(void)
{
	while(RTC.STATUS & RTC_SYNCBUSY_bm);
	
	RTC.PER = 51;//102;//256;//511;
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	RTC_INTCTRL = RTC_OVFINTLVL_LO_gc;
}

ISR(RTC_OVF_vect)
{
	static uint8_t mcnt=0,mcnt1=0,mcnt2=0;
	
	b.msec50_flag = 1;
	
	mcnt2++;
	if(mcnt2>=5)
	{
		mcnt2=0;
		b.msec250_flag = 1;
	}
	
	mcnt1++;
	if(mcnt1>=10)
	{
		mcnt1=0;
		
		b.mec500_blink_flag ^= 1;
		b.led_toggle ^= 1;
		b.AlarmLED = 1;
		
		b.msec500_flag = 1;
		
		if(RxTimeout)
		{
			RxTimeout--;
			if(!RxTimeout)
			{
				b.msgRcvOK=0;
				rxMode=0;
				RxInd=0;
			}
		}
	}
	
	//---------------------------------------------
	mcnt++;
	if(mcnt>=20)
	{
		mcnt=0;
		b.sec_flag=1;
	}
	//---------------------------------------------
}

//**********************************************************************************************************
//	ADC related functions
//**********************************************************************************************************
#ifdef ENABLE_BATTERY_DISPLAY
void InitADC(void)
{
	ADCB_PRESCALER = ADC_PRESCALER_DIV128_gc;	//ADC clock = 14.7456 MHz/128 = ~115.2KHz	
	
	ADCB_CTRLB = ADC_CURRLIMIT_HIGH_gc | ADC_RESOLUTION_8BIT_gc;
	ADCB_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCB_CH0_MUXCTRL = 0x00;
	
	ADCB_REFCTRL = ADC_BANDGAP_bm;
	//ADCB_REFCTRL = ADC_REFSEL_INT1V_gc;
	
	ADCB.EVCTRL = 0x00;
		
	ADCB_INTFLAGS = ADC_CH0IF_bm;
	
	//ADCB_CH0_INTCTRL = ADC_CH_INTLVL_HI_gc;
	//ADCB_CH0_INTCTRL = ADC_CH_INTLVL_MED_gc;
	//ADCB_CH0_INTCTRL = ADC_CH_INTLVL_LO_gc;
	ADCB_CH0_INTCTRL = ADC_CH_INTLVL_OFF_gc;
		
	ADCB_CH0_INTFLAGS = ADC_CH0IF_bm;
		
	ADCB_CTRLA = ADC_ENABLE_bm;
}

void ReadADCForBatteryVoltage(void)
{
	ADCB_CTRLA = ADC_ENABLE_bm;
		
	ADCB_CH0_CTRL |= ADC_CH_START_bm;
		
	while(!ADCB_CH0_INTFLAGS);
		
	ADC_sample=ADCB_CH0_RES;
		
	ADCB_CH0_INTFLAGS = ADC_CH0IF_bm;
		
	ADCB_CTRLA &= ~ADC_ENABLE_bm;
		
	//---------------------
	//BatteryPercentage = ((float)ADC_sample / 255)*100;
	//if(BatteryPercentage>100) BatteryPercentage=100;
	if(ADC_sample > 165) 
	{
		BatteryPercentage = ADC_sample - 165;
	}
	else
	{
		BatteryPercentage = 0;
	}
	//BatteryPercentage = 100 - BatteryPercentage;
	//---------------------
			
	/*opstr(0,"ADC Reading:");
	print_float(0,ADC_sample,&test[0],0);
	opstr(0,"   ");
	
	opstr(0,"Battery Voltage:");
	print_float(0,BatteryPercentage,&test[0],0);
	opstr(0,"\r\n");
	*/
}

// Interrupt Service Routine for handling the ADC conversion complete interrupt
/*ISR(ADCB_CH0_vect) 
{
	ADC_sample=ADCB_CH0_RES;
		
	ADCB_CH0_INTFLAGS = ADC_CH0IF_bm;
		
	//---------------------
	BatteryPercentage = ((float)ADC_sample / 255)*100;
	if(BatteryPercentage>100) BatteryPercentage=100;
	//---------------------
		
	opstr(0,"ADC Reading:");
	print_float(0,ADC_sample,&test[0],0);
	opstr(0,"   ");
		
	opstr(0,"Battery Voltage:");
	print_float(0,BatteryPercentage,&test[0],0);
	opstr(0,"\r\n");
}*/
#endif
	
//**********************************************************************************************************
//ALL FUNCTION DEFINATION START HERE
//*********************************************************************************************************
void DisableUnusedModules(void)
{
	// the power reduction settings are specific for each device
	// PRGEN general power reduction register
	//
	// disable
	//	- AES
	//  - DMA
	//  - event system
	//  - external bus interface
	//
	PR.PRGEN |= PR_AES_bm | PR_DMA_bm | PR_EVSYS_bm | PR_EBI_bm;
	//PR_PRGEN = 0b11010011;

	// PRPA, PRPB power reduction port A/B register
	//
	// disable
	//	- analog comparator
	//	- ADC on port B
	//  - DAC
	//
	// leave ADCA active !
	//
	PR.PRPA |= PR_AC_bm | PR_ADC_bm | PR_DAC_bm;
	PR.PRPB |= PR_AC_bm | PR_DAC_bm;
	//PR_PRPA = 0b00000111;
	//PR_PRPB = 0b00000101;
	
	// PRPC/D/E/F Power Reduction Port C/D/E/F Register
	//
	// disable unused timers
	//
	// - high resolution extension
	// - TCD1, TCD1       on, used for nested intérrupt tests
	// - TCC1, TCD0, TCE0 (if not simulating serial input)
	//
	// TCC0 is the tick timer, always on!
	//
	PR.PRPC |= PR_HIRES_bm;
	PR.PRPE |= PR_HIRES_bm;
	//PR_PRPC = 0b01001110;
	//PR_PRPE = 0b01001110;
	
	// enable eeprom and flash power reduction
	//
	//NVM.CTRLB |= NVM_EPRM_bm | NVM_FPRM_bm;

	//DISABLE_JTAG();
}

void Init_GPIO(void)
{
	//==============
	//  Port A
	//==============
	// PA0 is OUTPUT1
	// PA1 is OUTPUT2
	// PA2 is OUTPUT3
	// PA3 is OUTPUT4
	// PA4 is ALARM7
	// PA5 is NC
	// PA6 is KEY5
	// PA7 is KEY6
	
	PORTA_DIR = 0x3F;
	PORTA_OUT = 0xFF;      // set high
	
	PORTA.DIRCLR = PIN6_bm; // pin6 is input
	__PORT_PULLUP(PORTA, PIN6_bm); // Enable Pull up on KEY3
	
	PORTA.DIRCLR = PIN7_bm; // pin7 is input
	__PORT_PULLUP(PORTA, PIN7_bm); // Enable Pull up on KEY4
	
	//==============
	//  Port B
	//==============
	// PB0 is Analog Input1
	// PB1 is Analog Input2
	// PB2 is Analog Input3
	// PB3 is Door Status
	// PB4 is TMS
	// PB5 is TDI
	// PB6 is TCK
	// PB7 is TDO
	
	PORTB_DIR = 0x00;
	
	//==============
	//  Port C
	//==============
	// PC0 is SDA1
	// PC1 is SCL1
	// PC2 is RXD1
	// PC3 is TXD1
	// PC4 is DIR1
	// PC5 is DIR2
	// PC6 is RXD2
	// PC7 is TXD2
	
	PORTC.DIRSET = PIN0_bm; // pin0 is output
	PORTC.OUTSET = PIN0_bm; // set SDA1=1
	
	PORTC.DIRSET = PIN1_bm; // pin1 is output
	PORTC.OUTSET = PIN1_bm; // set SCL1=1
	
	PORTC.DIRCLR = PIN2_bm; // pin2 RXD1 is input
	
	PORTC.DIRSET = PIN3_bm; // pin3 TXD1 is output
	
	PORTC.DIRSET = PIN4_bm; // pin4 is output
	PORTC.OUTCLR = PIN4_bm; // DE1=0, set the RS485 driver to receive mode
	
	PORTC.DIRSET = PIN5_bm; // pin5 is output
	PORTC.OUTCLR = PIN5_bm; // DE2=0, set the RS485 driver to receive mode
	
	PORTC.DIRCLR = PIN6_bm; // pin2 RXD2 is input
	
	PORTC.DIRSET = PIN7_bm; // pin3 TXD2 is output
	
	//==============
	//  Port D
	//==============
	// PD0 is SDA2
	// PD1 is SCL2
	// PD2 is RXD3
	// PD3 is TXD3
	// PD4 is KEY1
	// PD5 is KEY2
	// PD6 is KEY3
	// PD7 is KEY4
	
	PORTD.DIRSET = PIN0_bm; // pin0 is output
	PORTD.OUTSET = PIN0_bm; // set SDA2=1
	
	PORTD.DIRSET = PIN1_bm; // pin1 is output
	PORTD.OUTSET = PIN1_bm; // set SCL2=1
	
	PORTD.DIRCLR = PIN2_bm; // pin2 RXD3 is input
	
	PORTD.DIRSET = PIN3_bm; // pin3 TXD3 is output
	
	PORTD.DIRCLR = PIN4_bm; // pin4 is input
	__PORT_PULLUP(PORTD, PIN4_bm); // Enable Pull up on KEY1
	
	PORTD.DIRCLR = PIN5_bm; // pin5 is input
	__PORT_PULLUP(PORTD, PIN5_bm); // Enable Pull up on KEY2
	
	PORTD.DIRCLR = PIN6_bm; // pin6 is input
	__PORT_PULLUP(PORTD, PIN6_bm); // Enable Pull up on KEY3
		
	PORTD.DIRCLR = PIN7_bm; // pin7 is input
	__PORT_PULLUP(PORTD, PIN7_bm); // Enable Pull up on KEY4	
	
	//==============
	//  Port E
	//==============
	// PE0 is SDA3
	// PE1 is SCL3
	// PE2 is RXD4
	// PE3 is TXD4
	// PE4 is CS
	// PE5 is SDI
	// PE6 is SDO
	// PE7 is SCLK
	
	PORTE.DIRSET = PIN0_bm; // pin0 is output
	PORTE.OUTSET = PIN0_bm; // set SDA3=1
	
	PORTE.DIRSET = PIN1_bm; // pin1 is output
	PORTE.OUTSET = PIN1_bm; // set SCL3=1
	
	PORTE.DIRCLR = PIN2_bm; // pin2 RXD4 is input
	
	PORTE.DIRSET = PIN3_bm; // pin3 TXD4 is output
	
	PORTE.DIRSET = PIN4_bm; // pin4 is output
	PORTE.OUTSET = PIN4_bm; // set CS=1
	
	PORTE.DIRSET = PIN5_bm; // pin5 is output
	PORTE.OUTSET = PIN5_bm; // set SDI=1
	
	PORTE.DIRCLR = PIN6_bm; // pin6 is input
	PORTE.OUTSET = PIN6_bm; // set SDO=1
	
	PORTE.DIRSET = PIN7_bm; // pin7 is output
	PORTE.OUTSET = PIN7_bm; // set SCLK=1
	
	//==============
	//  Port F
	//==============
	// PF0 is SDA4
	// PF1 is SCL4
	// PF2 is ALARM LED1
	// PF3 is OK LED1
	// PF4 is ALARM LED2
	// PF5 is OK LED2
	// PF6 is ALARM LED2
	// PF7 is OK LED2
	
	PORTF.DIRSET = PIN0_bm; // pin0 is output
	PORTF.OUTSET = PIN0_bm; // set SDA4=1
	
	PORTF.DIRSET = PIN1_bm; // pin1 is output
	PORTF.OUTSET = PIN1_bm; // set SCL4=1
	
	PORTF.DIRSET = PIN2_bm; // pin2 is output
	PORTF.OUTSET = PIN2_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN3_bm; // pin3 is output
	PORTF.OUTSET = PIN3_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN4_bm; // pin4 is output
	PORTF.OUTSET = PIN4_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN5_bm; // pin5 is output
	PORTF.OUTSET = PIN5_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN6_bm; // pin6 is output
	PORTF.OUTSET = PIN6_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN7_bm; // pin7 is output
	PORTF.OUTSET = PIN7_bm; // set LED=1 for off
	
	//==============
	//  Port H
	//==============
	// PH0 is BUZZER
	// PH1 is RST1
	// PH2 is XBEE RESET
	// PH3 is LCD RS
	// PH4 is LCD RB0
	// PH5 is LCD RB1
	// PH6 is LCD RST
	// PH7 is LCD CS
	
	PORTH.DIRSET = PIN0_bm; // pin0 is output
	PORTH.OUTCLR = PIN0_bm; // set BUZZER OFF
		
	PORTH.DIRSET = PIN1_bm; // pin1 is output
	PORTH.OUTSET = PIN1_bm; // set high
	
	PORTH.DIRSET = PIN2_bm; // pin2 is output
	PORTH.OUTSET = PIN2_bm; // set high
	
	PORTH.DIRSET = PIN3_bm; // pin3 is output
	PORTH.OUTSET = PIN3_bm; // set LCD RS to HIGH
	
	PORTH.DIRSET = PIN4_bm; // pin4 is output
	PORTH.OUTSET = PIN4_bm; // set LCD RB0 to HIGH
	
	PORTH.DIRSET = PIN5_bm; // pin5 is output
	PORTH.OUTSET = PIN5_bm; // set LCD RB1 to HIGH
	
	PORTH.DIRSET = PIN6_bm; // pin6 is output
	PORTH.OUTSET = PIN6_bm; // set LCD RST to HIGH
	
	PORTH.DIRSET = PIN7_bm; // pin7 is output
	PORTH.OUTSET = PIN7_bm; // set LCD CS to HIGH
	
	//==============
	//  Port J
	//==============
	// PJ0 is LCD D0
	// PJ1 is LCD D1
	// PJ2 is LCD D2
	// PJ3 is LCD D3
	// PJ4 is LCD D4
	// PJ5 is LCD D5
	// PJ6 is LCD D6
	// PJ7 is LCD D7
	
	PORTJ_DIR = 0xFF;		// Set all pins output
	PORTJ_OUT = 0x00;		// set low

	//==============
	//  Port K
	//==============
	// PK0 is INPUT1
	// PK1 is INPUT2
	// PK2 is INPUT3
	// PK3 is INPUT4
	// PK4 is INPUT5
	// PK5 is ALARM4
	// PK6 is ALARM5
	// PK7 is ALARM6
	
	PORTK_DIR = 0xE0;		
	__PORT_PULLUP(PORTK, PIN0_bm);
	__PORT_PULLUP(PORTK, PIN1_bm);
	__PORT_PULLUP(PORTK, PIN2_bm);
	__PORT_PULLUP(PORTK, PIN3_bm);
	__PORT_PULLUP(PORTK, PIN4_bm);
	
	PORTK.DIRSET = PIN5_bm; // pin5 is output
	PORTK.OUTCLR = PIN5_bm; // set ALARM4 to LOW
	
	PORTK.DIRSET = PIN6_bm; // pin6 is output
	PORTK.OUTCLR = PIN6_bm; // set ALARM5 to LOW
	
	PORTK.DIRSET = PIN7_bm; // pin7 is output
	PORTK.OUTCLR = PIN7_bm; // set ALARM6 to LOW
	
	//==============
	//  Port Q
	//==============
	// PQ0 is XTAL1
	// PQ1 is XTAL2
	// PQ2 is ALARM LED4
	// PQ3 is OK LED4
	
	PORTQ.DIRSET = PIN2_bm; // pin2 is output
	PORTQ.OUTSET = PIN2_bm; // set LED=1 for off
	
	PORTQ.DIRSET = PIN3_bm; // pin3 is output
	PORTQ.OUTSET = PIN3_bm; // set LED=1 for off
}

void Init_variables(void)
{
	unsigned short i=0;
	
	for(i=0;i<NO_DIGIT;i++)
	{	
		disp_buffer[i]=0;
		data[i]=BLANK;
	}
	
	for(i=0;i<RAW_DP_CNT_IND;i++) 
	{
		Raw_pressure_cnt2[i]=ZERO_DP2_COUNT;
	}
	memset(&Buffer1[0],0,sizeof(Buffer1));
	memset(&TxBuffer[0],0,sizeof(TxBuffer));
	memset(&RxBuffer[0],0,sizeof(RxBuffer));
	memset(&RAMBuffer[0],0,sizeof(RAMBuffer));
	
	if(!(gu16_parameterWord & ENABLE_DP1))
	{
		for(i=0;i<RAW_DP_CNT_IND;i++) Raw_pressure_cnt1[i]=ZERO_AP1_COUNT;
	}
	else
	{
		for(i=0;i<RAW_DP_CNT_IND;i++) Raw_pressure_cnt1[i]=ZERO_DP1_COUNT;
	}
	
	DP_StartUpTimer=0;
	TMRH_StartUpTimer=0;
	
	b.autoSendResponse = false;
	b.triggerXbeeReset = false;
	gu32_triggerXbeeResetTimer = (unsigned long)gu16_XbeeRstInterval*60;
	
	//gu8_AutoSentTimer=gu8_AutoSentInterval;
	gu8_groupID = ((DeviceID - 1)/gu8_DeviceInGroup)+1;
}

//**************************************************************************************************************************************

void SecondTick(void)
{
	//static uint8_t acnt1=0;
	
	b.batteryPerBlink ^= 1;
	
	if(gu8_masterEnable==1)
	{
		SendToSlave();
	}
	
	if(gu16_DPAutoCalTimer10Sec[0])
	{
		gu16_DPAutoCalTimer10Sec[0]--;
		if(!gu16_DPAutoCalTimer10Sec[0])
		{
			if(gu8_DPAutoCalDoorCnt[0] >= 3)
			{
				if(b.doorStatus==OPEN)
				{
					gu16_DPAutoCalTimer5Min[0] = 300;
				}
			}
			else
			{
				gu8_DPAutoCalDoorCnt[0] = 0;
			}
		}
	}
	
	if(gu16_DPAutoCalTimer5Min[0])
	{
		gu16_DPAutoCalTimer5Min[0]--;
		if(!gu16_DPAutoCalTimer5Min[0])
		{
			//Auto cal DP1
			DP1_Cal_Value_C = (RealDpressure1 - DP1_Cal_float_Value_F)*10.0;
			DP1_Cal_float_Value_C = (float)DP1_Cal_Value_C/10.0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
		}
	}
	
	//-----------------------------------------------------------------------------------------------------------
	if(gu16_DPAutoCalTimer10Sec[1])
	{
		gu16_DPAutoCalTimer10Sec[1]--;
		if(!gu16_DPAutoCalTimer10Sec[1])
		{
			if(gu8_DPAutoCalDoorCnt[1] >= 6)
			{
				if(b.doorStatus==OPEN)
				{
					gu16_DPAutoCalTimer5Min[1] = 300;
				}
			}
			else
			{
				gu8_DPAutoCalDoorCnt[1] = 0;
			}
		}
	}
	
	if(gu16_DPAutoCalTimer5Min[1])
	{
		gu16_DPAutoCalTimer5Min[1]--;
		if(!gu16_DPAutoCalTimer5Min[1])
		{
			//Auto cal DP2
			DP2_Cal_Value_C = (RealDpressure2 - DP2_Cal_float_Value_F)*10.0;
			DP2_Cal_float_Value_C = (float)DP2_Cal_Value_C/10.0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
		}
	}
	//-----------------------------------------------------------------------------------------------------------
	
	if(gu8_deviceIDChangeTryTimer)
	{
		gu8_deviceIDChangeTryTimer--;
		if(!gu8_deviceIDChangeTryTimer)
		{
			gu8_deviceIDChangeTry=0;
		}
	}
	

	if(gu8_Mac2ValidTimer)
	{
		gu8_Mac2ValidTimer--;
		if(!gu8_Mac2ValidTimer)
		{
			SetMAC2Xbee(&gu8arr_XbeeMac[0][0],0);
		}
	}
	
	if(gu32_triggerXbeeResetTimer)
	{
		gu32_triggerXbeeResetTimer--;
		if(!gu32_triggerXbeeResetTimer)
		{
			gu32_triggerXbeeResetTimer = (unsigned long)gu16_XbeeRstInterval*60;

			b.triggerXbeeReset = true;
		}
	}
	
	if(gu8_broadcast)
	{
		if(gu8_AutoSentTimer)
		{
			gu8_AutoSentTimer--;
			if(!gu8_AutoSentTimer)
			{
				b.autoSendResponse = true;

				gu8_AutoSentTimer=gu8_AutoSentInterval;
			}
		}
		
		if(gu8_AutoSentTimeout)
		{
			gu8_AutoSentTimeout--;
			if(!gu8_AutoSentTimeout)
			{
				b.autoSendResponse = true;
				
				gu8_AutoSentTimer=gu8_AutoSentInterval;
			}
		}
	}
	
	//--------------------------------------------
	/*if(gu8_DPAutoCalTimer)
	{
		gu8_DPAutoCalTimer--;
		if(!gu8_DPAutoCalTimer)
		{
			if(!b.DP1_NC)
			{
				DP1_Cal_Value_F = RealDpressure1*10.0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_F_ADDR,DP1_Cal_Value_F);
				DP1_Cal_float_Value_F = (float)DP1_Cal_Value_F/10.0;
				
				DP1_Cal_Value_C = 0;
				DP1_Cal_float_Value_C = 0;
				
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)DP1_CAL_DATE_ADDR,12);
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[21],(uint8_t*)DP1_CAL_CERT_ADDR,15);
			}
			
			if(!b.DP2_NC)
			{
				DP2_Cal_Value_F = RealDpressure2*10.0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_F_ADDR,DP2_Cal_Value_F);
				DP2_Cal_float_Value_F = (float)DP2_Cal_Value_F/10.0;
				
				DP2_Cal_Value_C = 0;
				DP2_Cal_float_Value_C = 0;
				
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[9],(uint8_t*)DP2_CAL_DATE_ADDR,12);
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RxBuffer[21],(uint8_t*)DP2_CAL_CERT_ADDR,15);
			}
			
			gu8_DPAutoCalFlag=1;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP_AUTO_CAL_FLAG,gu8_DPAutoCalFlag);
		}
	}*/
	//--------------------------------------------
	if(b.buzzerStart==YES)
	{
		if(buzzerOnTime)
		{
			buzzerOnTime--;
			if(!buzzerOnTime)
			{
				buzzerOffTime=Buzzer_OFF_Time;
				
				if(buzzerOffTime)
				{
					BUZZER_OFF;	
				}
				else
				{
					buzzerOnTime=Buzzer_ON_Time;
				}
			}
		}
		else if(buzzerOffTime)
		{
			buzzerOffTime--;
			if(!buzzerOffTime)
			{
				buzzerOnTime=Buzzer_ON_Time;
				
				if(buzzerOnTime)
				{
					BUZZER_ON;	
				}
				else
				{
					buzzerOffTime=Buzzer_OFF_Time;	
				}
			}
		}
	}
	if(DP_StartUpTimer)DP_StartUpTimer--;
	if(TMRH_StartUpTimer)TMRH_StartUpTimer--;
	
	if(gu8_restartTimer)
	{
		gu8_restartTimer--;
		if(!gu8_restartTimer)
		{
			CPU_CCP=0xD8;
			RST_CTRL=RST_SWRST_bm;
		}
	}
	
	//--------------------------------------------
	if(progTimeout)
	{
		progTimeout--;
		if(!progTimeout)
		{
			mode=NORMAL_MODE;
			Normal_para_cnt=0;
			autoCal_para_cnt=0;
			b.SetACKPwd=0;
			
			restoreFactoryCalibrationTimer=0;
			DPAutoCalModeTimer=0;
			DPAutoCalTimer=0;
			ProgModeTimer=0;
			MinMaxMeanModeTimer=0;
			MeanHrModeTimer=0;
			dispMinMaxMeanLogInd=0;
			gu8_MinMaxClearTimer=0;
			
			for(uint8_t i=0;i<NO_DIGIT;i++) data[i]=BLANK;
			data[4] = N;
			data[5] = 0;
			data[6] = r;
			disp_value();
			
			_delay_ms(4000);
		}
	}
	//--------------------------------------------
	if(PCCalibrationTimer)
	{
		PCCalibrationTimer--;
		if(!PCCalibrationTimer)
		{
			b.FactoryCalibrationOn=0;
			b.CustmerCalibrationOn=0;
		}
	}
	//--------------------------------------------
	if(StartBroadcastTimer)
	{
		StartBroadcastTimer--;
		if(!StartBroadcastTimer)
		{
			b.brodcastEnb=0;
		}
	}
	//--------------------------------------------
	if(AlarmAckTimer)
	{
		AlarmAckTimer--;
		if(!AlarmAckTimer)
		{
			b.DP1Log=0;
			b.DP2Log=0;
			b.TMLog=0;
			b.RHLog=0;
		}
	}

	/*acnt1++;
	if(acnt1>=gu8_TM_RH_ScanTime)
	{
		acnt1=0;
			
		Read_SHT25();
	}*/
	
	#ifdef ENABLE_BATTERY_DISPLAY
	static uint8_t acnt2=0;
	acnt2++;
	if(acnt2>=BATTERY_VOLTAGE_READ_TIME)
	{
		acnt2=0;
		ReadADCForBatteryVoltage();
	}
	#endif
	
	#ifndef DISABLE_DOOR_SENSING

	if((!DOOR_SENSE && gu8_doorSensingPolarity) || (DOOR_SENSE && !gu8_doorSensingPolarity))
	{
		if(b.doorStatus==CLOSE)
		{
			b.doorStatus=OPEN;
			b.autoSendResponse = true;
			//opstr(1,"DoorOpen\n");
			//StartBuzzer();
			if(!gu16_DPAutoCalTimer5Min[0])
			{
				if(!gu16_DPAutoCalTimer10Sec[0]) 
				{
					gu16_DPAutoCalTimer10Sec[0] = 10;
					gu8_DPAutoCalDoorCnt[0] = 1;
				}
				else
				{
					gu8_DPAutoCalDoorCnt[0]++;
				}
			}
			
			if(!gu16_DPAutoCalTimer5Min[1])
			{
				if(!gu16_DPAutoCalTimer10Sec[1]) 
				{
					gu16_DPAutoCalTimer10Sec[1] = 10;
					gu8_DPAutoCalDoorCnt[1] = 1;
				}
				else
				{
					gu8_DPAutoCalDoorCnt[1]++;
				}
			}
		}
	}
	else
	{
		if(b.doorStatus==OPEN)
		{
			b.doorStatus=CLOSE;
			b.autoSendResponse = true;
			gu8_dp_sw_enb = 0;
			gu16_DPAutoCalTimer5Min[0] = 0;
			gu16_DPAutoCalTimer5Min[1] = 0;
			//opstr(1,"DoorClose\n");
			//gu8_doorSensingTimer=0;
		}
	}	
				
	#endif
}

//**********************************************************************************************************
//	LCD Controller related functions
//*********************************************************************************************************

void InitLEDController(void)
{
	TM1680Configure();
	//----------------------------
	AllSegment(ON);
	
	//while(1);
	
	TM1680Blink(BLINK_1SEC);
	
	_delay_ms(2000);
	
	TM1680Blink(BLINK_OFF);
	//-------------------------------------------------------------------
	
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	data[1] = V;
	data[2] = E;
	data[3] = r;
	
	data[5] = 19;
	data[6] = 2;
						
	disp_value();
	
	wdt_reset();
	
	//----------Just to check display segment healthiness------------
	_delay_ms(2000);
	
	//-------------------------------------------------------------------
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	data[2] = I;
	data[3] = D;
	convert_char(DeviceID,&data[4],3);
	disp_value();
		
	wdt_reset();
	
	_delay_ms(2000);
	
	//-------------------------------------------------------------------
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	
	data[1] = B;
	data[2] = r;
	data[3] = t;
	
	switch(UART_BaudRate)
	{
		case BAUD_1200:		convert_char(1200,&data[4],4);		break;
		case BAUD_2400:		convert_char(2400,&data[4],4);		break;
		case BAUD_4800:		convert_char(4800,&data[4],4);		break;
		case BAUD_9600:		convert_char(9600,&data[4],4);		break;
		case BAUD_14400:	convert_char(14400,&data[4],5);		break;
		case BAUD_19200:	convert_char(19200,&data[4],5);		break;
		case BAUD_28800:	convert_char(28800,&data[4],5);		break;
		case BAUD_38400:	convert_char(38400,&data[4],5);		break;
		case BAUD_57600:	convert_char(57600,&data[4],5);		break;
		case BAUD_115200:	convert_float(115200,&data[4],0);	break;
		default:			convert_char(9600,&data[4],4);		break; 
	}

	disp_value();
	
	wdt_reset();
	
	_delay_ms(2000);
	
	//-------------------------------------------------------------------
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	data[2] = 5;
	data[3] = r;

	convert_float(gu32_SrNumber,&data[4],0);
	disp_value();
	
	wdt_reset();
	
	_delay_ms(2000);
	
	//-------------------------------------------------------------------
	//for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	//data[2] = 9;
	//data[3] = r;
	//convert_char(gu8_DeviceInGroup,&data[4],3);
	//disp_value();
	//
	//wdt_reset();
	//
	//_delay_ms(2000);
	
	
	wdt_reset();
}

void chartostr(unsigned short val,uint8_t *data1,uint8_t no_of_digit)
{
	data1+=(no_of_digit-1);
	
	while(no_of_digit)
	{
		*data1	= (val%10)+0x30;
		val/=10;
		data1--;
		no_of_digit--;
	}
}
	
void convert_char(unsigned short val,uint8_t* data1,uint8_t no_of_digit)
{
	data1+=(no_of_digit-1);
		
	while(no_of_digit)
	{
		*data1=(val%10);
		val/=10;
		data1--;
		no_of_digit--;
	}
}

void convert_long(unsigned long val,uint8_t* data1,uint8_t no_of_digit)
{
	data1+=(no_of_digit-1);
		
	while(no_of_digit)
	{
		*data1=(val%10);
		val/=10;
		data1--;
		no_of_digit--;
	}
}

void convert_float(float value,uint8_t* data1,uint8_t bytes_after_dp)
{
	uint8_t x;
	unsigned long temppow,lu32_templong;
	float tempdoub;
		
	x=1;
	temppow=10;
	tempdoub=value;
		
	//----------------------------------------------------------
	if(tempdoub<0.0)
	{
		tempdoub*=(-1.0);
		*data1=DASH;		//means '-' sign
		data1++;
	}
	//----------------------------------------------------------
		
	lu32_templong=tempdoub;
	tempdoub-=lu32_templong;
		
	while(lu32_templong>=temppow) //to find out digit before decimal point
	{
		temppow*=10;
		x++;
	}
	data1+=x;

	for(k=x;k>0;k--)  // to convert value before dp into BCD
	{
		data1--;
		*data1=lu32_templong%10;
		lu32_templong/=10;
	}

	if(bytes_after_dp)  // to convert value after dp into BCD
	{
		data1+=x;
		data1--;
		*data1 += 10;
		data1++;
		for(k=0;k<bytes_after_dp;k++)
		{
			tempdoub*=10;
			x=tempdoub;
			tempdoub-=x;
			*data1=x;
			data1++;
		}
	}
}

uint8_t changeNibbles(uint8_t byte)
{
	uint8_t byte1,byte2,byte3;
	byte1 = byte >> 4;
	byte2 = byte << 4;
	byte3 = byte1 | byte2;
	
	return byte3;
}
	
void AllSegment(uint8_t state)
{
	uint8_t disp[32];
	
	if(state == ON)
	{
		memset(disp,0xFF,sizeof(disp));
	}
	else
	{
		memset(disp,0x00,sizeof(disp));
	}

	TM1680WritePage(0, disp, 32);
}

void disp_value(void)
{
	if(gu8_restartTimer) return;
	
	for(uint8_t i=1;i<NO_DIGIT;i++) disp_buffer[i]=seg_code[data[i]];
	for(uint8_t i=0;i<32;i++) final_buffer[i]=0;
	
	if(lcd.Sym_RTC_AM) RTC_AM_on;
	if(lcd.Sym_RTC_PM) RTC_PM_on;
	if(lcd.Sym_RTC_BC1) RTC_BC1_on;
	if(lcd.Sym_RTC_COL) RTC_COL_on;
	
	if(disp_buffer[1] & 0x01) final_buffer[16] |= BIT1;//RTC_A2_on;
	if(disp_buffer[1] & 0x02) final_buffer[17] |= BIT1;//RTC_B2_on;
	if(disp_buffer[1] & 0x04) final_buffer[18] |= BIT1;//RTC_C2_on;
	if(disp_buffer[1] & 0x08) final_buffer[19] |= BIT1;//RTC_D2_on;
	if(disp_buffer[1] & 0x10) final_buffer[20] |= BIT1;//RTC_E2_on;
	if(disp_buffer[1] & 0x20) final_buffer[21] |= BIT1;//RTC_F2_on;
	if(disp_buffer[1] & 0x40) final_buffer[22] |= BIT1;//RTC_G2_on;

	if(disp_buffer[2] & 0x01) final_buffer[24] |= BIT0;//RTC_A3_on;
	if(disp_buffer[2] & 0x02) final_buffer[25] |= BIT0;//RTC_B3_on;
	if(disp_buffer[2] & 0x04) final_buffer[26] |= BIT0;//RTC_C3_on;
	if(disp_buffer[2] & 0x08) final_buffer[27] |= BIT0;//RTC_D3_on;
	if(disp_buffer[2] & 0x10) final_buffer[28] |= BIT0;//RTC_E3_on;
	if(disp_buffer[2] & 0x20) final_buffer[29] |= BIT0;//RTC_F3_on;
	if(disp_buffer[2] & 0x40) final_buffer[30] |= BIT0;//RTC_G3_on;

	if(disp_buffer[3] & 0x01) final_buffer[16] |= BIT0;//RTC_A4_on;
	if(disp_buffer[3] & 0x02) final_buffer[17] |= BIT0;//RTC_B4_on;
	if(disp_buffer[3] & 0x04) final_buffer[18] |= BIT0;//RTC_C4_on;
	if(disp_buffer[3] & 0x08) final_buffer[19] |= BIT0;//RTC_D4_on;
	if(disp_buffer[3] & 0x10) final_buffer[20] |= BIT0;//RTC_E4_on;
	if(disp_buffer[3] & 0x20) final_buffer[21] |= BIT0;//RTC_F4_on;
	if(disp_buffer[3] & 0x40) final_buffer[22] |= BIT0;//RTC_G4_on;
	
	if(lcd.Sym_DP_UNIT) DP_UNIT_on;
	
	if(DP2_Alrm_ON==NO_ALARM)
	{
		if(lcd.Sym_DP_MIN) DP_MIN_on;
		if(lcd.Sym_DP_LOGO) DP_LOGO_on;
		
		if(disp_buffer[4] & 0x01) final_buffer[24] |= BIT4;//DP_A1_on;
		if(disp_buffer[4] & 0x02) final_buffer[25] |= BIT4;//DP_B1_on;
		if(disp_buffer[4] & 0x04) final_buffer[26] |= BIT4;//DP_C1_on;
		if(disp_buffer[4] & 0x08) final_buffer[27] |= BIT4;//DP_D1_on;
		if(disp_buffer[4] & 0x10) final_buffer[28] |= BIT4;//DP_E1_on;
		if(disp_buffer[4] & 0x20) final_buffer[29] |= BIT4;//DP_F1_on;
		if(disp_buffer[4] & 0x40) final_buffer[30] |= BIT4;//DP_G1_on;

		if(disp_buffer[5] & 0x01) final_buffer[16] |= BIT4;//DP_A2_on;
		if(disp_buffer[5] & 0x02) final_buffer[17] |= BIT4;//DP_B2_on;
		if(disp_buffer[5] & 0x04) final_buffer[18] |= BIT4;//DP_C2_on;
		if(disp_buffer[5] & 0x08) final_buffer[19] |= BIT4;//DP_D2_on;
		if(disp_buffer[5] & 0x10) final_buffer[20] |= BIT4;//DP_E2_on;
		if(disp_buffer[5] & 0x20) final_buffer[21] |= BIT4;//DP_F2_on;
		if(disp_buffer[5] & 0x40) final_buffer[22] |= BIT4;//DP_G2_on;
		if(disp_buffer[5] & 0x80) final_buffer[23] |= BIT4;//DP_H2_on;

		if(disp_buffer[6] & 0x01) final_buffer[16] |= BIT2;//DP_A3_on;
		if(disp_buffer[6] & 0x02) final_buffer[17] |= BIT2;//DP_B3_on;
		if(disp_buffer[6] & 0x04) final_buffer[18] |= BIT2;//DP_C3_on;
		if(disp_buffer[6] & 0x08) final_buffer[19] |= BIT2;//DP_D3_on;
		if(disp_buffer[6] & 0x10) final_buffer[20] |= BIT2;//DP_E3_on;
		if(disp_buffer[6] & 0x20) final_buffer[21] |= BIT2;//DP_F3_on;
		if(disp_buffer[6] & 0x40) final_buffer[22] |= BIT2;//DP_G3_on;
	}
	else if(DP2_Alrm_ON==LOWER_ALARM)
	{
		if(lcd.Sym_DP_MIN_ALM) DP_MIN_ALM_on;
		if(lcd.Sym_DP_LOGO_ALM) DP_LOGO_ALM_on;
		
		if(disp_buffer[4] & 0x01) final_buffer[24] |= BIT5;//DP_A1_on;
		if(disp_buffer[4] & 0x02) final_buffer[25] |= BIT5;//DP_B1_on;
		if(disp_buffer[4] & 0x04) final_buffer[26] |= BIT5;//DP_C1_on;
		if(disp_buffer[4] & 0x08) final_buffer[27] |= BIT5;//DP_D1_on;
		if(disp_buffer[4] & 0x10) final_buffer[28] |= BIT5;//DP_E1_on;
		if(disp_buffer[4] & 0x20) final_buffer[29] |= BIT5;//DP_F1_on;
		if(disp_buffer[4] & 0x40) final_buffer[30] |= BIT5;//DP_G1_on;

		if(disp_buffer[5] & 0x01) final_buffer[16] |= BIT5;//DP_A2_on;
		if(disp_buffer[5] & 0x02) final_buffer[17] |= BIT5;//DP_B2_on;
		if(disp_buffer[5] & 0x04) final_buffer[18] |= BIT5;//DP_C2_on;
		if(disp_buffer[5] & 0x08) final_buffer[19] |= BIT5;//DP_D2_on;
		if(disp_buffer[5] & 0x10) final_buffer[20] |= BIT5;//DP_E2_on;
		if(disp_buffer[5] & 0x20) final_buffer[21] |= BIT5;//DP_F2_on;
		if(disp_buffer[5] & 0x40) final_buffer[22] |= BIT5;//DP_G2_on;
		if(disp_buffer[5] & 0x80) final_buffer[23] |= BIT5;//DP_H2_on;

		if(disp_buffer[6] & 0x01) final_buffer[16] |= BIT3;//DP_A3_on;
		if(disp_buffer[6] & 0x02) final_buffer[17] |= BIT3;//DP_B3_on;
		if(disp_buffer[6] & 0x04) final_buffer[18] |= BIT3;//DP_C3_on;
		if(disp_buffer[6] & 0x08) final_buffer[19] |= BIT3;//DP_D3_on;
		if(disp_buffer[6] & 0x10) final_buffer[20] |= BIT3;//DP_E3_on;
		if(disp_buffer[6] & 0x20) final_buffer[21] |= BIT3;//DP_F3_on;
		if(disp_buffer[6] & 0x40) final_buffer[22] |= BIT3;//DP_G3_on;
	}
	else
	{
		if(lcd.Sym_DP_MIN_ALM) 
		{
			DP_MIN_on;	
			DP_MIN_ALM_on;
		}
		if(lcd.Sym_DP_LOGO_ALM) DP_LOGO_ALM_on;
		
		if(disp_buffer[4] & 0x01) final_buffer[24] |= BIT4;//DP_A1_on;
		if(disp_buffer[4] & 0x02) final_buffer[25] |= BIT4;//DP_B1_on;
		if(disp_buffer[4] & 0x04) final_buffer[26] |= BIT4;//DP_C1_on;
		if(disp_buffer[4] & 0x08) final_buffer[27] |= BIT4;//DP_D1_on;
		if(disp_buffer[4] & 0x10) final_buffer[28] |= BIT4;//DP_E1_on;
		if(disp_buffer[4] & 0x20) final_buffer[29] |= BIT4;//DP_F1_on;
		if(disp_buffer[4] & 0x40) final_buffer[30] |= BIT4;//DP_G1_on;

		if(disp_buffer[5] & 0x01) final_buffer[16] |= BIT4;//DP_A2_on;
		if(disp_buffer[5] & 0x02) final_buffer[17] |= BIT4;//DP_B2_on;
		if(disp_buffer[5] & 0x04) final_buffer[18] |= BIT4;//DP_C2_on;
		if(disp_buffer[5] & 0x08) final_buffer[19] |= BIT4;//DP_D2_on;
		if(disp_buffer[5] & 0x10) final_buffer[20] |= BIT4;//DP_E2_on;
		if(disp_buffer[5] & 0x20) final_buffer[21] |= BIT4;//DP_F2_on;
		if(disp_buffer[5] & 0x40) final_buffer[22] |= BIT4;//DP_G2_on;
		if(disp_buffer[5] & 0x80) final_buffer[23] |= BIT4;//DP_H2_on;

		if(disp_buffer[6] & 0x01) final_buffer[16] |= BIT2;//DP_A3_on;
		if(disp_buffer[6] & 0x02) final_buffer[17] |= BIT2;//DP_B3_on;
		if(disp_buffer[6] & 0x04) final_buffer[18] |= BIT2;//DP_C3_on;
		if(disp_buffer[6] & 0x08) final_buffer[19] |= BIT2;//DP_D3_on;
		if(disp_buffer[6] & 0x10) final_buffer[20] |= BIT2;//DP_E3_on;
		if(disp_buffer[6] & 0x20) final_buffer[21] |= BIT2;//DP_F3_on;
		if(disp_buffer[6] & 0x40) final_buffer[22] |= BIT2;//DP_G3_on;
		
		if(disp_buffer[4] & 0x01) final_buffer[24] |= BIT5;//DP_A1_on;
		if(disp_buffer[4] & 0x02) final_buffer[25] |= BIT5;//DP_B1_on;
		if(disp_buffer[4] & 0x04) final_buffer[26] |= BIT5;//DP_C1_on;
		if(disp_buffer[4] & 0x08) final_buffer[27] |= BIT5;//DP_D1_on;
		if(disp_buffer[4] & 0x10) final_buffer[28] |= BIT5;//DP_E1_on;
		if(disp_buffer[4] & 0x20) final_buffer[29] |= BIT5;//DP_F1_on;
		if(disp_buffer[4] & 0x40) final_buffer[30] |= BIT5;//DP_G1_on;

		if(disp_buffer[5] & 0x01) final_buffer[16] |= BIT5;//DP_A2_on;
		if(disp_buffer[5] & 0x02) final_buffer[17] |= BIT5;//DP_B2_on;
		if(disp_buffer[5] & 0x04) final_buffer[18] |= BIT5;//DP_C2_on;
		if(disp_buffer[5] & 0x08) final_buffer[19] |= BIT5;//DP_D2_on;
		if(disp_buffer[5] & 0x10) final_buffer[20] |= BIT5;//DP_E2_on;
		if(disp_buffer[5] & 0x20) final_buffer[21] |= BIT5;//DP_F2_on;
		if(disp_buffer[5] & 0x40) final_buffer[22] |= BIT5;//DP_G2_on;
		if(disp_buffer[5] & 0x80) final_buffer[23] |= BIT5;//DP_H2_on;

		if(disp_buffer[6] & 0x01) final_buffer[16] |= BIT3;//DP_A3_on;
		if(disp_buffer[6] & 0x02) final_buffer[17] |= BIT3;//DP_B3_on;
		if(disp_buffer[6] & 0x04) final_buffer[18] |= BIT3;//DP_C3_on;
		if(disp_buffer[6] & 0x08) final_buffer[19] |= BIT3;//DP_D3_on;
		if(disp_buffer[6] & 0x10) final_buffer[20] |= BIT3;//DP_E3_on;
		if(disp_buffer[6] & 0x20) final_buffer[21] |= BIT3;//DP_F3_on;
		if(disp_buffer[6] & 0x40) final_buffer[22] |= BIT3;//DP_G3_on;
	}
	
	if(lcd.Sym_TM_UNIT_C) TM_UNIT_C_on;
	if(lcd.Sym_TM_UNIT_F) TM_UNIT_F_on;
	
	if(TM_Alrm_ON==NO_ALARM)
	{
		if(lcd.Sym_TM_MIN) TM_MIN_on;
		if(lcd.Sym_TM_LOGO) TM_LOGO_on;
		
		if(disp_buffer[7] & 0x01) final_buffer[8] |= BIT2;//TM_A1_on;
		if(disp_buffer[7] & 0x02) final_buffer[9] |= BIT2;//TM_B1_on;
		if(disp_buffer[7] & 0x04) final_buffer[10] |= BIT2;//TM_C1_on;
		if(disp_buffer[7] & 0x08) final_buffer[11] |= BIT2;//TM_D1_on;
		if(disp_buffer[7] & 0x10) final_buffer[12] |= BIT2;//TM_E1_on;
		if(disp_buffer[7] & 0x20) final_buffer[13] |= BIT2;//TM_F1_on;
		if(disp_buffer[7] & 0x40) final_buffer[14] |= BIT2;//TM_G1_on;

		if(disp_buffer[8] & 0x01) final_buffer[0] |= BIT4;//TM_A2_on;
		if(disp_buffer[8] & 0x02) final_buffer[1] |= BIT4;//TM_B2_on;
		if(disp_buffer[8] & 0x04) final_buffer[2] |= BIT4;//TM_C2_on;
		if(disp_buffer[8] & 0x08) final_buffer[3] |= BIT4;//TM_D2_on;
		if(disp_buffer[8] & 0x10) final_buffer[4] |= BIT4;//TM_E2_on;
		if(disp_buffer[8] & 0x20) final_buffer[5] |= BIT4;//TM_F2_on;
		if(disp_buffer[8] & 0x40) final_buffer[6] |= BIT4;//TM_G2_on;
		if(disp_buffer[8] & 0x80) final_buffer[7] |= BIT4;//TM_H2_on;

		if(disp_buffer[9] & 0x01) final_buffer[8] |= BIT4;//TM_A3_on;
		if(disp_buffer[9] & 0x02) final_buffer[9] |= BIT4;//TM_B3_on;
		if(disp_buffer[9] & 0x04) final_buffer[10] |= BIT4;//TM_C3_on;
		if(disp_buffer[9] & 0x08) final_buffer[11] |= BIT4;//TM_D3_on;
		if(disp_buffer[9] & 0x10) final_buffer[12] |= BIT4;//TM_E3_on;
		if(disp_buffer[9] & 0x20) final_buffer[13] |= BIT4;//TM_F3_on;
		if(disp_buffer[9] & 0x40) final_buffer[14] |= BIT4;//TM_G3_on;
	}
	else if(TM_Alrm_ON==UPPER_ALARM)
	{
		if(lcd.Sym_TM_MIN_ALM) TM_MIN_ALM_on;
		if(lcd.Sym_TM_LOGO_ALM) TM_LOGO_ALM_on;
		
		if(disp_buffer[7] & 0x01) final_buffer[8] |= BIT3;//TM_A1_on;
		if(disp_buffer[7] & 0x02) final_buffer[9] |= BIT3;//TM_B1_on;
		if(disp_buffer[7] & 0x04) final_buffer[10] |= BIT3;//TM_C1_on;
		if(disp_buffer[7] & 0x08) final_buffer[11] |= BIT3;//TM_D1_on;
		if(disp_buffer[7] & 0x10) final_buffer[12] |= BIT3;//TM_E1_on;
		if(disp_buffer[7] & 0x20) final_buffer[13] |= BIT3;//TM_F1_on;
		if(disp_buffer[7] & 0x40) final_buffer[14] |= BIT3;//TM_G1_on;

		if(disp_buffer[8] & 0x01) final_buffer[0] |= BIT5;//TM_A2_on;
		if(disp_buffer[8] & 0x02) final_buffer[1] |= BIT5;//TM_B2_on;
		if(disp_buffer[8] & 0x04) final_buffer[2] |= BIT5;//TM_C2_on;
		if(disp_buffer[8] & 0x08) final_buffer[3] |= BIT5;//TM_D2_on;
		if(disp_buffer[8] & 0x10) final_buffer[4] |= BIT5;//TM_E2_on;
		if(disp_buffer[8] & 0x20) final_buffer[5] |= BIT5;//TM_F2_on;
		if(disp_buffer[8] & 0x40) final_buffer[6] |= BIT5;//TM_G2_on;
		if(disp_buffer[8] & 0x80) final_buffer[7] |= BIT5;//TM_H2_on;

		if(disp_buffer[9] & 0x01) final_buffer[8] |= BIT5;//TM_A3_on;
		if(disp_buffer[9] & 0x02) final_buffer[9] |= BIT5;//TM_B3_on;
		if(disp_buffer[9] & 0x04) final_buffer[10] |= BIT5;//TM_C3_on;
		if(disp_buffer[9] & 0x08) final_buffer[11] |= BIT5;//TM_D3_on;
		if(disp_buffer[9] & 0x10) final_buffer[12] |= BIT5;//TM_E3_on;
		if(disp_buffer[9] & 0x20) final_buffer[13] |= BIT5;//TM_F3_on;
		if(disp_buffer[9] & 0x40) final_buffer[14] |= BIT5;//TM_G3_on;
	}
	else
	{
		if(lcd.Sym_TM_MIN) 
		{
			TM_MIN_on;
			TM_MIN_ALM_on;
		}
		if(lcd.Sym_TM_LOGO_ALM) TM_LOGO_ALM_on;
		
		if(disp_buffer[7] & 0x01) final_buffer[8] |= BIT2;//TM_A1_on;
		if(disp_buffer[7] & 0x02) final_buffer[9] |= BIT2;//TM_B1_on;
		if(disp_buffer[7] & 0x04) final_buffer[10] |= BIT2;//TM_C1_on;
		if(disp_buffer[7] & 0x08) final_buffer[11] |= BIT2;//TM_D1_on;
		if(disp_buffer[7] & 0x10) final_buffer[12] |= BIT2;//TM_E1_on;
		if(disp_buffer[7] & 0x20) final_buffer[13] |= BIT2;//TM_F1_on;
		if(disp_buffer[7] & 0x40) final_buffer[14] |= BIT2;//TM_G1_on;

		if(disp_buffer[8] & 0x01) final_buffer[0] |= BIT4;//TM_A2_on;
		if(disp_buffer[8] & 0x02) final_buffer[1] |= BIT4;//TM_B2_on;
		if(disp_buffer[8] & 0x04) final_buffer[2] |= BIT4;//TM_C2_on;
		if(disp_buffer[8] & 0x08) final_buffer[3] |= BIT4;//TM_D2_on;
		if(disp_buffer[8] & 0x10) final_buffer[4] |= BIT4;//TM_E2_on;
		if(disp_buffer[8] & 0x20) final_buffer[5] |= BIT4;//TM_F2_on;
		if(disp_buffer[8] & 0x40) final_buffer[6] |= BIT4;//TM_G2_on;
		if(disp_buffer[8] & 0x80) final_buffer[7] |= BIT4;//TM_H2_on;

		if(disp_buffer[9] & 0x01) final_buffer[8] |= BIT4;//TM_A3_on;
		if(disp_buffer[9] & 0x02) final_buffer[9] |= BIT4;//TM_B3_on;
		if(disp_buffer[9] & 0x04) final_buffer[10] |= BIT4;//TM_C3_on;
		if(disp_buffer[9] & 0x08) final_buffer[11] |= BIT4;//TM_D3_on;
		if(disp_buffer[9] & 0x10) final_buffer[12] |= BIT4;//TM_E3_on;
		if(disp_buffer[9] & 0x20) final_buffer[13] |= BIT4;//TM_F3_on;
		if(disp_buffer[9] & 0x40) final_buffer[14] |= BIT4;//TM_G3_on;
		
		if(disp_buffer[7] & 0x01) final_buffer[8] |= BIT3;//TM_A1_on;
		if(disp_buffer[7] & 0x02) final_buffer[9] |= BIT3;//TM_B1_on;
		if(disp_buffer[7] & 0x04) final_buffer[10] |= BIT3;//TM_C1_on;
		if(disp_buffer[7] & 0x08) final_buffer[11] |= BIT3;//TM_D1_on;
		if(disp_buffer[7] & 0x10) final_buffer[12] |= BIT3;//TM_E1_on;
		if(disp_buffer[7] & 0x20) final_buffer[13] |= BIT3;//TM_F1_on;
		if(disp_buffer[7] & 0x40) final_buffer[14] |= BIT3;//TM_G1_on;

		if(disp_buffer[8] & 0x01) final_buffer[0] |= BIT5;//TM_A2_on;
		if(disp_buffer[8] & 0x02) final_buffer[1] |= BIT5;//TM_B2_on;
		if(disp_buffer[8] & 0x04) final_buffer[2] |= BIT5;//TM_C2_on;
		if(disp_buffer[8] & 0x08) final_buffer[3] |= BIT5;//TM_D2_on;
		if(disp_buffer[8] & 0x10) final_buffer[4] |= BIT5;//TM_E2_on;
		if(disp_buffer[8] & 0x20) final_buffer[5] |= BIT5;//TM_F2_on;
		if(disp_buffer[8] & 0x40) final_buffer[6] |= BIT5;//TM_G2_on;
		if(disp_buffer[8] & 0x80) final_buffer[7] |= BIT5;//TM_H2_on;

		if(disp_buffer[9] & 0x01) final_buffer[8] |= BIT5;//TM_A3_on;
		if(disp_buffer[9] & 0x02) final_buffer[9] |= BIT5;//TM_B3_on;
		if(disp_buffer[9] & 0x04) final_buffer[10] |= BIT5;//TM_C3_on;
		if(disp_buffer[9] & 0x08) final_buffer[11] |= BIT5;//TM_D3_on;
		if(disp_buffer[9] & 0x10) final_buffer[12] |= BIT5;//TM_E3_on;
		if(disp_buffer[9] & 0x20) final_buffer[13] |= BIT5;//TM_F3_on;
		if(disp_buffer[9] & 0x40) final_buffer[14] |= BIT5;//TM_G3_on;
	}

	if(lcd.Sym_RH_UNIT) RH_UNIT_on;
	
	if(RH_Alrm_ON==NO_ALARM)
	{
		if(lcd.Sym_RH_MIN) RH_MIN_on;
		if(lcd.Sym_RH_LOGO) RH_LOGO_on;
		
		if(disp_buffer[10] & 0x01) final_buffer[0] |= BIT0;//RH_A1_on;
		if(disp_buffer[10] & 0x02) final_buffer[1] |= BIT0;//RH_B1_on;
		if(disp_buffer[10] & 0x04) final_buffer[2] |= BIT0;//RH_C1_on;
		if(disp_buffer[10] & 0x08) final_buffer[3] |= BIT0;//RH_D1_on;
		if(disp_buffer[10] & 0x10) final_buffer[4] |= BIT0;//RH_E1_on;
		if(disp_buffer[10] & 0x20) final_buffer[5] |= BIT0;//RH_F1_on;
		if(disp_buffer[10] & 0x40) final_buffer[6] |= BIT0;//RH_G1_on;

		if(disp_buffer[11] & 0x01) final_buffer[8] |= BIT0;//RH_A2_on;
		if(disp_buffer[11] & 0x02) final_buffer[9] |= BIT0;//RH_B2_on;
		if(disp_buffer[11] & 0x04) final_buffer[10] |= BIT0;//RH_C2_on;
		if(disp_buffer[11] & 0x08) final_buffer[11] |= BIT0;//RH_D2_on;
		if(disp_buffer[11] & 0x10) final_buffer[12] |= BIT0;//RH_E2_on;
		if(disp_buffer[11] & 0x20) final_buffer[13] |= BIT0;//RH_F2_on;
		if(disp_buffer[11] & 0x40) final_buffer[14] |= BIT0;//RH_G2_on;
		if(disp_buffer[11] & 0x80) final_buffer[15] |= BIT0;//RH_H2_on;

		if(disp_buffer[12] & 0x01) final_buffer[0] |= BIT2;//RH_A3_on;
		if(disp_buffer[12] & 0x02) final_buffer[1] |= BIT2;//RH_B3_on;
		if(disp_buffer[12] & 0x04) final_buffer[2] |= BIT2;//RH_C3_on;
		if(disp_buffer[12] & 0x08) final_buffer[3] |= BIT2;//RH_D3_on;
		if(disp_buffer[12] & 0x10) final_buffer[4] |= BIT2;//RH_E3_on;
		if(disp_buffer[12] & 0x20) final_buffer[5] |= BIT2;//RH_F3_on;
		if(disp_buffer[12] & 0x40) final_buffer[6] |= BIT2;//RH_G3_on;
	}
	else if(RH_Alrm_ON==UPPER_ALARM)
	{
		if(lcd.Sym_RH_MIN_ALM) RH_MIN_ALM_on;
		if(lcd.Sym_RH_LOGO_ALM) RH_LOGO_ALM_on;
		
		if(disp_buffer[10] & 0x01) final_buffer[0] |= BIT1;//RH_A1_on;
		if(disp_buffer[10] & 0x02) final_buffer[1] |= BIT1;//RH_B1_on;
		if(disp_buffer[10] & 0x04) final_buffer[2] |= BIT1;//RH_C1_on;
		if(disp_buffer[10] & 0x08) final_buffer[3] |= BIT1;//RH_D1_on;
		if(disp_buffer[10] & 0x10) final_buffer[4] |= BIT1;//RH_E1_on;
		if(disp_buffer[10] & 0x20) final_buffer[5] |= BIT1;//RH_F1_on;
		if(disp_buffer[10] & 0x40) final_buffer[6] |= BIT1;//RH_G1_on;

		if(disp_buffer[11] & 0x01) final_buffer[8] |= BIT1;//RH_A2_on;
		if(disp_buffer[11] & 0x02) final_buffer[9] |= BIT1;//RH_B2_on;
		if(disp_buffer[11] & 0x04) final_buffer[10] |= BIT1;//RH_C2_on;
		if(disp_buffer[11] & 0x08) final_buffer[11] |= BIT1;//RH_D2_on;
		if(disp_buffer[11] & 0x10) final_buffer[12] |= BIT1;//RH_E2_on;
		if(disp_buffer[11] & 0x20) final_buffer[13] |= BIT1;//RH_F2_on;
		if(disp_buffer[11] & 0x40) final_buffer[14] |= BIT1;//RH_G2_on;
		if(disp_buffer[11] & 0x80) final_buffer[15] |= BIT1;//RH_H2_on;

		if(disp_buffer[12] & 0x01) final_buffer[0] |= BIT3;//RH_A3_on;
		if(disp_buffer[12] & 0x02) final_buffer[1] |= BIT3;//RH_B3_on;
		if(disp_buffer[12] & 0x04) final_buffer[2] |= BIT3;//RH_C3_on;
		if(disp_buffer[12] & 0x08) final_buffer[3] |= BIT3;//RH_D3_on;
		if(disp_buffer[12] & 0x10) final_buffer[4] |= BIT3;//RH_E3_on;
		if(disp_buffer[12] & 0x20) final_buffer[5] |= BIT3;//RH_F3_on;
		if(disp_buffer[12] & 0x40) final_buffer[6] |= BIT3;//RH_G3_on;
	}	
	else
	{
		if(lcd.Sym_RH_MIN) 
		{
			RH_MIN_on;
			RH_MIN_ALM_on;
		}
		if(lcd.Sym_RH_LOGO_ALM) RH_LOGO_ALM_on;
		
		if(disp_buffer[10] & 0x01) final_buffer[0] |= BIT0;//RH_A1_on;
		if(disp_buffer[10] & 0x02) final_buffer[1] |= BIT0;//RH_B1_on;
		if(disp_buffer[10] & 0x04) final_buffer[2] |= BIT0;//RH_C1_on;
		if(disp_buffer[10] & 0x08) final_buffer[3] |= BIT0;//RH_D1_on;
		if(disp_buffer[10] & 0x10) final_buffer[4] |= BIT0;//RH_E1_on;
		if(disp_buffer[10] & 0x20) final_buffer[5] |= BIT0;//RH_F1_on;
		if(disp_buffer[10] & 0x40) final_buffer[6] |= BIT0;//RH_G1_on;

		if(disp_buffer[11] & 0x01) final_buffer[8] |= BIT0;//RH_A2_on;
		if(disp_buffer[11] & 0x02) final_buffer[9] |= BIT0;//RH_B2_on;
		if(disp_buffer[11] & 0x04) final_buffer[10] |= BIT0;//RH_C2_on;
		if(disp_buffer[11] & 0x08) final_buffer[11] |= BIT0;//RH_D2_on;
		if(disp_buffer[11] & 0x10) final_buffer[12] |= BIT0;//RH_E2_on;
		if(disp_buffer[11] & 0x20) final_buffer[13] |= BIT0;//RH_F2_on;
		if(disp_buffer[11] & 0x40) final_buffer[14] |= BIT0;//RH_G2_on;
		if(disp_buffer[11] & 0x80) final_buffer[15] |= BIT0;//RH_H2_on;

		if(disp_buffer[12] & 0x01) final_buffer[0] |= BIT2;//RH_A3_on;
		if(disp_buffer[12] & 0x02) final_buffer[1] |= BIT2;//RH_B3_on;
		if(disp_buffer[12] & 0x04) final_buffer[2] |= BIT2;//RH_C3_on;
		if(disp_buffer[12] & 0x08) final_buffer[3] |= BIT2;//RH_D3_on;
		if(disp_buffer[12] & 0x10) final_buffer[4] |= BIT2;//RH_E3_on;
		if(disp_buffer[12] & 0x20) final_buffer[5] |= BIT2;//RH_F3_on;
		if(disp_buffer[12] & 0x40) final_buffer[6] |= BIT2;//RH_G3_on;
		
		if(disp_buffer[10] & 0x01) final_buffer[0] |= BIT1;//RH_A1_on;
		if(disp_buffer[10] & 0x02) final_buffer[1] |= BIT1;//RH_B1_on;
		if(disp_buffer[10] & 0x04) final_buffer[2] |= BIT1;//RH_C1_on;
		if(disp_buffer[10] & 0x08) final_buffer[3] |= BIT1;//RH_D1_on;
		if(disp_buffer[10] & 0x10) final_buffer[4] |= BIT1;//RH_E1_on;
		if(disp_buffer[10] & 0x20) final_buffer[5] |= BIT1;//RH_F1_on;
		if(disp_buffer[10] & 0x40) final_buffer[6] |= BIT1;//RH_G1_on;

		if(disp_buffer[11] & 0x01) final_buffer[8] |= BIT1;//RH_A2_on;
		if(disp_buffer[11] & 0x02) final_buffer[9] |= BIT1;//RH_B2_on;
		if(disp_buffer[11] & 0x04) final_buffer[10] |= BIT1;//RH_C2_on;
		if(disp_buffer[11] & 0x08) final_buffer[11] |= BIT1;//RH_D2_on;
		if(disp_buffer[11] & 0x10) final_buffer[12] |= BIT1;//RH_E2_on;
		if(disp_buffer[11] & 0x20) final_buffer[13] |= BIT1;//RH_F2_on;
		if(disp_buffer[11] & 0x40) final_buffer[14] |= BIT1;//RH_G2_on;
		if(disp_buffer[11] & 0x80) final_buffer[15] |= BIT1;//RH_H2_on;

		if(disp_buffer[12] & 0x01) final_buffer[0] |= BIT3;//RH_A3_on;
		if(disp_buffer[12] & 0x02) final_buffer[1] |= BIT3;//RH_B3_on;
		if(disp_buffer[12] & 0x04) final_buffer[2] |= BIT3;//RH_C3_on;
		if(disp_buffer[12] & 0x08) final_buffer[3] |= BIT3;//RH_D3_on;
		if(disp_buffer[12] & 0x10) final_buffer[4] |= BIT3;//RH_E3_on;
		if(disp_buffer[12] & 0x20) final_buffer[5] |= BIT3;//RH_F3_on;
		if(disp_buffer[12] & 0x40) final_buffer[6] |= BIT3;//RH_G3_on;
	}
	
	if(lcd.Sym_DOOR) DOOR_on;
	if(lcd.Sym_DOOR_SYM) DOOR_SYM_on;
	if(lcd.Sym_DOOR_ALM) DOOR_ALM_on;
	if(lcd.Sym_LOGO) LOGO_on;
	if(lcd.Sym_MIN) MIN_on;
	if(lcd.Sym_MAX) MAX_on;
	if(lcd.Sym_MEAN) MEAN_on;
	if(lcd.Sym_SET) SET_on;
	if(lcd.Sym_ID) ID_on;
	if(lcd.Sym_ACK) ACK_on;
	
	for(uint8_t i=0;i<32;i++) final_buffer[i] = changeNibbles(final_buffer[i]);
	TM1680WritePage(0x00, final_buffer, sizeof(final_buffer));	
}
	
void conv_value(void)
{
	for(uint8_t i=1;i<NO_DIGIT;i++) data[i] = BLANK;
	
	lcd.Sym_DOOR = 0;
	lcd.Sym_DOOR_SYM = 0;
	lcd.Sym_DOOR_ALM = 0;
	lcd.Sym_LOGO = 0;
	lcd.Sym_MIN = 0;
	lcd.Sym_MAX = 0;
	lcd.Sym_MEAN = 0;
	lcd.Sym_SET = 0;
	lcd.Sym_ID = 0;
	lcd.Sym_ACK = 0;
	lcd.Sym_RTC_AM = 0;
	lcd.Sym_RTC_PM = 0;
	lcd.Sym_RTC_BC1 = 0;
	lcd.Sym_RTC_COL = 0;

	lcd.Sym_DP_LOGO = 0;
	lcd.Sym_DP_LOGO_ALM = 0;
	lcd.Sym_DP_UNIT = 0;
	lcd.Sym_DP_MIN = 0;
	lcd.Sym_DP_MIN_ALM = 0;
	
	lcd.Sym_RH_LOGO = 0;
	lcd.Sym_RH_LOGO_ALM = 0;
	lcd.Sym_RH_UNIT = 0;
	lcd.Sym_RH_MIN = 0;
	lcd.Sym_RH_MIN_ALM = 0;
	
	lcd.Sym_TM_LOGO = 0;
	lcd.Sym_TM_LOGO_ALM = 0;
	lcd.Sym_TM_UNIT_C = 0;
	lcd.Sym_TM_UNIT_F = 0;
	lcd.Sym_TM_MIN = 0;
	lcd.Sym_TM_MIN_ALM = 0;
	
	if(gu16_parameterWord & ENABLE_LOGO)
	{
		lcd.Sym_LOGO = 1;
	}
	
	#ifndef DISABLE_DOOR_SENSING
	lcd.Sym_DOOR_SYM = 1;
	if(b.doorStatus==OPEN)
	{
		lcd.Sym_DOOR_ALM = 1;
	}
	else
	{
		lcd.Sym_DOOR = 1;
	}
	#endif
	
	switch(mode)
	{
		case NORMAL_MODE:
		
			if(gu16_parameterWord & ENABLE_RTC)
			{
				if(!rtcValid)
				{
					//if(b.mec500_blink_flag)
					{
						convert_char(rtc.minute,&data[2],2);

						if(b.AM_PM_Flag)
						{
							lcd.Sym_RTC_AM = 1;
							convert_char(rtc.hour,&data[0],2);
						}
						else
						{
							lcd.Sym_RTC_PM = 1;
						
							if(rtc.hour>12)
							convert_char(rtc.hour-12,&data[0],2);
							else
							convert_char(rtc.hour,&data[0],2);
						}
						if(data[0] == 1) lcd.Sym_RTC_BC1 = 1;
					
						lcd.Sym_RTC_COL = 1;
					}
				}
				else
				{
					convert_char(rtc.minute,&data[2],2);
				
					if(b.AM_PM_Flag)
					{
						lcd.Sym_RTC_AM = 1;
						convert_char(rtc.hour,&data[0],2);
					}
					else
					{
						lcd.Sym_RTC_PM = 1;
					
						if(rtc.hour>12)
						convert_char(rtc.hour-12,&data[0],2);
						else
						convert_char(rtc.hour,&data[0],2);
					}
					if(data[0] == 1) lcd.Sym_RTC_BC1 = 1;
				
					if(b.Sec_blink_flag) lcd.Sym_RTC_COL = 1;
				}
			}
			
			switch(Normal_para_cnt)
			{
				case 0:
				
					if(gu16_parameterWord & ENABLE_DP2)
					{
						if(b.DP2_NC)
						{
							data[4]=E;
							data[5]=r;
							data[6]=r;
						}
						else
						{
							//----------------------------------------------------
							tempfloat = Dpressure2;
						
							if(!b.DP2_limit)
							{
								if(tempfloat<0.0)
								{
									tempfloat *= (-1.0);
									
									if(DP2_Alrm_ON) 
									{
										lcd.Sym_DP_MIN_ALM = 1;
									}
									else
									{
										lcd.Sym_DP_MIN = 1;
									}
								}
								//----------------------------------------------------
								if(tempfloat < 10.0)
								{
									convert_float(tempfloat,&data[5],1);
								}
								else if(tempfloat < 100.0)
								{
									convert_float(tempfloat,&data[4],1);
								}
								else
								{
									convert_float(tempfloat,&data[4],0);
								}
							}
							else if(b.DP2_limit==1)
							{
								data[5]=H;
								data[6]=I;
							}
							else if(b.DP2_limit==2)
							{
								data[5]=L;
								data[6]=0;
							}
							//----------------------------------------------------
						}
						lcd.Sym_DP_UNIT = 1;
						if(DP2_Alrm_ON) 
						{
							lcd.Sym_DP_LOGO_ALM = 1;
						}
						else
						{
							lcd.Sym_DP_LOGO = 1;
						}
					}
				
					if(gu16_parameterWord & ENABLE_TEMP)
					{
						if(b.RH_TEMP_NC)
						{
							data[7]=E;
							data[8]=r;
							data[9]=r;
						}
						else
						{
							//----------------------------------------------------
							if(!TM_Unit)
							{
								tempfloat = temperatureC;
							}
							else
							{
								tempfloat = temperatureF;
							}
						
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								
								if(TM_Alrm_ON) 
								{
									lcd.Sym_TM_MIN_ALM = 1;
								}
								else
								{
									lcd.Sym_TM_MIN = 1;
								}
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[8],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[7],1);
							}
							else
							{
								convert_float(99.9,&data[7],0);
							}
							//----------------------------------------------------
						}
						if(!TM_Unit)
						{
							lcd.Sym_TM_UNIT_C = 1;
						}
						else
						{
							lcd.Sym_TM_UNIT_F = 1;
						}
						
						if(TM_Alrm_ON) 
						{
							lcd.Sym_TM_LOGO_ALM = 1;
						}
						else
						{
							lcd.Sym_TM_LOGO = 1;
						}
					}

					if(gu16_parameterWord & ENABLE_RH)
					{
						if(b.RH_TEMP_NC)
						{
							data[10]=E;
							data[11]=r;
							data[12]=r;
						}
						else
						{
							//----------------------------------------------------
							tempfloat = humidityRH;
						
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								if(RH_Alrm_ON) 
								{
									lcd.Sym_RH_MIN_ALM = 1;
								}
								else
								{
									lcd.Sym_RH_MIN = 1;
								}
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[11],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[10],1);
							}
							else
							{
								convert_float(99.9,&data[10],0);
							}
							//----------------------------------------------------
						}
						
						if(RH_Alrm_ON) 
						{
							lcd.Sym_RH_LOGO_ALM = 1;
						}
						else
						{
							lcd.Sym_RH_LOGO = 1;
						}
						lcd.Sym_RH_UNIT = 1;
					}
				
				break;
				
				case 1:
				
					lcd.Sym_ID = 1;
					convert_char(DeviceID,&data[4],3);
				
				break;
				
				case 2:
				
					data[10] = B;
					data[11] = D;
					data[12] = r;
				
					switch(UART_BaudRate)
					{
						case BAUD_1200:		convert_char(1200,&data[4],4);		break;
						case BAUD_2400:		convert_char(2400,&data[4],4);		break;
						case BAUD_4800:		convert_char(4800,&data[4],4);		break;
						case BAUD_9600:		convert_char(9600,&data[4],4);		break;
						case BAUD_14400:	convert_char(14400,&data[4],5);		break;
						case BAUD_19200:	convert_char(19200,&data[4],5);		break;
						case BAUD_28800:	convert_char(28800,&data[4],5);		break;
						case BAUD_38400:	convert_char(38400,&data[4],5);		break;
						case BAUD_57600:	convert_char(57600,&data[4],5);		break;
						case BAUD_115200:	convert_float(115200,&data[1],0);	break;
					}
				
				break;
				
				case 3:
				
					lcd.Sym_MIN = 1;
				
					if(gu16_parameterWord & ENABLE_DP2)
					{
						if(b.DP2_NC)
						{
							data[4]=E;
							data[5]=r;
							data[6]=r;
						}
						else
						{
							//----------------------------------------------------
							tempfloat = DP2_Min;
						
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								lcd.Sym_DP_MIN = 1;
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[5],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[4],1);
							}
							else
							{
								convert_float(tempfloat,&data[4],0);
							}
						
							//----------------------------------------------------
						}
						lcd.Sym_DP_UNIT = 1;
						lcd.Sym_DP_LOGO = 1;
					}
				
					if(gu16_parameterWord & ENABLE_TEMP)
					{
						if(b.RH_TEMP_NC)
						{
							data[7]=E;
							data[8]=r;
							data[9]=r;
						}
						else
						{
							//----------------------------------------------------
							if(!TM_Unit)
							{
								tempfloat = TM_Min;
								lcd.Sym_TM_UNIT_C = 1;
							}
							else
							{
								tempfloat = (TM_Min * 1.8) + 32.0;
								lcd.Sym_TM_UNIT_F = 1;
							}
						
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								lcd.Sym_TM_MIN = 1;
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[8],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[7],1);
							}
							else
							{
								convert_float(99.9,&data[7],0);
							}
							//----------------------------------------------------
							if(TM_Alrm_ON) 
							{
								lcd.Sym_TM_LOGO_ALM = 1;
							}
							else
							{
								lcd.Sym_TM_LOGO = 1;
							}
						}
					}
					
					if(gu16_parameterWord & ENABLE_RH)
					{
						if(b.RH_TEMP_NC)
						{
							data[10]=E;
							data[11]=r;
							data[12]=r;
						}
						else
						{
							//----------------------------------------------------
							tempfloat = RH_Min;
							
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								lcd.Sym_RH_MIN = 1;
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[11],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[10],1);
							}
							else
							{
								convert_float(99.9,&data[10],0);
							}
							//----------------------------------------------------
						}
		
						lcd.Sym_RH_LOGO = 1;
						lcd.Sym_RH_UNIT = 1;
					}
					
				break;
					
				case 4:
					
					lcd.Sym_MAX = 1;
					
					if(gu16_parameterWord & ENABLE_DP2)
					{
						if(b.DP2_NC)
						{
							data[4]=E;
							data[5]=r;
							data[6]=r;
						}
						else
						{
							//----------------------------------------------------
							tempfloat = DP2_Max;
							
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								lcd.Sym_DP_MIN = 1;
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[5],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[4],1);
							}
							else
							{
								convert_float(tempfloat,&data[4],0);
							}
							
							//----------------------------------------------------
						}
						lcd.Sym_DP_UNIT = 1;
						lcd.Sym_DP_LOGO = 1;
					}
					
					if(gu16_parameterWord & ENABLE_TEMP)
					{
						if(b.RH_TEMP_NC)
						{
							data[7]=E;
							data[8]=r;
							data[9]=r;
						}
						else
						{
							//----------------------------------------------------
							if(!TM_Unit)
							{
								tempfloat = TM_Max;
							}
							else
							{
								tempfloat = (TM_Max * 1.8) + 32.0;
							}
							
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								lcd.Sym_TM_MIN = 1;
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[8],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[7],1);
							}
							else
							{
								convert_float(99.9,&data[7],0);
							}
							//----------------------------------------------------
						}
						if(!TM_Unit)
						{
							lcd.Sym_TM_UNIT_C = 1;
						}
						else
						{
							lcd.Sym_TM_UNIT_F = 1;
						}

						if(TM_Alrm_ON) 
						{
							lcd.Sym_TM_LOGO_ALM = 1;
						}
						else
						{
							lcd.Sym_TM_LOGO = 1;
						}
					}

					if(gu16_parameterWord & ENABLE_RH)
					{
						if(b.RH_TEMP_NC)
						{
							data[10]=E;
							data[11]=r;
							data[12]=r;
						}
						else
						{
							//----------------------------------------------------
							tempfloat = RH_Max;
							
							if(tempfloat<0.0)
							{
								tempfloat *= (-1.0);
								lcd.Sym_RH_MIN = 1;
							}
							//----------------------------------------------------
							if(tempfloat < 10.0)
							{
								convert_float(tempfloat,&data[11],1);
							}
							else if(tempfloat < 100.0)
							{
								convert_float(tempfloat,&data[10],1);
							}
							else
							{
								convert_float(99.9,&data[10],0);
							}
							//----------------------------------------------------
						}
						
						lcd.Sym_RH_LOGO = 1;
						lcd.Sym_RH_UNIT = 1;
					}
					
				break;
					
				case 5:
					
					lcd.Sym_ACK = 1;
					convert_char(dummy1,&data[4],2);
					if(b.SetACKPwd==2)
					{
						convert_char(dummy,&data[10],3);
					}
					
					if(b.SetACKPwd)lcd.Sym_SET = 1;
					
				break;
			}
			
		break;	
		
		case DP_AUTO_CAL_MODE:
		
			data[4] = A;
			data[5] = U;
			data[6] = t;
			
			data[7] = C;
			data[8] = A;
			data[9] = L;

			lcd.Sym_DP_LOGO = 1;
			
		break;
		
		case PROG_MODE:
		
			switch(prog_para_cnt)
			{
				case 0:
				
					data[4] = P;
					data[5] = r;
					data[6] = 9;
				
				break;
				
				case 1:
				
					data[4] = D;
					data[5] = V;
					data[6] = C;
				
					data[2] = 1;
					data[3] = D;
				
					convert_char(dummy,&data[7],3);
				
				break;
				
				case 2:
				
					data[4] = B;
					data[5] = C;
					data[6] = L;
				
					if(!dummy)
					{
						data[7] = 0;
						data[8] = F;
						data[9] = F;
					}
					else
					{
						data[7] = 0;
						data[8] = N;
					}
				
				break;
				
				case 3:
				
					data[1] = 5;
					data[2] = C;
					data[3] = N;
				
					data[4] = t;
					data[5] = M;
					data[6] = E;
				
					convert_char(dummy,&data[7],2);
				
				break;
				
				case 4:
				
					lcd.Sym_DP_LOGO = 1;
					lcd.Sym_DP_UNIT = 1;
					
					data[1] = 0;
					data[2] = N;
					
					data[10] = U;
					data[11] = P;
					
					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 5:
				
					lcd.Sym_DP_LOGO = 1;
					lcd.Sym_DP_UNIT = 1;
				
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					data[10] = U;
					data[11] = P;
	
					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 6:
				
					lcd.Sym_DP_LOGO = 1;
					lcd.Sym_DP_UNIT = 1;
				
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					data[10] = L;
					data[11] = 0;
				
					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 7:
				
					lcd.Sym_DP_LOGO = 1;
					lcd.Sym_DP_UNIT = 1;
				
					data[1] = 0;
					data[2] = N;
				
					data[10] = L;
					data[11] = 0;
				
					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 8:
				
					lcd.Sym_TM_LOGO = 1;
					
					if(!TM_Unit)
					{
						lcd.Sym_TM_UNIT_C = 1;
					}
					else
					{
						lcd.Sym_TM_UNIT_F = 1;
					}
				
					data[1] = 0;
					data[2] = N;
				
					data[10] = U;
					data[11] = P;
				
					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 9:
				
					lcd.Sym_TM_LOGO = 1;
					
					if(!TM_Unit)
					{
						lcd.Sym_TM_UNIT_C = 1;
					}
					else
					{
						lcd.Sym_TM_UNIT_F = 1;
					}
				
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					data[10] = U;
					data[11] = P;

					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 10:
				
					lcd.Sym_TM_LOGO = 1;
					
					if(!TM_Unit)
					{																					
						lcd.Sym_TM_UNIT_C = 1;
					}
					else
					{
						lcd.Sym_TM_UNIT_F = 1;
					}			
				
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					data[10] = L;
					data[11] = 0;

					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 11:
				
					lcd.Sym_TM_LOGO = 1;
				
					if(!TM_Unit)
					{
						lcd.Sym_TM_UNIT_C = 1;
					}
					else
					{
						lcd.Sym_TM_UNIT_F = 1;
					}
				
					data[1] = 0;
					data[2] = N;
				
					data[10] = L;
					data[11] = 0;
				
					if(dummy<0)
					{
						lcd.Sym_TM_MIN = 1;
						convert_char(-dummy,&data[6],4);
					}
					else
					{
						convert_char(dummy,&data[6],4);
					}
				
				break;
				
				case 12:
				
					data[7] = U;
					data[8] = N;
					data[9] = t;
				
					if(!dummy)
					{
						lcd.Sym_TM_UNIT_C = 1;
					}
					else
					{
						lcd.Sym_TM_UNIT_F = 1;
					}
				
				break;
				
				case 13:
				
					lcd.Sym_RH_LOGO = 1;
					lcd.Sym_RH_UNIT = 1;
				
					data[1] = 0;
					data[2] = N;
				
					data[10] = U;
					data[11] = P;
				
					convert_char(dummy,&data[6],4);
				
				break;
				
				case 14:
				
					lcd.Sym_RH_LOGO = 1;
					lcd.Sym_RH_UNIT = 1;
					
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					data[10] = U;
					data[11] = P;
	
					convert_char(dummy,&data[6],4);
				
				break;
				
				case 15:
				
					lcd.Sym_RH_LOGO = 1;
					lcd.Sym_RH_UNIT = 1;
				
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					data[10] = L;
					data[11] = 0;
				
					convert_char(dummy,&data[6],4);
				
				break;
				
				case 16:
				
					lcd.Sym_RH_LOGO = 1;
					lcd.Sym_RH_UNIT = 1;
				
					data[1] = 0;
					data[2] = N;
				
					data[10] = L;
					data[11] = 0;

					convert_char(dummy,&data[6],4);
				
				break;
				
				case 17:
				
					data[1] = r;
					data[2] = t;
					data[3] = C;
				
					data[4] = H;
					data[5] = r;
				
					convert_char(dummy,&data[7],2);
				
				break;
				
				case 18:
				
					data[1] = r;
					data[2] = t;
					data[3] = C;
				
					data[4] = M;
					data[5] = N;
				
					convert_char(dummy,&data[7],2);
				
				break;
				
				case 19:
				
					data[1] = r;
					data[2] = t;
					data[3] = C;
				
					data[4] = D;
					data[5] = t;
				
					convert_char(dummy,&data[7],2);
				
				break;
				
				case 20:
				
					data[1] = r;
					data[2] = t;
					data[3] = C;
				
					data[4] = M;
					data[5] = 0;
				
					convert_char(dummy,&data[7],2);
				
				break;
				
				case 21:
				
					data[1] = r;
					data[2] = t;
					data[3] = C;
				
					data[4] = Y;
					data[5] = r;
				
					convert_char(dummy,&data[7],2);
				
				break;
				
				case 22:
				
					data[4] = B;
					data[5] = 2;
					data[6] = r;
				
					data[2] = 0;
					data[3] = N;
				
					convert_char(dummy,&data[7],3);
				
				break;
				
				case 23:
				
					data[4] = B;
					data[5] = 2;
					data[6] = r;
				
					data[1] = 0;
					data[2] = F;
					data[3] = F;
				
					convert_char(dummy,&data[7],3);
				
				break;
				
				case 24:
				
					data[4] = L;
					data[5] = 0;
					data[6] = 9;
				
					data[1] = t;
					data[2] = M;
					data[3] = E;
				
					convert_char(dummy,&data[7],3);
				
				break;
				
				case 25:
				
					data[1] = U;
					data[2] = r;
					data[3] = t;
				
					data[10] = B;
					data[11] = D;
					data[12] = r;
				
					switch(dummy)
					{
						case BAUD_1200:		convert_char(1200,&data[4],4);		break;
						case BAUD_2400:		convert_char(2400,&data[4],4);		break;
						case BAUD_4800:		convert_char(4800,&data[4],4);		break;
						case BAUD_9600:		convert_char(9600,&data[4],4);		break;
						case BAUD_14400:	convert_char(14400,&data[4],5);		break;
						case BAUD_19200:	convert_char(19200,&data[4],5);		break;
						case BAUD_28800:	convert_char(28800,&data[4],5);		break;
						case BAUD_38400:	convert_char(38400,&data[4],5);		break;
						case BAUD_57600:	convert_char(57600,&data[4],5);		break;
						case BAUD_115200:	convert_float(115200,&data[4],6);	break;
					}

				break;
				
				case 26:
				
					data[1] = U;
					data[2] = r;
					data[3] = t;
				
					data[10] = B;
					data[11] = 1;
					data[12] = t;
				
					switch(dummy)
					{
						case DATABIT_5:		data[4]=5;		break;
						case DATABIT_6:		data[4]=6;		break;
						case DATABIT_7:		data[4]=7;		break;
						case DATABIT_8:		data[4]=8;		break;
					}

				break;
				
				case 27:
				
					data[1] = U;
					data[2] = r;
					data[3] = t;
				
					data[10] = P;
					data[11] = r;
					data[12] = t;
				
					switch(dummy)
					{
						case PARITY_NONE:		data[4]=N;	data[5]=0;					break;
						case PARITY_EVEN:		data[4]=E;	data[5]=V;	data[6]=N;		break;
						case PARITY_ODD:		data[4]=0;	data[5]=D;	data[6]=D;		break;
					}
				
				break;
				
				case 28:
				
					data[1] = U;
					data[2] = r;
					data[3] = t;
				
					data[10] = 5;
					data[11] = t;
					data[12] = P;
				
					switch(dummy)
					{
						case STOP_BIT_1:		data[4]=1;			break;
						case STOP_BIT_2:		data[4]=2;			break;
					}
				
				break;
				
				case 29:
				
					data[1] = C;
					data[2] = A;
					data[3] = L;
				
					convert_char(dummy,&data[4],3);
				
				break;

				case 30:
				
					if(!TM_Unit)
					{
						lcd.Sym_TM_UNIT_C = 1;
					}
					else
					{
						lcd.Sym_TM_UNIT_F = 1;
					}
				
					data[1] = C;
					data[2] = A;
					data[3] = L;
				
					if(b.RH_TEMP_NC)
					{
						data[7]=E;
						data[8]=r;
						data[9]=r;
					}
					else
					{
						//----------------------------------------------------
						if(dummy<0)
						{
							lcd.Sym_TM_MIN = 1;
							convert_char(-dummy,&data[6],4);
						}
						else
						{
							convert_char(dummy,&data[6],4);
						}
						//----------------------------------------------------
					}
				
				break;
				
				case 31:

					lcd.Sym_RH_LOGO = 1;
					lcd.Sym_RH_UNIT = 1;
					
					data[1] = C;
					data[2] = A;
					data[3] = L;
				
					if(b.RH_TEMP_NC)
					{
						data[7]=E;
						data[8]=r;
						data[9]=r;
					}
					else
					{
						//----------------------------------------------------
						convert_char(dummy,&data[6],4);
						//----------------------------------------------------
					}
				
				break;
			}
			
		break;
	}
}

void TMUnitChange(void)
{
	eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TEMP_UNIT,TM_Unit);
	
	if(!TM_Unit)
	{
		TM_Upper_Alm_ON = (TM_Upper_Alm_ON-320) / 1.8;
		TM_Upper_Alm_OFF = (TM_Upper_Alm_OFF-320) / 1.8;
		TM_Lower_Alm_ON = (TM_Lower_Alm_ON-320) / 1.8;
		TM_Lower_Alm_OFF = (TM_Lower_Alm_OFF-320) / 1.8;
		
		TM_Cal_Value_F = (TM_Cal_Value_F-320) / 1.8;
		TM_Cal_Value_C = (TM_Cal_Value_C-320) / 1.8;
	}
	else
	{
		TM_Upper_Alm_ON = (TM_Upper_Alm_ON * 1.8) + 320;
		TM_Upper_Alm_OFF = (TM_Upper_Alm_OFF * 1.8) + 320;
		TM_Lower_Alm_ON = (TM_Lower_Alm_ON * 1.8) + 320;
		TM_Lower_Alm_OFF = (TM_Lower_Alm_OFF * 1.8) + 320;
		
		TM_Cal_Value_F = ((float)TM_Cal_Value_F * 1.8) + 32.0;
		TM_Cal_Value_C = ((float)TM_Cal_Value_C * 1.8) + 32.0;
	}
	TM_Cal_float_Value_F = (float)TM_Cal_Value_F/10.0;
	TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
	
	eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
	eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_OFF,TM_Upper_Alm_OFF);
	eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_ON,TM_Lower_Alm_ON);
	eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_OFF,TM_Lower_Alm_OFF);
}
//****************************************************************************************************************************************/
void delay(unsigned int cnt)
{
	while(cnt--)
	{
		//wdt_reset();
	}
}
//****************************************************************************************************************************************/
void Check_RTC(void)
{
	b.Sec_blink_flag ^= 1;

	ep.currentEpochTime++;
	get_date_time(&rtc,ep.currentEpochTime);

	current_min = rtc.minute;
	current_hr = rtc.hour;
	
	//---------------------------------------------------------------
	if(last_min != current_min)
	{
		last_min = current_min;

		FillRamBuffer(NORMAL_LOG,0,0xFFFF);
	}
	
	//---------------------------------------------------------------	
	if(rtc.hour>=12) 
	{
		b.AM_PM_Flag=0;
	}
	else                    
	{
		b.AM_PM_Flag=1;
	}
	//---------------------------------------------------------------
		
	#ifdef DEBUG_RTC

		opstr(0,"\r\nEpoch:");
		print_Hex(0,ep.cept[3]);
		print_Hex(0,ep.cept[2]);
		print_Hex(0,ep.cept[1]);
		print_Hex(0,ep.cept[0]);
	
		opstr(0,"  RTC:");
		opchar(0,(rtc.day/10) + 0x30);
		opchar(0,(rtc.day%10) + 0x30);
		opchar(0,'/');
		opchar(0,(rtc.month/10) + 0x30);
		opchar(0,(rtc.month%10) + 0x30);
		opchar(0,'/');
		opchar(0,(rtc.year/10) + 0x30);
		opchar(0,(rtc.year%10) + 0x30);
		opchar(0,' ');
		opchar(0,(rtc.hour/10) + 0x30);
		opchar(0,(rtc.hour%10) + 0x30);
		opchar(0,':');
		opchar(0,(rtc.minute/10) + 0x30);
		opchar(0,(rtc.minute%10) + 0x30);
		opchar(0,':');
		opchar(0,(rtc.second/10) + 0x30);
		opchar(0,(rtc.second%10) + 0x30);
	
		opstr(0,"   ADC Reading:");
		print_float(0,ADC_sample,&test[0],0);
	
		opstr(0,"   Battery Voltage:");
		print_float(0,BatteryPercentage,&test[0],0);
		opstr(0,"\r\n");
	
	#endif
}

//Read SHT25
void Read_SHT25(void)
{
	float tempvar=0;
	unsigned char error=0;
	error = 0;                                       // reset error status
	
    // --- Reset sensor by command ---
    //error |= SHT2x_SoftReset();

    // --- Read the sensors serial number (64bit) ---
    //error |= SHT2x_GetSerialNumber(SerialNumber_SHT2x);

    // --- Set Resolution e.g. RH 10bit, Temp 13bit ---
    //error |= SHT2x_ReadUserRegister(&userRegister);  //get actual user reg
    //userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
    //error |= SHT2x_WriteUserRegister(&userRegister); //write changed user reg

	if(gu16_parameterWord & ENABLE_RH)
	{	
		// --- measure humidity with "Hold Master Mode (HM)"  ---
		//error |= SHT2x_MeasureHM(HUMIDITY, &sRH);
		error |= SHT2x_MeasurePoll(HUMIDITY, &sRH);
	}
	else
	{	
		sRH.u16=0;
	}
	
	if(gu16_parameterWord & ENABLE_TEMP)
	{	
		// --- measure temperature with "Polling Mode" (no hold master) ---
		error |= SHT2x_MeasurePoll(TEMP, &sT);
	}
	else
	{	
		sT.u16=0;
	}
		
	if((sT.u16==0xFFFF) || (sRH.u16==0xFFFF))
	{
		b.RH_TEMP_NC = 1;
		temperatureC = 0;
		temperatureF = 0;
		humidityRH   = 0;
		
		TM_Alrm_ON=NO_ALARM;
		RH_Alrm_ON=NO_ALARM;
		
		//TMRH_StartUpTimer=S_STABLE_TIME;
	}
	else
	{
		b.RH_TEMP_NC=0;
		
		//-- calculate humidity and temperature --
		if(gu16_parameterWord & ENABLE_TEMP)
		{	
			sT.u16>>=2;
			//sT.u16 += TM_Cal_Count;
			//sT.u16 += TM_Cal_Count_C;
			
			//if(TM_Cal_Count < 0) sT.u16 += TM_Cal_Count;
			//else 				     sT.u16 -= TM_Cal_Count;
			
			sT.u16<<=2;
			
			temperatureC = SHT2x_CalcTemperatureC(sT.u16);
			RealtemperatureC = temperatureC;
			temperatureC -= TM_Cal_float_Value_F;
			temperatureC -= TM_Cal_float_Value_C;
			
			temperatureC = Kalman_Update(&Kalman[2], temperatureC);
			
			if(TM_Unit) 
			{
				temperatureF = (temperatureC * 1.8) + 32.0;
				RealtemperatureF = temperatureF;
			}
			
			if(!TMRH_StartUpTimer)
			{
				if(!TM_Unit)
				{
					tempvar=temperatureC;
				}
				else
				{
					tempvar=temperatureF;
				}
				
				//Find Temperature Min/Max --------------------------------------------------------
				if(temperatureC > TM_Max)
				{
					TM_Max = temperatureC;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Max,(uint8_t*)TEMP_MAXIMUM,4);
				}
				
				if(temperatureC < TM_Min)
				{
					TM_Min = temperatureC;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Min,(uint8_t*)TEMP_MINIMUM,4);
				}
								
				//Check Alarm Limit for Temp -----------------------------------------------
				if(tempvar > (float)TM_Upper_Alm_ON/10.0)
				{
					TM_Alrm_ON=UPPER_ALARM;
					
					if(!b.TMLog)
					{
						LastTM_Alrm_ON=TM_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
						
						FillRamBuffer(TM_ALM_OCCURE_LOG,0,0xFFFF);
						
						b.autoSendResponse = true;
						b.TMLog=1;
					}
					else
					{
						if(LastTM_Alrm_ON!=TM_Alrm_ON)
						{
							FillRamBuffer(TM_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastTM_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
							
							b.TMLog=0;
						}
					}
				}
				else if(tempvar < (float)TM_Lower_Alm_ON/10.0)
				{
					TM_Alrm_ON=LOWER_ALARM;
					
					if(!b.TMLog)
					{
						LastTM_Alrm_ON=TM_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
						
						FillRamBuffer(TM_ALM_OCCURE_LOG,0,0xFFFF);
						
						b.autoSendResponse = true;
						b.TMLog=1;
					}
					else
					{
						if(LastTM_Alrm_ON!=TM_Alrm_ON)
						{
							FillRamBuffer(TM_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastTM_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
							
							b.TMLog=0;
						}
					}
				}
				else if((tempvar < ((float)TM_Upper_Alm_OFF/10.0)) && (tempvar > ((float)TM_Lower_Alm_OFF/10.0)))
				{
					TM_Alrm_ON=NO_ALARM;
					
					if(b.TMLog==1)
					{
						FillRamBuffer(TM_ALM_RESTORE_LOG,0,0xFFFF);
						
						b.TMLog=0;
						
						LastTM_Alrm_ON=TM_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
					}
				}
			}
		}
		else
		{
			temperatureC=0.0;
			temperatureF=0.0;
			
			TM_Max=0.0;
			TM_Min=0.0;
			
			TM_Alrm_ON=NO_ALARM;
		}
		
		if(gu16_parameterWord & ENABLE_RH)
		{	
			sRH.u16>>=2;
			//sRH.u16 += RH_Cal_Count;
			//sRH.u16 += RH_Cal_Count_C;
			
			sRH.u16<<=2;
			
			humidityRH = SHT2x_CalcRH(sRH.u16);
			RealhumidityRH = humidityRH;
			humidityRH -= RH_Cal_float_Value_F;
			humidityRH -= RH_Cal_float_Value_C;
			
			humidityRH = Kalman_Update(&Kalman[3], humidityRH);
			
			if(!TMRH_StartUpTimer)
			{
				//Find RH Min/Max -----------------------------------------------------------------
				if(humidityRH > RH_Max) 
				{
					RH_Max = humidityRH;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Max,(uint8_t*)RH_MAXIMUM,4);
				}
				
				if(humidityRH < RH_Min) 
				{
					RH_Min = humidityRH;
					eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Min,(uint8_t*)RH_MINIMUM,4);
				}
				
				//Check Alarm Limit for RH -----------------------------------------------
				if(humidityRH > (float)RH_Upper_Alm_ON/10.0)
				{
					RH_Alrm_ON=UPPER_ALARM;
					
					if(!b.RHLog)
					{
						LastRH_Alrm_ON=RH_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
						
						FillRamBuffer(RH_ALM_OCCURE_LOG,0,0xFFFF);
						
						b.autoSendResponse = true;
						b.RHLog=1;
					}
					else
					{
						if(LastRH_Alrm_ON!=RH_Alrm_ON)
						{
							FillRamBuffer(RH_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastRH_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
							
							b.RHLog=0;
						}
					}
				}
				else if(humidityRH < (float)RH_Lower_Alm_ON/10.0)
				{
					RH_Alrm_ON=LOWER_ALARM;
					
					if(!b.RHLog)
					{
						LastRH_Alrm_ON=RH_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
						
						FillRamBuffer(RH_ALM_OCCURE_LOG,0,0xFFFF);
						
						b.autoSendResponse = true;
						b.RHLog=1;
					}
					else
					{
						if(LastRH_Alrm_ON!=RH_Alrm_ON)
						{
							FillRamBuffer(RH_ALM_RESTORE_LOG,0,0xFFFF);
							
							LastRH_Alrm_ON=NO_ALARM;
							eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
							
							b.RHLog=0;
						}
					}
				}
				else if((humidityRH < (float)RH_Upper_Alm_OFF/10.0) && (humidityRH > (float)RH_Lower_Alm_OFF/10.0))
				{
					RH_Alrm_ON=NO_ALARM;
					
					if(b.RHLog==1)
					{
						FillRamBuffer(RH_ALM_RESTORE_LOG,0,0xFFFF);
						
						b.RHLog=0;
						
						LastRH_Alrm_ON=RH_Alrm_ON;
						eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
					}
				}
			}
		}
		else
		{
			humidityRH=0.0;
			
			RH_Max=0.0;
			RH_Min=0.0;
			
			RH_Alrm_ON=NO_ALARM;
		}
	}
	
    // --- check end of battery status (eob)---
    // note: a RH / Temp. measurement must be executed to update the status of eob
    //error |= SHT2x_ReadUserRegister(&userRegister);  //get actual user reg
    //if( (userRegister & SHT2x_EOB_MASK) == SHT2x_EOB_ON ) endOfBattery = 1;
    //else endOfBattery = 0;

    //-- write humidity and temperature values on LCD --
    //sprintf(humitityOutStr,    "Humidity RH:%6.2f %% ",humidityRH);
    //sprintf(temperatureOutStr, "Temperature:%6.2f?C",temperatureC);
    //DisplayWriteString(2,0,humitityOutStr);
    //DisplayWriteString(3,0,temperatureOutStr);

    //-- write error or low batt status un LCD --
    //if(error != 0)
    //{ DisplayWriteString(1,3,"Error occurred");
    //  DisplayWriteString(2,0,"Humidity RH: --.-- %%");
    //  DisplayWriteString(3,0,"Temperature: --.--?C");
    //}
    //else if(endOfBattery) DisplayWriteString(1,3,"Low Batt");
    //else DisplayWriteString(1,0,"                    ");

    //_delay_ms(300);     // wait 0.3s for next measurement
	
	#ifdef DEBUG_SHT25
		opstr(0,"Temp: ");	print_float(0,sT.u16,test,0);			opstr(0," ");
							print_float(0,temperatureC,test,2);		opstr(0," ");
		opstr(0,"RH: ");	print_float(0,sRH.u16,test,0);			opstr(0," ");
							print_float(0,humidityRH,test,2);		opstr(0," ");
		opstr(0,"Err: ");	print_float(0,error,test,0);			opstr(0," ");
	#endif
}

//*****************************************************************************************************************************************/
void boot_data(void)
{
	FirstTimeCheck = eeprom_read_byte ((uint8_t*)FIRST_BOOT_CHECK);
	if(FirstTimeCheck != 0xAA)
	{
		FirstTimeCheck=0xAA;
		eeprom_busy_wait();  eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)FIRST_BOOT_CHECK,FirstTimeCheck); 
		
		gu16_parameterWord=PARAMETER_WORD;
		eeprom_busy_wait();  eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DISP_PARA_SELECT,gu16_parameterWord);
		
		//gu8_DPAutoCalFlag=0;
		//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP_AUTO_CAL_FLAG,gu8_DPAutoCalFlag);
		
		gu8_masterEnable=0;
		eeprom_busy_wait();  eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)MASTER_ENABLE_ADDR,gu8_masterEnable);
		
		memset(gu8ar_SrNumber,'0',sizeof(gu8ar_SrNumber));
		eeprom_busy_wait();  eeprom_write_block(&gu8ar_SrNumber[0],(void*)DEVICE_SR_NO,sizeof(gu8ar_SrNumber));
		gu32_SrNumber = ascii2hex(&gu8ar_SrNumber[8],8);
		
		gu8_DP1_LEDBlinkForPara=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_LED_SETTING_ADDR,gu8_DP1_LEDBlinkForPara);
		
		gu8_DP2_LEDBlinkForPara=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_LED_SETTING_ADDR,gu8_DP2_LEDBlinkForPara);
		
		gu8_TM_LEDBlinkForPara=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_LED_SETTING_ADDR,gu8_TM_LEDBlinkForPara);
		
		gu8_RH_LEDBlinkForPara=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RH_LED_SETTING_ADDR,gu8_RH_LEDBlinkForPara);
		
		gu8_BackLitOnOff=1;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BACKLIT_ON_OFF_ADDR,gu8_BackLitOnOff);
		
		gu8_TM_RH_ScanTime=5;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_RH_SCAN_TIME_ADDR,gu8_TM_RH_ScanTime);
		
		RTCCorruptDataInd=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)CORRUPT_RTC_IND_ADDR,RTCCorruptDataInd);
		
		DP1_UserCalDateInd=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_USER_CAL_DATE_IND_ADDR,DP1_UserCalDateInd);
		
		DP2_UserCalDateInd=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_USER_CAL_DATE_IND_ADDR,DP2_UserCalDateInd);
		
		TM_UserCalDateInd=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_USER_CAL_DATE_IND_ADDR,TM_UserCalDateInd);
		
		RH_UserCalDateInd=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RH_USER_CAL_DATE_IND_ADDR,RH_UserCalDateInd);
		
		//RTCSetFlag=0;
		//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG_ADDR,RTCSetFlag);
		//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG1_ADDR,RTCSetFlag);
		//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG2_ADDR,RTCSetFlag);
		
		ParaScrollTime=DEFAULT_PARA_SCROLL_TIME;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)PARA_SCROLL_TIME,ParaScrollTime);
		
		if(gu16_parameterWord & ENABLE_DP1)
		{
			//DPressure1 Parameter -----------------------------------------------------
			DP1_Upper_Alm_ON=DEFAULT_DP1_UPPER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_UP_ALM_ON,DP1_Upper_Alm_ON);
		
			DP1_Upper_Alm_OFF=DEFAULT_DP1_UPPER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_UP_ALM_OFF,DP1_Upper_Alm_OFF);
		
			DP1_Lower_Alm_ON=DEFAULT_DP1_LOWER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_LO_ALM_ON,DP1_Lower_Alm_ON);
		
			DP1_Lower_Alm_OFF=DEFAULT_DP1_LOWER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_LO_ALM_OFF,DP1_Lower_Alm_OFF);
		
			DP1_Cal_Value_F=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_F_ADDR,DP1_Cal_Value_F);
			
			DP1_Cal_Value_C=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
			
			DP1_Cal_float_Value_F = 0.0;
			DP1_Cal_float_Value_C = 0.0;			
				
			//DP1_Cal_Count=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT,DP1_Cal_Count);
		//
			//DP1_Cal_Count_C=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT_C,DP1_Cal_Count_C);
			
			LastDP1_Alrm_ON=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
		}

		if(gu16_parameterWord & ENABLE_DP2)
		{
			//DPressure2 Parameter -----------------------------------------------------
			DP2_Upper_Alm_ON=DEFAULT_DP2_UPPER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_ON,DP2_Upper_Alm_ON);
		
			DP2_Upper_Alm_OFF=DEFAULT_DP2_UPPER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_OFF,DP2_Upper_Alm_OFF);
		
			DP2_Lower_Alm_ON=DEFAULT_DP2_LOWER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_ON,DP2_Lower_Alm_ON);
		
			DP2_Lower_Alm_OFF=DEFAULT_DP2_LOWER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_OFF,DP2_Lower_Alm_OFF);
		
			DP2_Cal_Value_F=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_F_ADDR,DP2_Cal_Value_F);
			
			DP2_Cal_Value_C=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
			
			DP2_Cal_float_Value_F = 0.0;
			DP2_Cal_float_Value_C = 0.0;
			
			//DP2_Cal_Count=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT,DP2_Cal_Count);
		//
			//DP2_Cal_Count_C=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT_C,DP2_Cal_Count_C);
			
			LastDP2_Alrm_ON=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
		}
		
		//gu8_dp_sw_enb = 0;
		//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP_FACT_ENB_ADDR,gu8_dp_sw_enb);
		
		gu8_broadcast = 0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BROADCAST_ENB_ADDR,gu8_broadcast);
		
		gu8_rly_stat = 0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RELAY_STAT_ADDR,gu8_rly_stat);
		
		for(uint8_t i=0; i<MAX_SUPPORTED_DP; i++)
		{
			su16_dp_sw_factor[i]=0;
			f32_dp_sw_factor[i]=0.0;
			eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_SW_FACT_ADDR+(i*2)),su16_dp_sw_factor[i]);
			
			u16_dp_limit[i]=2500;
			f32_dp_limit[i]=250.0;
			eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_LIMIT_ADDR+(i*2)),u16_dp_limit[i]);
		}

		if(gu16_parameterWord & ENABLE_TEMP)
		{
			//Temperature Parameter -----------------------------------------------------
			TM_Upper_Alm_ON=DEFAULT_TM_C_UPPER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
		
			TM_Upper_Alm_OFF=DEFAULT_TM_C_UPPER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_OFF,TM_Upper_Alm_OFF);
		
			TM_Lower_Alm_ON=DEFAULT_TM_C_LOWER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_ON,TM_Lower_Alm_ON);
		
			TM_Lower_Alm_OFF=DEFAULT_TM_C_LOWER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_OFF,TM_Lower_Alm_OFF);
		
			TM_Cal_Value_F=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
			
			TM_Cal_Value_C=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
			
			TM_Cal_float_Value_F = 0.0;
			TM_Cal_float_Value_C = 0.0;
			
			//TM_Cal_Count=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT,TM_Cal_Count);
		//
			//TM_Cal_Count_C=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT_C,TM_Cal_Count_C);
		//
			TM_Unit=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TEMP_UNIT,TM_Unit);
			
			LastTM_Alrm_ON=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
		}
			
		if(gu16_parameterWord & ENABLE_RH)
		{
			//RHumidity Parameter -----------------------------------------------------
			RH_Upper_Alm_ON=DEFAULT_RH_UPPER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_ON,RH_Upper_Alm_ON);
		
			RH_Upper_Alm_OFF=DEFAULT_RH_UPPER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_OFF,RH_Upper_Alm_OFF);
		
			RH_Lower_Alm_ON=DEFAULT_RH_LOWER_ALM_ON;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_ON,RH_Lower_Alm_ON);
		
			RH_Lower_Alm_OFF=DEFAULT_RH_LOWER_ALM_OFF;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_OFF,RH_Lower_Alm_OFF);
		
			RH_Cal_Value_F=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_F_ADDR,RH_Cal_Value_F);
			
			RH_Cal_Value_C=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
			
			RH_Cal_float_Value_F = 0.0;
			RH_Cal_float_Value_C = 0.0;
			
			//RH_Cal_Count=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT,RH_Cal_Count);
		
			//RH_Cal_Count_C=0;
			//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT_C,RH_Cal_Count_C);
			
			LastRH_Alrm_ON=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
		}
		
		//RS485 Parameter -------------------------------------------
		DeviceID=DEFAULT_DEVICE_ID;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICE_ID,DeviceID);
		
		//Buzzer Parameter -------------------------------------------
		Buzzer_ON_Time=DEFAULT_BUZZER_ON_TIME;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_ON_TIME,Buzzer_ON_Time);
		
		Buzzer_OFF_Time=DEFAULT_BUZZER_OFF_TIME;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_OFF_TIME,Buzzer_OFF_Time);
		
		//UART Parameter -------------------------------------------
		UART_BaudRate=DEFAULT_UART_BAUDRATE;	//57600
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_BAUDRATE,UART_BaudRate);
		
		UART_DataBits=DEFAULT_UART_DATABITS;	//8
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_DATBITS,UART_DataBits);
		
		UART_Parity=DEFAULT_UART_PARITYBITS;		//None
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_PARITY,UART_Parity);
		
		UART_StopBit=DEFAULT_UART_STOPBITS;	//1
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_STOPBIT,UART_StopBit);
		
		//Customer Password -------------------------------------------
		CustPassword=DEFAULT_CUSTOMER_PWD;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)CUSTOMER_PASSWORD,CustPassword);
		
		//Factory Customer Password -------------------------------------------
		FactCustPassword=DEFAULT_FACTORY_PWD;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)FAC_CUSTOMER_PASSWORD,FactCustPassword);
		
		//Acknowledge Parameter -------------------------------------------
		AckTimer=1;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)ACK_TIMER,AckTimer);
		
		AckPwdInd=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)ACK_PWD_IND,AckPwdInd);
		
		for(uint8_t i=0;i<NO_OF_ACKPWD;i++)
		{
			AckPwd[i]=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)(ACK_PASSWORD+(i*2)),AckPwd[i]);
		}
		
	
		//Reset Data Logging Parameter -------------------------------------------
		CurrentLogIndReadLoc = 0;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)CURR_LOG_IND_RDLC,CurrentLogIndReadLoc);
		
		FlashOVFByte=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)FLSH_OVF_IND,FlashOVFByte);
		
		CurrentLogInd = 0;
		eeprom_busy_wait();  eeprom_write_block((uint8_t*)&CurrentLogInd,(uint8_t*)CURR_LOG_IND,4);
		
		CurrentLog24IndReadLoc = 0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)CURR_LOG24_IND_RDLC,CurrentLog24IndReadLoc);
		
		CurrentLog24Ind = 0;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)CURR_LOG24_IND,CurrentLog24Ind);
		
		b.DP1Log=0;
		b.DP2Log=0;
		b.TMLog=0;
		b.RHLog=0;
		
		LastDP1_Alrm_ON=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
		
		LastDP2_Alrm_ON=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
		
		LastTM_Alrm_ON=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
		
		LastRH_Alrm_ON=0;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
		
		gu8_doorSensingPolarity=1;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DOOR_SENSE_POLARITY_ADDR,gu8_doorSensingPolarity);
		
		gu8_doorSensingTime=60;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DOOR_SENSE_TIME_ADDR,gu8_doorSensingTime);
		
		gu8_Dp1AlarmSensingTime=5;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_ALM_SENSE_TIME_ADDR,gu8_Dp1AlarmSensingTime);
		
		gu8_Dp2AlarmSensingTime=5;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_ALM_SENSE_TIME_ADDR,gu8_Dp2AlarmSensingTime);
		
		gu8_LCDBrigthnessCnt=DEFAULT_LCD_BRIGHTNESS;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LCD_BRIGHT_CNT_ADDR,gu8_LCDBrigthnessCnt);
		
		gu8_AutoSentInterval=DEFAULT_AUTO_SENT_INTERVAL;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)AUTO_SENT_INTERVAL_ADDR,gu8_AutoSentInterval);
		
		gu8_DeviceInGroup=DEFAULT_DEVICES_IN_GROUP;
		eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICES_IN_GROUP_ADDR,gu8_DeviceInGroup);
		
		gu16_XbeeRstInterval=DEFAULT_XBEE_RST_INTERVAL;
		eeprom_busy_wait();  eeprom_write_word ((unsigned int*)XBEE_RST_INTERVAL_ADDR,gu16_XbeeRstInterval);
		
		memcpy(&gu8arr_XbeeMac[0][0],"000000000000FFFF",XBEE_MAC_SIZE);
		eeprom_busy_wait();  eeprom_write_block(&gu8arr_XbeeMac[0][0],(uint8_t*)(XBEE_MAC_ADDR),XBEE_MAC_SIZE);
		for(uint8_t i=1;i<NO_OF_XBEE_MAC;i++)
		{
			memset(&gu8arr_XbeeMac[i][0],'0',XBEE_MAC_SIZE);
			eeprom_busy_wait();  eeprom_write_block(&gu8arr_XbeeMac[i][0],(uint8_t*)(XBEE_MAC_ADDR+(i*XBEE_MAC_SIZE)),XBEE_MAC_SIZE);
		}
	}
	else
	{	
		gu8_doorSensingPolarity  = eeprom_read_byte ((uint8_t*)DOOR_SENSE_POLARITY_ADDR);
		if(gu8_doorSensingPolarity > 1)
		{
			gu8_doorSensingPolarity=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DOOR_SENSE_POLARITY_ADDR,gu8_doorSensingPolarity);
		}
		
		gu8_LCDBrigthnessCnt  = eeprom_read_byte ((uint8_t*)LCD_BRIGHT_CNT_ADDR);
		if(gu8_LCDBrigthnessCnt > 15)
		{
			gu8_LCDBrigthnessCnt=DEFAULT_LCD_BRIGHTNESS;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LCD_BRIGHT_CNT_ADDR,gu8_LCDBrigthnessCnt);
		}
		
		gu8_AutoSentInterval  = eeprom_read_byte ((uint8_t*)AUTO_SENT_INTERVAL_ADDR);
		if((!gu8_AutoSentInterval) || (gu8_AutoSentInterval > 240))
		{
			gu8_AutoSentInterval=DEFAULT_AUTO_SENT_INTERVAL;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)AUTO_SENT_INTERVAL_ADDR,gu8_AutoSentInterval);
		}
		
		gu8_DeviceInGroup  = eeprom_read_byte ((uint8_t*)DEVICES_IN_GROUP_ADDR);
		if((gu8_DeviceInGroup < 2) || (gu8_DeviceInGroup > 100))
		{
			gu8_DeviceInGroup=DEFAULT_DEVICES_IN_GROUP;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICES_IN_GROUP_ADDR,gu8_DeviceInGroup);
		}
		
		gu16_XbeeRstInterval  = eeprom_read_word ((unsigned int*)XBEE_RST_INTERVAL_ADDR);
		if(gu16_XbeeRstInterval > 1440)
		{
			gu16_XbeeRstInterval=DEFAULT_XBEE_RST_INTERVAL;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)XBEE_RST_INTERVAL_ADDR,gu16_XbeeRstInterval);
		}
		
		eeprom_read_block(&gu8ar_SrNumber[0],(void*)DEVICE_SR_NO,sizeof(gu8ar_SrNumber));
		gu32_SrNumber = ascii2hex(&gu8ar_SrNumber[8],8);
		
		gu8_doorSensingTime  = eeprom_read_byte ((uint8_t*)DOOR_SENSE_TIME_ADDR);
		if(gu8_doorSensingTime > 250)
		{
			gu8_doorSensingTime=60;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DOOR_SENSE_TIME_ADDR,gu8_doorSensingTime);
		}
		
		gu8_Dp1AlarmSensingTime  = eeprom_read_byte ((uint8_t*)DP1_ALM_SENSE_TIME_ADDR);
		if(gu8_Dp1AlarmSensingTime > 250)
		{
			gu8_Dp1AlarmSensingTime=5;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_ALM_SENSE_TIME_ADDR,gu8_Dp1AlarmSensingTime);
		}
		
		gu8_Dp2AlarmSensingTime  = eeprom_read_byte ((uint8_t*)DP2_ALM_SENSE_TIME_ADDR);
		if(gu8_Dp2AlarmSensingTime > 250)
		{
			gu8_Dp2AlarmSensingTime=5;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_ALM_SENSE_TIME_ADDR,gu8_Dp2AlarmSensingTime);
		}
			
		gu8_masterEnable  = eeprom_read_byte ((uint8_t*)MASTER_ENABLE_ADDR);
		if(gu8_masterEnable > 1)
		{
			gu8_masterEnable=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)MASTER_ENABLE_ADDR,gu8_masterEnable);
		}
		
		gu8_DP1_LEDBlinkForPara  = eeprom_read_byte ((uint8_t*)DP1_LED_SETTING_ADDR);
		if(gu8_DP1_LEDBlinkForPara > 0x0F)
		{
			gu8_DP1_LEDBlinkForPara=0b00000100;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_LED_SETTING_ADDR,gu8_DP1_LEDBlinkForPara);
		}
		
		gu8_DP2_LEDBlinkForPara  = eeprom_read_byte ((uint8_t*)DP2_LED_SETTING_ADDR);
		if(gu8_DP2_LEDBlinkForPara > 0x0F)
		{
			gu8_DP2_LEDBlinkForPara=0b00001000;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_LED_SETTING_ADDR,gu8_DP2_LEDBlinkForPara);
		}
		
		gu8_TM_LEDBlinkForPara  = eeprom_read_byte ((uint8_t*)TM_LED_SETTING_ADDR);
		if(gu8_TM_LEDBlinkForPara > 0x0F)
		{
			gu8_TM_LEDBlinkForPara=0b00000001;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_LED_SETTING_ADDR,gu8_TM_LEDBlinkForPara);
		}
		
		gu8_RH_LEDBlinkForPara  = eeprom_read_byte ((uint8_t*)RH_LED_SETTING_ADDR);
		if(gu8_RH_LEDBlinkForPara > 0x0F)
		{
			gu8_RH_LEDBlinkForPara=0b00000010;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RH_LED_SETTING_ADDR,gu8_RH_LEDBlinkForPara);
		}
		
		gu8_BackLitOnOff  = eeprom_read_byte ((uint8_t*)BACKLIT_ON_OFF_ADDR);
		if(gu8_BackLitOnOff>1)
		{
			gu8_BackLitOnOff=1;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BACKLIT_ON_OFF_ADDR,gu8_BackLitOnOff);
		}
		
		gu8_TM_RH_ScanTime  = eeprom_read_byte ((uint8_t*)TM_RH_SCAN_TIME_ADDR);
		if((gu8_TM_RH_ScanTime<5) || (gu8_TM_RH_ScanTime>20))
		{
			gu8_TM_RH_ScanTime=5;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_RH_SCAN_TIME_ADDR,gu8_TM_RH_ScanTime);
		}
		
		RTCCorruptDataInd  = eeprom_read_byte ((uint8_t*)CORRUPT_RTC_IND_ADDR);
		if(RTCCorruptDataInd>15)
		{
			RTCCorruptDataInd=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)CORRUPT_RTC_IND_ADDR,RTCCorruptDataInd);
		}
		
		/*eeprom_read_byte ((uint8_t*)DP_AUTO_CAL_FLAG);
		if(gu8_DPAutoCalFlag>1)
		{
			gu8_DPAutoCalFlag=1;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP_AUTO_CAL_FLAG,gu8_DPAutoCalFlag);
		}*/
		
		gu16_parameterWord = eeprom_read_word ((unsigned int*)DISP_PARA_SELECT);
		//gu16_parameterWord = PARAMETER_WORD;

		DP1_UserCalDateInd  = eeprom_read_byte ((uint8_t*)DP1_USER_CAL_DATE_IND_ADDR);
		if(DP1_UserCalDateInd>15)
		{
			DP1_UserCalDateInd=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP1_USER_CAL_DATE_IND_ADDR,DP1_UserCalDateInd);
		}
		
		DP2_UserCalDateInd  = eeprom_read_byte ((uint8_t*)DP2_USER_CAL_DATE_IND_ADDR);
		if(DP2_UserCalDateInd>15)
		{
			DP2_UserCalDateInd=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DP2_USER_CAL_DATE_IND_ADDR,DP2_UserCalDateInd);
		}
		
		TM_UserCalDateInd  = eeprom_read_byte ((uint8_t*)TM_USER_CAL_DATE_IND_ADDR);
		if(TM_UserCalDateInd>15)
		{
			TM_UserCalDateInd=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TM_USER_CAL_DATE_IND_ADDR,TM_UserCalDateInd);
		}
		
		RH_UserCalDateInd  = eeprom_read_byte ((uint8_t*)RH_USER_CAL_DATE_IND_ADDR);
		if(RH_UserCalDateInd>15)
		{
			RH_UserCalDateInd=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RH_USER_CAL_DATE_IND_ADDR,RH_UserCalDateInd);
		}
		
		//Buffer1[0]  = eeprom_read_byte ((uint8_t*)RTC_SET_FLAG_ADDR);
		//Buffer1[1]  = eeprom_read_byte ((uint8_t*)RTC_SET_FLAG1_ADDR);
		//Buffer1[2]  = eeprom_read_byte ((uint8_t*)RTC_SET_FLAG2_ADDR);
		//
		//if(Buffer1[0]>1) Buffer1[0]=0;
		//if(Buffer1[1]>1) Buffer1[1]=0;
		//if(Buffer1[2]>1) Buffer1[2]=0;
		//
		//if((Buffer1[0]==1) || (Buffer1[1]==1) || (Buffer1[2]==1))
		//{
			//RTCSetFlag=1;
			//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG_ADDR,RTCSetFlag);
			//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG1_ADDR,RTCSetFlag);
			//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG2_ADDR,RTCSetFlag);
		//}
		//else
		//{
			//RTCSetFlag=0;
		//}
		
		//RTCSetFlag  = eeprom_read_byte ((uint8_t*)RTC_SET_FLAG_ADDR);
		//if(RTCSetFlag>1)
		//{
			//RTCSetFlag=0;
			//eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)RTC_SET_FLAG_ADDR,RTCSetFlag);
		//}
		
		ParaScrollTime  = eeprom_read_byte ((uint8_t*)PARA_SCROLL_TIME);
		if(ParaScrollTime>60)
		{
			ParaScrollTime=DEFAULT_PARA_SCROLL_TIME;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)PARA_SCROLL_TIME,ParaScrollTime);
		}

		if(gu16_parameterWord & ENABLE_DP1)
		{
			//DPressure1 Parameter -----------------------------------------------------
			DP1_Upper_Alm_ON  = eeprom_read_word ((unsigned int*)DP1_UP_ALM_ON);
			if((DP1_Upper_Alm_ON<(DEFAUT_DP1_MAX*10)) || (DP1_Upper_Alm_ON>(DEFAUT_DP1_MIN*10)))
			{
				DP1_Upper_Alm_ON=DEFAULT_DP1_UPPER_ALM_ON;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_UP_ALM_ON,DP1_Upper_Alm_ON);
			}
		
			DP1_Upper_Alm_OFF  = eeprom_read_word ((unsigned int*)DP1_UP_ALM_OFF);
			if((DP1_Upper_Alm_OFF<(DEFAUT_DP1_MAX*10)) || (DP1_Upper_Alm_OFF>(DEFAUT_DP1_MIN*10)))
			{
				DP1_Upper_Alm_OFF=DEFAULT_DP1_UPPER_ALM_OFF;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_UP_ALM_OFF,DP1_Upper_Alm_OFF);
			}
		
			DP1_Lower_Alm_ON  = eeprom_read_word ((unsigned int*)DP1_LO_ALM_ON);
			if((DP1_Lower_Alm_ON<(DEFAUT_DP1_MAX*10)) || (DP1_Lower_Alm_ON>(DEFAUT_DP1_MIN*10)))
			{
				DP1_Lower_Alm_ON=DEFAULT_DP1_LOWER_ALM_ON;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_LO_ALM_ON,DP1_Lower_Alm_ON);
			}
		
			DP1_Lower_Alm_OFF  = eeprom_read_word ((unsigned int*)DP1_LO_ALM_OFF);
			if((DP1_Lower_Alm_OFF<(DEFAUT_DP1_MAX*10)) || (DP1_Lower_Alm_OFF>(DEFAUT_DP1_MIN*10)))
			{
				DP1_Lower_Alm_OFF=DEFAULT_DP1_LOWER_ALM_OFF;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_LO_ALM_OFF,DP1_Lower_Alm_OFF);
			}
		
			DP1_Cal_Value_F  = eeprom_read_word ((unsigned int*)DP1_CAL_VAL_F_ADDR);
			//if((DP1_Cal_Value_F<(DEFAUT_DP1_MAX*10.0)) || (DP1_Cal_Value_F>(DEFAUT_DP1_MIN*10.0)))
			//{
				//DP1_Cal_Value_F=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_F_ADDR,DP1_Cal_Value_F);
			//}
			DP1_Cal_float_Value_F = (float)DP1_Cal_Value_F/10.0;
			
			DP1_Cal_Value_C  = eeprom_read_word ((unsigned int*)DP1_CAL_VAL_C_ADDR);
			//if((DP1_Cal_Value_C<(DEFAUT_DP1_MAX*10.0)) || (DP1_Cal_Value_C>(DEFAUT_DP1_MIN*10.0)))
			//{
				//DP1_Cal_Value_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_VAL_C_ADDR,DP1_Cal_Value_C);
			//}
			DP1_Cal_float_Value_C = (float)DP1_Cal_Value_C/10.0;
			
			//DP1_Cal_Count  = eeprom_read_word ((unsigned int*)DP1_CAL_CNT);
			//if((DP1_Cal_Count<-500) || (DP1_Cal_Count>500))
			//{
				//DP1_Cal_Count=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT,DP1_Cal_Count);
			//}
		//
			//DP1_Cal_Count_C  = eeprom_read_word ((unsigned int*)DP1_CAL_CNT_C);
			//if((DP1_Cal_Count_C<-500) || (DP1_Cal_Count_C>500))
			//{
				//DP1_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP1_CAL_CNT_C,DP1_Cal_Count_C);
			//}
		
			eeprom_read_block((uint8_t*)&DP1_Max,(uint8_t*)DP1_MAXIMUM,4);
			if(DP1_Max<DEFAUT_DP1_MAX)
			{
				DP1_Max = DEFAUT_DP1_MAX;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Max,(uint8_t*)DP1_MAXIMUM,4);
			}
		
			eeprom_read_block((uint8_t*)&DP1_Min,(uint8_t*)DP1_MINIMUM,4);
			if(DP1_Min>DEFAUT_DP1_MIN)
			{
				DP1_Min = DEFAUT_DP1_MIN;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP1_Min,(uint8_t*)DP1_MINIMUM,4);
			}
			
			LastDP1_Alrm_ON = eeprom_read_byte ((uint8_t*)LAST_DP1_ALRM_STAT);
			if(LastDP1_Alrm_ON>2)
			{
				LastDP1_Alrm_ON=0;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP1_ALRM_STAT,LastDP1_Alrm_ON);
			}
		}
		
		if(gu16_parameterWord & ENABLE_DP2)
		{
			//DPressure2 Parameter -----------------------------------------------------
			DP2_Upper_Alm_ON  = eeprom_read_word ((unsigned int*)DP2_UP_ALM_ON);
			if((DP2_Upper_Alm_ON<(DEFAUT_DP2_MAX*10)) || (DP2_Upper_Alm_ON>(DEFAUT_DP2_MIN*10)))
			{
				DP2_Upper_Alm_ON=DEFAULT_DP2_UPPER_ALM_ON;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_ON,DP2_Upper_Alm_ON);
			}
		
			DP2_Upper_Alm_OFF  = eeprom_read_word ((unsigned int*)DP2_UP_ALM_OFF);
			if((DP2_Upper_Alm_OFF<(DEFAUT_DP2_MAX*10)) || (DP2_Upper_Alm_OFF>(DEFAUT_DP2_MIN*10)))
			{
				DP2_Upper_Alm_OFF=DEFAULT_DP2_UPPER_ALM_OFF;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_UP_ALM_OFF,DP2_Upper_Alm_OFF);
			}
		
			DP2_Lower_Alm_ON  = eeprom_read_word ((unsigned int*)DP2_LO_ALM_ON);
			if((DP2_Lower_Alm_ON<(DEFAUT_DP2_MAX*10)) || (DP2_Lower_Alm_ON>(DEFAUT_DP2_MIN*10)))
			{
				DP2_Lower_Alm_ON=DEFAULT_DP2_LOWER_ALM_ON;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_ON,DP2_Lower_Alm_ON);
			}
		
			DP2_Lower_Alm_OFF  = eeprom_read_word ((unsigned int*)DP2_LO_ALM_OFF);
			if((DP2_Lower_Alm_OFF<(DEFAUT_DP2_MAX*10)) || (DP2_Lower_Alm_OFF>(DEFAUT_DP2_MIN*10)))
			{
				DP2_Lower_Alm_OFF=DEFAULT_DP2_LOWER_ALM_OFF;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_LO_ALM_OFF,DP2_Lower_Alm_OFF);
			}
			
			DP2_Cal_Value_F  = eeprom_read_word ((unsigned int*)DP2_CAL_VAL_F_ADDR);
			//if((DP2_Cal_Value_F<(DEFAUT_DP2_MAX*10.0)) || (DP2_Cal_Value_F>(DEFAUT_DP2_MIN*10.0)))
			//{
				//DP2_Cal_Value_F=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_F_ADDR,DP2_Cal_Value_F);
			//}
			DP2_Cal_float_Value_F = (float)DP2_Cal_Value_F/10.0;
			
			DP2_Cal_Value_C  = eeprom_read_word ((unsigned int*)DP2_CAL_VAL_C_ADDR);
			//if((DP2_Cal_Value_C<(DEFAUT_DP2_MAX*10.0)) || (DP2_Cal_Value_C>(DEFAUT_DP2_MIN*10.0)))
			//{
				//DP2_Cal_Value_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_VAL_C_ADDR,DP2_Cal_Value_C);
			//}
			DP2_Cal_float_Value_C = (float)DP2_Cal_Value_C/10.0;
			
			//DP2_Cal_Count  = eeprom_read_word ((unsigned int*)DP2_CAL_CNT);
			//if((DP2_Cal_Count<-500) || (DP2_Cal_Count>500))
			//{
				//DP2_Cal_Count=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT,DP2_Cal_Count);
			//}
		//
			//DP2_Cal_Count_C  = eeprom_read_word ((unsigned int*)DP2_CAL_CNT_C);
			//if((DP2_Cal_Count_C<-500) || (DP2_Cal_Count_C>500))
			//{
				//DP2_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)DP2_CAL_CNT_C,DP2_Cal_Count_C);
			//}
		
			eeprom_read_block((uint8_t*)&DP2_Max,(uint8_t*)DP2_MAXIMUM,4);
			if(DP2_Max<DEFAUT_DP2_MAX)
			{
				DP2_Max = DEFAUT_DP2_MAX;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Max,(uint8_t*)DP2_MAXIMUM,4);
			}
		
			eeprom_read_block((uint8_t*)&DP2_Min,(uint8_t*)DP2_MINIMUM,4);
			if(DP2_Min>DEFAUT_DP2_MIN)
			{
				DP2_Min = DEFAUT_DP2_MIN;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&DP2_Min,(uint8_t*)DP2_MINIMUM,4);
			}
			
			LastDP2_Alrm_ON = eeprom_read_byte ((uint8_t*)LAST_DP2_ALRM_STAT);
			if(LastDP2_Alrm_ON>2)
			{
				LastDP2_Alrm_ON=0;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_DP2_ALRM_STAT,LastDP2_Alrm_ON);
			}
		}
		
		gu8_rly_stat = eeprom_read_byte ((uint8_t*)RELAY_STAT_ADDR);

		/*gu8_dp_sw_enb = eeprom_read_byte ((uint8_t*)DP_FACT_ENB_ADDR);
		if(gu8_dp_sw_enb>1)
		{	
			gu8_dp_sw_enb=0;
			eeprom_busy_wait();  eeprom_write_byte((uint8_t*)DP_FACT_ENB_ADDR,gu8_dp_sw_enb);
		}*/
		
		gu8_broadcast = eeprom_read_byte ((uint8_t*)BROADCAST_ENB_ADDR);
		if(gu8_broadcast>1)
		{	
			gu8_broadcast=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)BROADCAST_ENB_ADDR,gu8_broadcast);
		}
		
		
		for(uint8_t i=0; i<MAX_SUPPORTED_DP; i++)
		{
			su16_dp_sw_factor[i]  = eeprom_read_word ((unsigned int*)(DP_SW_FACT_ADDR+(i*2)));
			if((su16_dp_sw_factor[i]<-10000) && (su16_dp_sw_factor[i]>10000))
			{	
				su16_dp_sw_factor[i]=0;
				eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_SW_FACT_ADDR+(i*2)),su16_dp_sw_factor[i]);
			}
			f32_dp_sw_factor[i]=(float)su16_dp_sw_factor[i]/100.0;
			
			u16_dp_limit[i]  = eeprom_read_word ((unsigned int*)(DP_LIMIT_ADDR+(i*2)));
			if((u16_dp_limit[i]<500) || (u16_dp_limit[i]>9990))
			{
				u16_dp_limit[i]=2500;
				eeprom_busy_wait();  eeprom_write_word((unsigned int*)(DP_LIMIT_ADDR+(i*2)),u16_dp_limit[i]);
			}
			f32_dp_limit[i]=(float)u16_dp_limit[i]/10.0;
		}
		
		if(gu16_parameterWord & ENABLE_TEMP)
		{
			//Temperature Parameter -----------------------------------------------------
			TM_Unit  = eeprom_read_byte ((uint8_t*)TEMP_UNIT);
			if(TM_Unit>1)
			{
				TM_Unit=0;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)TEMP_UNIT,TM_Unit);
			}
			
			TM_Cal_Value_F  = eeprom_read_word ((unsigned int*)TM_CAL_VAL_F_ADDR);
			//if((TM_Cal_Value_F<(DEFAUT_TEMP_C_MAX*10.0)) || (TM_Cal_Value_F>(DEFAUT_TEMP_C_MIN*10.0)))
			//{
				//TM_Cal_Value_F=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_F_ADDR,TM_Cal_Value_F);
			//}
			
			TM_Cal_Value_C  = eeprom_read_word ((unsigned int*)TM_CAL_VAL_C_ADDR);
			//if((TM_Cal_Value_C<(DEFAUT_TEMP_C_MAX*10.0)) || (TM_Cal_Value_C>(DEFAUT_TEMP_C_MIN*10.0)))
			//{
				//TM_Cal_Value_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TM_CAL_VAL_C_ADDR,TM_Cal_Value_C);
			//}
		
			if(!TM_Unit)
			{
				TM_Upper_Alm_ON  = eeprom_read_word ((unsigned int*)TEMP_UP_ALM_ON);
				if((TM_Upper_Alm_ON<(DEFAUT_TEMP_C_MAX*10)) || (TM_Upper_Alm_ON>(DEFAUT_TEMP_C_MIN*10)))
				{
					TM_Upper_Alm_ON=DEFAULT_TM_C_UPPER_ALM_ON;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
				}
			
				TM_Upper_Alm_OFF  = eeprom_read_word ((unsigned int*)TEMP_UP_ALM_OFF);
				if((TM_Upper_Alm_OFF<(DEFAUT_TEMP_C_MAX*10)) || (TM_Upper_Alm_OFF>(DEFAUT_TEMP_C_MIN*10)))
				{
					TM_Upper_Alm_OFF=DEFAULT_TM_C_UPPER_ALM_OFF;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_OFF,TM_Upper_Alm_OFF);
				}
			
				TM_Lower_Alm_ON  = eeprom_read_word ((unsigned int*)TEMP_LO_ALM_ON);
				if((TM_Lower_Alm_ON<(DEFAUT_TEMP_C_MAX*10)) || (TM_Lower_Alm_ON>(DEFAUT_TEMP_C_MIN*10)))
				{
					TM_Lower_Alm_ON=DEFAULT_TM_C_LOWER_ALM_ON;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_ON,TM_Lower_Alm_ON);
				}
			
				TM_Lower_Alm_OFF  = eeprom_read_word ((unsigned int*)TEMP_LO_ALM_OFF);
				if((TM_Lower_Alm_OFF<(DEFAUT_TEMP_C_MAX*10)) || (TM_Lower_Alm_OFF>(DEFAUT_TEMP_C_MIN*10)))
				{
					TM_Lower_Alm_OFF=DEFAULT_TM_C_LOWER_ALM_OFF;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_OFF,TM_Lower_Alm_OFF);
				}
			}
			else
			{
				TM_Upper_Alm_ON  = eeprom_read_word ((unsigned int*)TEMP_UP_ALM_ON);
				if((TM_Upper_Alm_ON<(DEFAUT_TEMP_F_MAX*10)) || (TM_Upper_Alm_ON>(DEFAUT_TEMP_F_MIN*10)))
				{
					TM_Upper_Alm_ON=DEFAULT_TM_F_UPPER_ALM_ON;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_ON,TM_Upper_Alm_ON);
				}
			
				TM_Upper_Alm_OFF  = eeprom_read_word ((unsigned int*)TEMP_UP_ALM_OFF);
				if((TM_Upper_Alm_OFF<(DEFAUT_TEMP_F_MAX*10)) || (TM_Upper_Alm_OFF>(DEFAUT_TEMP_F_MIN*10)))
				{
					TM_Upper_Alm_OFF=DEFAULT_TM_F_UPPER_ALM_OFF;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_UP_ALM_OFF,TM_Upper_Alm_OFF);
				}
			
				TM_Lower_Alm_ON  = eeprom_read_word ((unsigned int*)TEMP_LO_ALM_ON);
				if((TM_Lower_Alm_ON<(DEFAUT_TEMP_F_MAX*10)) || (TM_Lower_Alm_ON>(DEFAUT_TEMP_F_MIN*10)))
				{
					TM_Lower_Alm_ON=DEFAULT_TM_F_LOWER_ALM_ON;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_ON,TM_Lower_Alm_ON);
				}
			
				TM_Lower_Alm_OFF  = eeprom_read_word ((unsigned int*)TEMP_LO_ALM_OFF);
				if((TM_Lower_Alm_OFF<(DEFAUT_TEMP_F_MAX*10)) || (TM_Lower_Alm_OFF>(DEFAUT_TEMP_F_MIN*10)))
				{
					TM_Lower_Alm_OFF=DEFAULT_TM_F_LOWER_ALM_OFF;
					eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_LO_ALM_OFF,TM_Lower_Alm_OFF);
				}
				
				TM_Cal_Value_F = ((float)TM_Cal_Value_F * 1.8) + 32.0;
				TM_Cal_Value_C = ((float)TM_Cal_Value_C * 1.8) + 32.0;
			}
		
			TM_Cal_float_Value_F = (float)TM_Cal_Value_F/10.0;
			TM_Cal_float_Value_C = (float)TM_Cal_Value_C/10.0;
				
			//TM_Cal_Count  = eeprom_read_word ((unsigned int*)TEMP_CAL_CNT);
			//if((TM_Cal_Count<-1000) || (TM_Cal_Count>1000))
			//{
				//TM_Cal_Count=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT,TM_Cal_Count);
			//}
		//
			//TM_Cal_Count_C  = eeprom_read_word ((unsigned int*)TEMP_CAL_CNT_C);
			//if((TM_Cal_Count_C<-1000) || (TM_Cal_Count_C>1000))
			//{
				//TM_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)TEMP_CAL_CNT_C,TM_Cal_Count_C);
			//}

			eeprom_read_block((uint8_t*)&TM_Max,(uint8_t*)TEMP_MAXIMUM,4);
			eeprom_read_block((uint8_t*)&TM_Min,(uint8_t*)TEMP_MINIMUM,4);
			if(TM_Min>DEFAUT_TEMP_C_MIN) TM_Min = DEFAUT_TEMP_C_MIN;
			if(TM_Max<DEFAUT_TEMP_C_MAX) TM_Max = DEFAUT_TEMP_C_MAX;
			eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Min,(uint8_t*)TEMP_MINIMUM,4);
			eeprom_busy_wait();  eeprom_write_block((uint8_t*)&TM_Max,(uint8_t*)TEMP_MAXIMUM,4);
			
			LastTM_Alrm_ON = eeprom_read_byte ((uint8_t*)LAST_TM_ALRM_STAT);
			if(LastTM_Alrm_ON>2)
			{
				LastTM_Alrm_ON=0;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_TM_ALRM_STAT,LastTM_Alrm_ON);
			}
		}
		
		if(gu16_parameterWord & ENABLE_RH)
		{
			//RHumidity Parameter -----------------------------------------------------
			RH_Upper_Alm_ON  = eeprom_read_word ((unsigned int*)RH_UP_ALM_ON);
			if(RH_Upper_Alm_ON>(DEFAUT_RH_MIN*10))
			{
				RH_Upper_Alm_ON=DEFAULT_RH_UPPER_ALM_ON;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_ON,RH_Upper_Alm_ON);
			}
		
			RH_Upper_Alm_OFF  = eeprom_read_word ((unsigned int*)RH_UP_ALM_OFF);
			if(RH_Upper_Alm_OFF>(DEFAUT_RH_MIN*10))
			{
				RH_Upper_Alm_OFF=DEFAULT_RH_UPPER_ALM_OFF;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_UP_ALM_OFF,RH_Upper_Alm_OFF);
			}
		
			RH_Lower_Alm_ON  = eeprom_read_word ((unsigned int*)RH_LO_ALM_ON);
			if(RH_Lower_Alm_ON>(DEFAUT_RH_MIN*10))
			{
				RH_Lower_Alm_ON=DEFAULT_RH_LOWER_ALM_ON;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_ON,RH_Lower_Alm_ON);
			}
		
			RH_Lower_Alm_OFF  = eeprom_read_word ((unsigned int*)RH_LO_ALM_OFF);
			if(RH_Lower_Alm_OFF>(DEFAUT_RH_MIN*10))
			{
				RH_Lower_Alm_OFF=DEFAULT_RH_LOWER_ALM_OFF;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_LO_ALM_OFF,RH_Lower_Alm_OFF);
			}
		
			RH_Cal_Value_F  = eeprom_read_word ((unsigned int*)RH_CAL_VAL_F_ADDR);
			//if((RH_Cal_Value_F<(-DEFAUT_RH_MIN*10.0)) || (RH_Cal_Value_F>(DEFAUT_RH_MIN*10.0)))
			//{
				//RH_Cal_Value_F=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_F_ADDR,RH_Cal_Value_F);
			//}
			RH_Cal_float_Value_F = (float)RH_Cal_Value_F/10.0;
			
			RH_Cal_Value_C  = eeprom_read_word ((unsigned int*)RH_CAL_VAL_C_ADDR);
			//if((RH_Cal_Value_C<(DEFAUT_RH_MAX*10.0)) || (RH_Cal_Value_C>(DEFAUT_RH_MIN*10.0)))
			//{
				//RH_Cal_Value_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_VAL_C_ADDR,RH_Cal_Value_C);
			//}
			RH_Cal_float_Value_C = (float)RH_Cal_Value_C/10.0;
			
			//RH_Cal_Count  = eeprom_read_word ((unsigned int*)RH_CAL_CNT);
			//if((RH_Cal_Count<-1500) || (RH_Cal_Count>1500))
			//{
				//RH_Cal_Count=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT,RH_Cal_Count);
			//}
		//
			//RH_Cal_Count_C  = eeprom_read_word ((unsigned int*)RH_CAL_CNT_C);
			//if((RH_Cal_Count_C<-1500) || (RH_Cal_Count_C>1500))
			//{
				//RH_Cal_Count_C=0;
				//eeprom_busy_wait();  eeprom_write_word ((unsigned int*)RH_CAL_CNT_C,RH_Cal_Count_C);
			//}
		
			eeprom_read_block((uint8_t*)&RH_Max,(uint8_t*)RH_MAXIMUM,4);
			if(RH_Max<DEFAUT_RH_MAX)
			{
				RH_Max = DEFAUT_RH_MAX;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Max,(uint8_t*)RH_MAXIMUM,4);
			}
		
			eeprom_read_block((uint8_t*)&RH_Min,(uint8_t*)RH_MINIMUM,4);
			if(RH_Min>DEFAUT_RH_MIN)
			{
				RH_Min = DEFAUT_RH_MIN;
				eeprom_busy_wait();  eeprom_write_block((uint8_t*)&RH_Min,(uint8_t*)RH_MINIMUM,4);
			}
			
			LastRH_Alrm_ON = eeprom_read_byte ((uint8_t*)LAST_RH_ALRM_STAT);
			if(LastRH_Alrm_ON>2)
			{
				LastRH_Alrm_ON=0;
				eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)LAST_RH_ALRM_STAT,LastRH_Alrm_ON);
			}
		}
		
		//RS485 Parameter -------------------------------------------
		DeviceID  = eeprom_read_byte ((uint8_t*)DEVICE_ID);
		if(DeviceID>250)
		{
			DeviceID=DEFAULT_DEVICE_ID;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)DEVICE_ID,DeviceID);
		}
		
		//Buzzer Parameter -------------------------------------------
		Buzzer_ON_Time  = eeprom_read_word ((unsigned int*)BUZZER_ON_TIME);
		if(Buzzer_ON_Time>60)
		{
			Buzzer_ON_Time=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_ON_TIME,Buzzer_ON_Time);
		}
		
		Buzzer_OFF_Time  = eeprom_read_word ((unsigned int*)BUZZER_OFF_TIME);
		if(Buzzer_OFF_Time>960)
		{
			Buzzer_OFF_Time=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)BUZZER_OFF_TIME,Buzzer_OFF_Time);
		}
		
		//UART Parameter -------------------------------------------
		UART_BaudRate = eeprom_read_byte ((uint8_t*)UART_BAUDRATE);
		if((UART_BaudRate<3) || (UART_BaudRate>9))
		{
			UART_BaudRate=DEFAULT_UART_BAUDRATE;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_BAUDRATE,UART_BaudRate);
		}
		
		UART_DataBits = eeprom_read_byte ((uint8_t*)UART_DATBITS);
		if(UART_DataBits>3)
		{
			UART_DataBits=DEFAULT_UART_DATABITS;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_DATBITS,UART_DataBits);
		}
		
		UART_Parity = eeprom_read_byte ((uint8_t*)UART_PARITY);
		if(UART_Parity>2)
		{
			UART_Parity=DEFAULT_UART_PARITYBITS;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_PARITY,UART_Parity);
		}
		
		UART_StopBit = eeprom_read_byte ((uint8_t*)UART_STOPBIT);
		if(UART_StopBit>1)
		{
			UART_StopBit=DEFAULT_UART_STOPBITS;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)UART_STOPBIT,UART_StopBit);
		}

		//Customer Password -------------------------------------------
		CustPassword = eeprom_read_word ((unsigned int*)CUSTOMER_PASSWORD);
		if(CustPassword>999)
		{
			CustPassword=DEFAULT_CUSTOMER_PWD;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)CUSTOMER_PASSWORD,CustPassword);
		}
		
		//Factory Customer Password -------------------------------------------
		FactCustPassword = eeprom_read_word ((unsigned int*)FAC_CUSTOMER_PASSWORD);
		if(FactCustPassword>9999)
		{
			FactCustPassword=DEFAULT_FACTORY_PWD;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)FAC_CUSTOMER_PASSWORD,FactCustPassword);
		}
		
		//Acknowledge Parameter -------------------------------------------
		AckTimer = eeprom_read_word ((unsigned int*)ACK_TIMER);
		if(AckTimer>1440)
		{
			AckTimer=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)ACK_TIMER,AckTimer);
		}
		
		AckPwdInd  = eeprom_read_byte ((uint8_t*)ACK_PWD_IND);
		if(AckPwdInd>NO_OF_ACKPWD)
		{
			AckPwdInd=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)ACK_PWD_IND,AckPwdInd);
		}
		
		for(uint8_t i=0;i<NO_OF_ACKPWD;i++)
		{
			AckPwd[i] = eeprom_read_word ((unsigned int*)(ACK_PASSWORD+(i*2)));
			if(AckPwd[i]>999)
			{
				AckPwd[i]=0;
				eeprom_busy_wait();  eeprom_write_word ((unsigned int*)(ACK_PASSWORD+(i*2)),AckPwd[i]);
			}
		}
		
		for(uint8_t i=0;i<NO_OF_XBEE_MAC;i++)
		{
			eeprom_read_block(&gu8arr_XbeeMac[i][0],(uint8_t*)(XBEE_MAC_ADDR+(i*XBEE_MAC_SIZE)),XBEE_MAC_SIZE);
			
			if(gu8arr_XbeeMac[i][0]==0xFF)
			{
				memset(&gu8arr_XbeeMac[i][0],'0',XBEE_MAC_SIZE);
				eeprom_busy_wait();  eeprom_write_block(&gu8arr_XbeeMac[i][0],(uint8_t*)(XBEE_MAC_ADDR+(i*XBEE_MAC_SIZE)),XBEE_MAC_SIZE);
			}
		}
		
		//Data Logging Parameter -------------------------------------------	
		
		CurrentLogIndReadLoc  = eeprom_read_word ((unsigned int*)CURR_LOG_IND_RDLC);
		if(CurrentLogIndReadLoc>=100)
		{
			CurrentLogIndReadLoc=0;
			eeprom_busy_wait();  eeprom_write_word ((unsigned int*)CURR_LOG_IND_RDLC,CurrentLogIndReadLoc);
		}
		
		FlashOVFByte  = eeprom_read_byte ((uint8_t*)FLSH_OVF_IND);
		if(FlashOVFByte>1)
		{
			FlashOVFByte=0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)FLSH_OVF_IND,FlashOVFByte);
		}
		
		CurrentLog24IndReadLoc  = eeprom_read_byte ((uint8_t*)CURR_LOG24_IND_RDLC);
		if(CurrentLog24IndReadLoc>=100)
		{
			CurrentLog24IndReadLoc = 0;
			eeprom_busy_wait();  eeprom_write_byte ((uint8_t*)CURR_LOG24_IND_RDLC,CurrentLog24IndReadLoc);
		}
	}
	
	/*if(!gu8_DPAutoCalFlag)
	{
		gu8_DPAutoCalTimer=60;
	}
	else
	{
		gu8_DPAutoCalTimer=0;
	}*/
}

void USART_SetBaudrate(USART_t * usart, int16_t bsel,int8_t bscale)
{
	usart->BAUDCTRLA = (bsel & 0xff) << USART_BSEL_gp;
	usart->BAUDCTRLB = ((bscale) << USART_BSCALE_gp) | ((bsel >> 8) << USART_BSEL_gp);
}

void InitUSART(USART_t * usart, PORT_t * port, uint8_t rxpin, uint8_t txpin, uint8_t ubrr, uint8_t noOfbits, uint8_t parity, uint8_t stopbit)
{
	int16_t brt_bsel=0;
	int8_t brt_bscale=0;
	
	//-----------------------------
	// USART setup ( RXD0, TXD0 )

	/* (TXD0) as output. */
	port->DIRSET = txpin;
	
	/* (RXD0) as input. */
	port->DIRCLR = rxpin;
	
	//disable while setting baud rate
	usart->CTRLA=0;
	usart->CTRLC=0;
	
	//for Fosc=32 MHz
	switch(ubrr)
	{
		case BAUD_1200:		brt_bscale = -1; 		brt_bsel = 3331;	break;
		case BAUD_2400:		brt_bscale = -2; 		brt_bsel = 3329;	break;
		case BAUD_4800:		brt_bscale = -3; 		brt_bsel = 3325;	break;
		case BAUD_9600:		brt_bscale = -4;		brt_bsel = 3317;	break;
		case BAUD_14400:		brt_bscale = -4; 		brt_bsel = 2206;	break;
		case BAUD_19200:		brt_bscale = -5; 		brt_bsel = 3301;	break;
		case BAUD_28800:		brt_bscale = -5; 		brt_bsel = 2190;	break;
		case BAUD_38400:		brt_bscale = -6; 		brt_bsel = 3269;	break;
		case BAUD_57600:		brt_bscale = -6;		brt_bsel = 2158; 	break;
	}
	
	USART_SetBaudrate(usart, brt_bsel, brt_bscale);
	
	switch(noOfbits)
	{
		case DATABIT_5:																	break;
		case DATABIT_6:		usart->CTRLC |= USART_CHSIZE0_bm;							break;
		case DATABIT_7:		usart->CTRLC |= USART_CHSIZE1_bm;							break;
		case DATABIT_8:		usart->CTRLC |= (USART_CHSIZE1_bm | USART_CHSIZE0_bm);		break;
		default:    																	break;	// 8 bit
	}

	switch(parity)
	{
		case PARITY_EVEN:	usart->CTRLC |= USART_PMODE1_bm;							break;
		case PARITY_ODD:	usart->CTRLC |= (USART_PMODE1_bm | USART_PMODE0_bm);		break;
		case PARITY_NONE:																break;
	}
	
	switch(stopbit)
	{
		case STOP_BIT_2:	usart->CTRLC |= USART_SBMODE_bm;		break;
		case STOP_BIT_1:											break;
		default:												break;	//1 bit
	}

	usart->CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	//USARTC0_CTRLA = (USART_RXCINTLVL_HI_gc | USART_TXCINTLVL_HI_gc | USART_DREINTLVL_HI_gc);
	usart->CTRLA = USART_RXCINTLVL_HI_gc;
}

void opstr(uint8_t portNo,char *str)
{
	if(!portNo)
	{
		RS485_TX0_ENB;
	
		_delay_us(365);
	
		while(*str != '\0')
		{
			while(!(USARTC0_STATUS & USART_DREIF_bm));
			USARTC0_DATA=*str;
		
			while(!(USARTC0_STATUS & USART_TXCIF_bm));
			USARTC0_STATUS |= USART_DREIF_bm;
		
			str++;
		}
	
		RS485_TX0_DIS;
	}
	else
	{
		RS485_TX1_ENB;
		
		_delay_us(365);
		
		while(*str != '\0')
		{
			while(!(USARTC1_STATUS & USART_DREIF_bm));
			USARTC1_DATA=*str;
			
			while(!(USARTC1_STATUS & USART_TXCIF_bm));
			USARTC1_STATUS |= USART_DREIF_bm;
			
			str++;
		}

		RS485_TX1_DIS;
	}
}

void SendToUART(uint8_t port,uint8_t *str,unsigned short NoOfBytes)
{
	if(!port)
	{
		RS485_TX0_ENB;
		
		_delay_us(365);
			
		while(NoOfBytes)
		{
			while(!(USARTC0_STATUS & USART_DREIF_bm));
			USARTC0_DATA=*str;
		
			while(!(USARTC0_STATUS & USART_TXCIF_bm));
			USARTC0_STATUS |= USART_DREIF_bm;
		
			str++;
			NoOfBytes--;
		}
	
		RS485_TX0_DIS;
	}
	else if(port==1)
	{
		RS485_TX1_ENB;
		
		_delay_us(365);
	
		while(NoOfBytes)
		{
			while(!(USARTC1_STATUS & USART_DREIF_bm));
			USARTC1_DATA=*str;
		
			while(!(USARTC1_STATUS & USART_TXCIF_bm));
			USARTC1_STATUS |= USART_DREIF_bm;
		
			str++;
			NoOfBytes--;
		}
	
		RS485_TX1_DIS;
	}
	else
	{
		//udi_cdc_multi_write_buf(0,str,NoOfBytes);
		
		_delay_ms(50);
	}
}

void opchar(uint8_t portNo,uint8_t str)
{
	if(!portNo)
	{
		RS485_TX0_ENB;
	
		_delay_us(365);
	
		while(!(USARTC0_STATUS & USART_DREIF_bm));
		USARTC0_DATA=str;
	
		while(!(USARTC0_STATUS & USART_TXCIF_bm));
		USARTC0_STATUS |= USART_DREIF_bm;

		RS485_TX0_DIS;
	}
	else
	{
		RS485_TX1_ENB;
		
		_delay_us(365);
		
		while(!(USARTC1_STATUS & USART_DREIF_bm));
		USARTC1_DATA=str;
		
		while(!(USARTC1_STATUS & USART_TXCIF_bm));
		USARTC1_STATUS |= USART_DREIF_bm;
		
		RS485_TX1_DIS;
	}
}

void print_Hex(uint8_t portNo,uint8_t str)
{
	uint8_t cnt=0;
	
	cnt=str >> 4;
	
	if(cnt > 9)
	{
		cnt -= 10;
		opchar(portNo,cnt + 'A');
	}
	else
	{
		opchar(portNo,cnt + '0');
	}
	
	cnt=str & 0x0F;
	
	if(cnt > 9)
	{
		cnt -= 10;
		opchar(portNo,cnt + 'A');
	}
	else
	{
		opchar(portNo,cnt + '0');
	}
}

void print_float(uint8_t portNo,float val,uint8_t *data1,uint8_t bytes_after_dp)
{
	uint8_t x;
	unsigned long temppow,lu32_templong;
	float tempdoub;
	
	x=1;
	temppow=10;
	tempdoub=val;
	
	//----------------------------------------------------------
	if(tempdoub<0.0) 
	{
		tempdoub*=(-1.0);
		*data1='-';		//means '-' sign
		data1++;
	}
	//----------------------------------------------------------
	
	lu32_templong=tempdoub;
	tempdoub-=lu32_templong;
	
	while(lu32_templong>=temppow) //to findout digit before decimal point
	{
		temppow*=10;
		x++;
	}
	data1+=x;

	for(k=x;k>0;k--)  // to convert value before dp into BCD
	{
		data1--;
		*data1=(lu32_templong%10)+0x30;  
		lu32_templong/=10;
	}

	if(bytes_after_dp)  // to convert value after dp into BCD
	{
		data1+=x;
		*data1 = '.';
		data1++;
		for(k=0;k<bytes_after_dp;k++)
		{
			tempdoub*=10;
			x=tempdoub;
			tempdoub-=x;
			*data1=x+0x30;
			data1++;
		}
	}	
	else
	{
		data1+=x;
		//data1++;
	}
	
	*data1=0;
	
	opstr(portNo,(char*)&test[0]);
}

void print_short(uint8_t portNo,long val,uint8_t *data1,uint8_t no_of_digit)
{	
	if(val<0)
	{
		val *= (-1);
		*data1 = '-';
		data1++;
	}
	
	data1+=(no_of_digit-1);
	
	data1++;
	*data1=0;
	data1--;
	
	while(no_of_digit)
	{
		*data1=(val%10)+0x30;
		val/=10;
		data1--;
		no_of_digit--;
	}
	
	opstr(portNo,(char*)&test[0]);
}

//**************************************************************************************************************************************/

//------------------------------------------------------------------------------
// Epoch Time Functions
//------------------------------------------------------------------------------
/* Return 1 if YEAR + TM_YEAR_BASE is a leap year.  */
static inline int leapyear (long int year)
{
  /* Don't add YEAR to TM_YEAR_BASE, as that might overflow.
     Also, work even if YEAR is negative.  */
    return((year & 3) == 0 && (year % 100 != 0 || ((year / 100) & 3) == (- (TM_YEAR_BASE / 100) & 3)));
}


unsigned long  ydhms_diff (long int year1, long int yday1, int hour1, int min1, int sec1,int year0, int yday0, int hour0, int min0, int sec0)
{
    /* Compute intervening leap days correctly even if year is negative.
     Take care to avoid integer overflow here.  */
    int a4 = SHR (year1, 2) + SHR (TM_YEAR_BASE, 2) - ! (year1 & 3);
    int b4 = SHR (year0, 2) + SHR (TM_YEAR_BASE, 2) - ! (year0 & 3);
    int a100 = a4 / 25 - (a4 % 25 < 0);
    int b100 = b4 / 25 - (b4 % 25 < 0);
    int a400 = SHR (a100, 2);
    int b400 = SHR (b100, 2);
    int intervening_leap_days = (a4 - b4) - (a100 - b100) + (a400 - b400);
    
    /* Compute the desired time in time_t precision.  Overflow might
     occur here.  */
    unsigned long int tyear1 = year1;
    unsigned long int years = tyear1 - year0;
    unsigned long int days = 365 * years + yday1 - yday0 + intervening_leap_days;
    unsigned long int hours = 24 * days + hour1 - hour0;
    unsigned long int minutes = 60 * hours + min1 - min0;
    unsigned long int seconds = 60 * minutes + sec1 - sec0;
    return seconds;
}

unsigned long get_epoch_time(struct RTCData t1)
{
    t1.year -= TM_YEAR_BASE;
    t1.month--;

    int mon_remainder1 = t1.month % 12;
    int negative_mon_remainder1 = mon_remainder1 < 0;
    int mon_years1 = t1.month / 12 - negative_mon_remainder1;
    long int lyear_requested1 = t1.year;
    long int year1 = lyear_requested1 + mon_years1;
    
    int mon_yday1 = ((__mon_yday[leapyear (year1)][mon_remainder1 + 12 * negative_mon_remainder1]) - 1);
    long int lmday1 = t1.day;
    long int yday1 = mon_yday1 + lmday1;
    
    return(ydhms_diff (year1, yday1, t1.hour, t1.minute, t1.second,EPOCH_YEAR - TM_YEAR_BASE, 0, 0, 0, 0));
}

void get_date_time(struct RTCData* t1,unsigned long epoch)
{
	unsigned long dayclock, dayno;
	int year = EPOCH_YEAR;
	
	dayclock = epoch % 86400;
	dayno = epoch / 86400;
	
	t1->second = dayclock % 60;
	t1->minute = (dayclock % 3600) / 60;
	t1->hour = dayclock / 3600;
	t1->day = (dayno + 4) % 7; // Day 0 was a sunday
	while (dayno >= (unsigned long) YEARSIZE(year))
	{
		dayno -= YEARSIZE(year);
		year++;
	}
	t1->year = year - TM_YEAR_BASE;
	//date_time->tm_yday = dayno;
	t1->month = 0;
	while (dayno >= (unsigned long) _ytab[LEAPYEAR(year)][t1->month])
	{
		dayno -= _ytab[LEAPYEAR(year)][t1->month];
		t1->month++;
	}
	t1->month++;
	t1->day = dayno + 1;
	
	//t1->year += 2000;
}
//------------------------------------------------------------------------------
// UART Interrupt Vector
//------------------------------------------------------------------------------

ISR(USARTC0_RXC_vect)
{
	uint8_t var=0;
	
	var = USARTC0_DATA;
	
	if(!b.msgRcvOK)
	{
		if((var==0xFF) && (!rxMode))
		{
			RxBuffer1[RxInd++]=var;
			rxMode=1;
			RxTimeout=4;
		}
		else if((var==0xEA) && (!rxMode))
		{
			RxBuffer1[RxInd++]=var;
			rxMode=2;
			RxTimeout=4;	
		}
		else if(rxMode==1)
		{
			if((var==DeviceID) || (var==0x00))
			{
				RxBuffer1[RxInd++]=var;
				rxMode=3;
				RxTimeout=4;
			}
			else
			{
				rxMode=0;
				RxTimeout=0;
				RxInd=0;
			}
		}
		else if(rxMode==2)
		{
			RxBuffer1[RxInd++]=var;
			
			if(RxInd==9)
			{
				if(!memcmp(&RxBuffer1[1],&gu8ar_SrNumber[8],8))
				{
					rxMode=3;
					RxInd=1;
					RxTimeout=4;
				}
				else
				{
					rxMode=0;
					RxTimeout=0;
					RxInd=0;
				}
			}
		}
		else if(rxMode==3)
		{
			RxBuffer1[RxInd++]=var;
			RxTimeout=4;

			if((var==0xFE) || (var==0xEB))
			{
				crcVal=CalCRC(&RxBuffer1[1],RxInd-3);
				
				#ifdef DEBUG_RCV_CMD
				opstr(0,"\r\nCRC:");
				print_Hex(0,crcVal);
				#endif
				
				rxMode=0;
				RxTimeout=0;
				if(RxBuffer1[RxInd-2]==crcVal)
				{
					if(!b.FlashReadCmd && !b.Flash24ReadCmd && !b.MinMaxMeanLogReadCmd && !b.MeanHrLogReadCmd && !b.RamReadCmd && !b.RamAllReadCmd)
					{
						for(uint8_t m=0;m<RxInd;m++) RxBuffer[m]=RxBuffer1[m];
						RxBuffer[1]=DeviceID;
						b.msgRcvOK=1;
						comport=0;
					}
					else
					{
						RxInd=0;
					}
				}
				else
				{
					RxInd=0;
				}
			}
		}
		if(RxInd>=RX_IND_MAX) RxInd=0;
	}
}

ISR(USARTC1_RXC_vect)
{
	uint8_t var=0;
	
	var = USARTC1_DATA;
	
	XbeeRxBuffer[XbeeRxInd++]=var;
	if(XbeeRxInd>=XBEE_RX_IND_MAX) XbeeRxInd=0;
	
	if(!b.msgRcvOK)
	{
		if((var==0xFF) && (!rxMode))
		{
			RxBuffer1[RxInd++]=var;
			rxMode=1;
			RxTimeout=4;	
		}
		else if((var==0xEA) && (!rxMode))
		{
			RxBuffer1[RxInd++]=var;
			rxMode=2;
			RxTimeout=4;	
		}
		else if(rxMode==1)
		{
			if((var==DeviceID) || (var==0x00))
			{
				RxBuffer1[RxInd++]=var;
				rxMode=3;
				RxTimeout=4;
			}
			else
			{
				rxMode=0;
				RxTimeout=0;
				RxInd=0;
			}
		}
		else if(rxMode==2)
		{
			RxBuffer1[RxInd++]=var;
			
			if(RxInd==9)
			{
				if(!memcmp(&RxBuffer1[1],&gu8ar_SrNumber[8],8))
				{
					rxMode=3;
					RxInd=1;
					RxTimeout=4;
				}
				else
				{
					rxMode=0;
					RxTimeout=0;
					RxInd=0;
				}
			}
		}
		else if(rxMode==3)
		{
			RxBuffer1[RxInd++]=var;
			RxTimeout=4;

			if((var==0xFE) || (var==0xEB))
			{
				crcVal=CalCRC(&RxBuffer1[1],RxInd-3);
				
				#ifdef DEBUG_RCV_CMD
				opstr(1,"\r\nCRC:");
				print_Hex(1,crcVal);
				#endif
				
				rxMode=0;
				RxTimeout=0;
				if(RxBuffer1[RxInd-2]==crcVal) 
				{
					if(!b.FlashReadCmd && !b.Flash24ReadCmd && !b.MinMaxMeanLogReadCmd && !b.MeanHrLogReadCmd && !b.RamReadCmd && !b.RamAllReadCmd)
					{
						for(uint8_t m=0;m<RxInd;m++) RxBuffer[m]=RxBuffer1[m];
						RxBuffer[1]=DeviceID;
						b.msgRcvOK=1;
						comport=1;
					}
					else
					{
						RxInd=0;
					}
				}
				else
				{
					RxInd=0;			
				}
			}
		}
		if(RxInd>=RX_IND_MAX) RxInd=0;
	}
}

/*
ISR(USARTC0_DRE_vect){}

ISR(PCINT2_vect){}

ISR(ADC_vect){}

ISR(TIMER1_CAPT_vect){}

ISR(INT0_vect){}
*/

