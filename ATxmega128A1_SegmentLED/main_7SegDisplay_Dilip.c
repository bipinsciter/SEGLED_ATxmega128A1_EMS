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

// Convenience macro for enabling pull-ups on specified pins on any port.
#define __PORT_PULLUP(port, mask) { \
	PORTCFG.MPCMASK = mask ; \
	port.PIN0CTRL = PORT_OPC_PULLUP_gc; \
}


//***********************************************************************************


#define NO_DIGIT	13
#define PARAMETER_WORD	(ENABLE_DP1 | ENABLE_DP2 | ENABLE_TEMP | ENABLE_RH | ENABLE_LOGO | ENABLE_RTC | ENABLE_LCD | ENABLE_ALERT)

unsigned short gu16_parameterWord = PARAMETER_WORD;

//*************************************************************************
#define DEFAULT_LCD_BRIGHTNESS	 10
#define DEFAULT_AUTO_SENT_INTERVAL	 5
#define DEFAULT_XBEE_RST_INTERVAL	 360
#define DEFAULT_DEVICES_IN_GROUP	 5

//Display Macro ***********************************************************
#define TEMP_DISPLAY			0
#define RH_DISPLAY				1
#define DP1_DISPLAY				2
#define DP2_DISPLAY				3
#define NO_DISPLAY				4

//*************************************************************************
// DISPLAY CONSTANTS
//*************************************************************************
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

//*************************************************************************
#define SHR(a, b) (-1 >> 1 == -1 ? (a) >> (b) : (a) / (1 << (b)) - ((a) % (1 << (b)) < 0))

#define EPOCH_YEAR				1970
#define TM_YEAR_BASE			2000

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

//**********************************************************************************************************
//											VARIABLE DECLARATION
//**********************************************************************************************************
volatile static struct bits
{
	uint8_t AM_PM_Flag : 1;	
	uint8_t Sec_blink_flag : 1;	
	uint8_t RTCChangeOccure : 1;
	uint8_t sec_flag : 1;	
	uint8_t msec_flag : 1;	
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
	uint8_t doorSense : 1;
	uint8_t AlarmLED : 1;
	uint8_t buzzeralert : 1;
	uint8_t autoSendResponse : 1;
	uint8_t triggerXbeeReset : 1;
	
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
	0x10,  //code for I
	0x71,  //code for F
	0x3E,  //code for U
	0x74,  //code for H
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
unsigned short Raw_pressure_cnt1[RAW_DP_CNT_IND];
unsigned long Avg_Raw_pressure_cnt1=0;

uint8_t Raw_pressure_cnt_ind2=0;
unsigned short Raw_pressure_cnt2[RAW_DP_CNT_IND];
unsigned long Avg_Raw_pressure_cnt2=0;

float Dpressure1=0.0,Dpressure2=0.0;

uint8_t FirstTimeCheck=0;	
uint8_t progTimeout=0;

uint8_t prog_cnt=0;
uint8_t mode=NORMAL_MODE;
	
uint8_t keybyte=0;
uint8_t debounce=DEBOUNCE;

float tempfloat=0.0,tempfloat1=0.0,tempfloat2=0.0,f32_dp_sw_factor[5]={0,0,0,0,0};
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
signed short su16_dp_sw_factor[5]={0,0,0,0,0};

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
static inline int leapyear (long int year);
unsigned long  ydhms_diff (long int year1, long int yday1, int hour1, int min1, int sec1,int year0, int yday0, int hour0, int min0, int sec0);
unsigned long get_epoch_time(struct RTCData);
void get_date_time(struct RTCData* t1,unsigned long epoch);
//-------------------RS485 ROUTINE-------------------------------------------------------------------------------
void InitUSART(USART_t * usart, PORT_t * port, uint8_t rxpin, uint8_t txpin, uint16_t ubrr, uint8_t noOfbits, uint8_t parity, uint8_t stopbit);
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

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	InitSystemClock();	
	DisableUnusedModules();
	Init_GPIO();
	
	BUZZER_ON;
	XBEE_RST_LOW;
	
	//-------------------------------------------------------
	//Initialize Variables
	//-------------------------------------------------------
	Init_variables();
	
	//-------------------------------------------------------
	//Initialize I2C for RTC, SHT25
	//-------------------------------------------------------
	I2C_Init();	
	
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
	//Initialize RTC
	//-------------------------------------------------------
	Init_InternalRTC();
	
	//-------------------------------------------------------
	//Initialize DMA for UART Data transfer
	//-------------------------------------------------------
	Init_DMA();
	
	// Turn Interrupts on.
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();			//Global Interrupt Enable
	

 	while(1)
 	{	
		if(b.resetDevice) 	
		{
			while(1);
		}	
						
		//Serve Watchdog Timer
		wdt_reset();

		//1 Second Tick ====================================================
		if(b.sec_flag)
		{
			SecondTick();

			b.sec_flag=0;
		}

		//====================================================
		if(gu16_parameterWord & ENABLE_LCD)
		{		
			conv_value();
			disp_value();	
		}
 	}//END OF WHILE LOOP
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
	
	RTC.PER = 511;
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	RTC_INTCTRL = RTC_OVFINTLVL_LO_gc;
}

ISR(RTC_OVF_vect)
{
	static uint8_t mcnt=0;
	
	b.mec500_blink_flag ^= 1;
	b.led_toggle^=1;
	b.AlarmLED=1;
	//---------------------------------------------
	mcnt++;
	if(mcnt>=2)
	{
		mcnt=0;
		b.sec_flag=1;
	}
	
	b.msec_flag=1;
	//---------------------------------------------
}
	
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

//**************************************************************************************************************************************

void SecondTick(void)
{
	static uint8_t acnt1=0;
	
	b.batteryPerBlink ^= 1;
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
	
	_delay_ms(12000);
	
	TM1680Blink(BLINK_OFF);
	//-------------------------------------------------------------------
	
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	data[1] = V;
	data[2] = E;
	data[3] = r;
	
	data[5] = 11;
	data[6] = 0;
						
	disp_value();
	
	wdt_reset();
	
	//----------Just to check display segment healthiness------------
	_delay_ms(4000);
	
	//-------------------------------------------------------------------
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	data[2] = I;
	data[3] = D;
	convert_char(DeviceID,&data[4],3);
	disp_value();
		
	wdt_reset();
	
	_delay_ms(4000);
	
	//-------------------------------------------------------------------
	for(i=0;i<NO_DIGIT;i++) data[i]=BLANK;
	data[2] = 5;
	data[3] = r;

	convert_float(gu32_SrNumber,&data[4],0);
	disp_value();
	
	wdt_reset();
	
	_delay_ms(4000);
	
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
	if(b.doorSense)
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

//****************************************************************************************************************************************/
void delay(unsigned int cnt)
{
	while(cnt--)
	{
		//wdt_reset();
	}
}

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
