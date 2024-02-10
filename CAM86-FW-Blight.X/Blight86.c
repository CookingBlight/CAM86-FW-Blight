/*  Copyright ? 2017 Gilmanov Rim, Vakulenko Sergiy and Luka Pravica
   
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 2 of the License, or
	(at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
	This firmware needs to run with a compiler optimization level set to 1.
	Otherwise it imports the "zad" functions without doing a call.
	All line numbers are from the Luka hex file.
*/

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL // Verified with fuses.
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>
#include <avr/eeprom.h>

#define VERSION 10

//User configurable definitions
//System specific definitions
#define CYCLE 1020 //ms - note that if you change this value you will need to change cameraGetCoolerPower in the LLD

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;

#define TRUE 1
#define FALSE 0

#define TRUE_INV_PROT 0xaa55
#define FALSE_INV_PROT 0x55aa
#define HIGH_MASK_PROT 0xaa00

#define TEMP_OFFSET 1280 //128C

#define MIN_TEMP (TEMP_OFFSET-500) //-50C
#define MAX_TEMP (TEMP_OFFSET+500) //50C
#define FACTORY_TARGET_TEMP 1730 //25C

#define pixel4		PORTC = k0; \
					PORTC = k1; \
					PORTC = k2; \
					PORTC = k3; \
					PORTC = k4; \
					PORTC = k5; \
					PORTC = k6; \
					PORTC = k7;
#define pixel8		pixel4 \
					pixel4
#define pixel40		pixel8 \
					pixel8 \
					pixel8 \
					pixel8 \
					pixel8
#define pixel200	pixel40 \
					pixel40 \
					pixel40 \
					pixel40 \
					pixel40
#define pixel1000	pixel200 \
					pixel200 \
					pixel200 \
					pixel200 \
					pixel200

#define DHT22_BIT 0x02
#define DHT22_SET PORTB |= DHT22_BIT
#define DHT22_CLR PORTB &= ~DHT22_BIT
#define DHT22_PIN (PINB & DHT22_BIT)

#define COMMAND_READFRAME				0x1b
#define COMMAND_SHIFT3					0x2b
#define COMMAND_OFF15V					0x3b
#define COMMAND_SET_ROISTARTY			0x4b
#define COMMAND_SET_ROINUMY				0x5b
#define COMMAND_SET_EXPOSURE			0x6b
#define COMMAND_SET_BINNING				0x8b
#define COMMAND_ONOFFCOOLER				0x9b
#define COMMAND_SET_TARGETTEMP			0xab
#define COMMAND_CLEARFRAME				0xcb
#define COMMAND_INITMCU					0xdb
#define COMMAND_SET_DELAYPERLINE		0xeb
#define COMMAND_SET_COOLERONDURINGREAD  0xfb
#define COMMAND_SET_COOLERPOWERSTART	0x0a
#define COMMAND_SET_COOLERPOWERMAX		0x1a
#define COMMAND_SET_PIDKP				0x2a
#define COMMAND_SET_PIDKI				0x3a
#define COMMAND_SET_PIDKD				0x4a
#define COMMAND_GET_CASETEMP			0xf1
#define COMMAND_GET_CASEHUM				0xf2
#define COMMAND_GET_CCDTEMP				0xbf
#define COMMAND_GET_TARGETTEMP			0xbe
#define COMMAND_GET_COOLERSTATUS		0xbd
#define COMMAND_GET_COOLERPOWER			0xbc
#define COMMAND_GET_VERSION				0xbb
#define COMMAND_GET_COOLERPOWERSTART	0xba
#define COMMAND_GET_COOLERPOWERMAX		0xb9
#define COMMAND_GET_PIDKP_LW			0xb8
#define COMMAND_GET_PIDKP_HW			0xb7
#define COMMAND_GET_PIDKI_LW			0xb6
#define COMMAND_GET_PIDKI_HW			0xb5
#define COMMAND_GET_PIDKD_LW			0xb4
#define COMMAND_GET_PIDKD_HW			0xb3

uint16_t uy0; 
uint16_t dy;
uint16_t exposure;
uint8_t binning;

uint8_t cmdReceived;
uint8_t command;
uint16_t parcom;

uint16_t sensorTemp;
uint16_t targetTemp;
uint8_t coolerOn;
uint8_t coolerPower;

uint8_t coolerOnDuringReading;

uint16_t targetTempInBuffer;
uint8_t coolerOnInBuffer;
uint16_t sensorTempOutBuffer;
uint8_t coolerPowerOutBuffer;

uint16_t delay_per_line;

uint16_t sensorTempDHTOutBuffer;
uint16_t sensorHumOutBuffer;

uint8_t DHTpr;

uint8_t TECstartingPowerPercentage = 60;
uint8_t TECmaxPowerPercentage = 100;

float U = 0.0;
float Kp = 0.04;
float Ki = 0.0;
float Kd = 0.0;

struct TDHT22_Data
{
	unsigned int Humidity;
	unsigned int Temperature;
};

struct TDHT22_Data DHT22_Data;

/*
uint16_t EEMEM Edelay_per_line;
uint8_t EEMEM ETECstartingPowerPercentage;
uint8_t EEMEM ETECmaxPowerPercentage;
float EEMEM EKp;
float EEMEM EKi;
float EEMEM EKd;
*/

/* Function prototypes */
void initDS12B20(void);
void readDS12B20(void);
void reset(void);
void wbyte(uint8_t byte);
uint8_t rbyte(void);
void PIDLoop (void);
void enforceTECPowerLimits(void);
void initCamera (void);

void frame(void);
void shift(void);
void shift2(void);
void shift3(void);
void zad(void); 
void zad2(uint16_t expp);
void zad3();
void clearline(void);
void clearframe(void);
uint16_t resi(void);
void readDHT22();

// Jump 0x89
// Line 138 - 501
ISR(PCINT0_vect)
// Interrupt, triggered when new command arrives
{
	PCMSK0 = 0x00; // Turn off pin interrupt on the CLK pin while handling the interrupt.
	parcom = resi();
	cmdReceived = TRUE;
	switch (command)
	{
		// read frame
		case COMMAND_READFRAME:
			// Line 170
			frame();
			break;
			// shift3
		case COMMAND_SHIFT3:
			// Line 176
			shift3();
			break;
		// off 15v
		case COMMAND_OFF15V:
			PORTD = 0x4f;
			
			// Line 187
			zad2(1);
			PORTD = 0x4f;
			break;
		// Line 192
		//ROI, StartY
		case COMMAND_SET_ROISTARTY:
			uy0 = parcom;
			break;
		// Line 205
		//ROI, NumY
		case COMMAND_SET_ROINUMY:
			dy = parcom;
			break;
		// Line 218
		//set exposure
		case COMMAND_SET_EXPOSURE:
			exposure = parcom;
			break;
		// Line 231
		//set binning
		case COMMAND_SET_BINNING:
			if ((parcom==TRUE) || (parcom==FALSE))
			{
				binning = parcom;
			}
			break;
		// Line 250
		// on/off cooler
		case COMMAND_ONOFFCOOLER:
			if ((parcom==TRUE) || (parcom==FALSE))
			{
				coolerOnInBuffer = parcom;
				// restart cooler with starting value only if it is not already running
				if ((coolerOnInBuffer == TRUE) && (coolerOn == FALSE)) {
					U = ((float)TECstartingPowerPercentage)/100.0 * CYCLE;
					enforceTECPowerLimits();
				}
			}
			break;
		// Line 310
		// set target temperature
		case COMMAND_SET_TARGETTEMP:
			if ((parcom <= MAX_TEMP) && (parcom >= MIN_TEMP))
			{
				targetTempInBuffer = parcom;
			}
			break;
		// Line 336
		// clear frame
		case COMMAND_CLEARFRAME:
			clearframe();
			break;
		// Line 343
		//Init command - initialize MCU
		case COMMAND_INITMCU:
			initCamera();
			break;
		// Line 349
		//delay per one line
		case COMMAND_SET_DELAYPERLINE:
			delay_per_line = parcom;
			break;
		// Line 362
		// if cooler should be on during the frame reading
		case COMMAND_SET_COOLERONDURINGREAD:
			coolerOnDuringReading = parcom;
			break;
		// Line 371
		// set power that cooler will start at 
		case COMMAND_SET_COOLERPOWERSTART:  
			TECstartingPowerPercentage = parcom;
			break;
		// Line 380
		// set max power to the cooler
		case COMMAND_SET_COOLERPOWERMAX:
			if (parcom > 100)
				TECmaxPowerPercentage = 100;  
			else
				TECmaxPowerPercentage = parcom;
			break;
		// Line 399
		// set TEC KP, Ki and Kd parameter
		// note the parameter is transfered as integer multiplied by 100.0
		// the range of the parameter is 0.01 to 655.35
		// note that we cannot read the parameter in this form as the compiler has issues converting float to int in some cases (see below)
		case COMMAND_SET_PIDKP:
			Kp = parcom / 100.0; // divide by 100.0 to get the real value of the parameter
			break;
		// Line 428
		case COMMAND_SET_PIDKI:
			Ki = parcom / 100.0; // divide by 100.0 to get the real value of the parameter
			break;
		// Line 457
		case COMMAND_SET_PIDKD:
			Kd = parcom / 100.0; // divide by 100.0 to get the real value of the parameter
			break;
		default:
			break;
	}			 
	PCMSK0 = 0x20; // Turn pin interrupt back on for CLK signal.
}

// Call 0x1F5
// Line 502 - 524
void initDS12B20(void)
// Init DS18B20
{
	PORTB &= ~0x04;
	zad2(1);
	PORTB |= 0x04;
	DDRB &= ~0x04; //DDRB = 0x13;
	zad2(1);
	DDRB |= 0x04; //DDRB = 0x17;
	//skip ROM
	wbyte(0xcc);
	// write on scratchPad
	wbyte(0x4e);
	// User byte 0 - Unused
	wbyte(0x00);
	// User byte 1 - Unused
	wbyte(0x00);
	// set up en 12 bits (0x7F)
	wbyte(0x7f);
}

// Call 0x20C
// Line 525 - 651
void readDS12B20(void)
// Read sensor temperature
{
	uint16_t temperature = 0, fract = 0;
	uint8_t sign = 0;
	
	cmdReceived = FALSE;
	
	PORTB &= ~0x04;
	zad2(1);
	PORTB |= 0x04;
	DDRB &= ~0x04; //DDRB = 0x13;
	zad2(1);
	DDRB |= 0x04; //DDRB = 0x17;
	//skip ROM
	wbyte(0xcc);
	//get data
	wbyte(0xbe);
	
	temperature = rbyte();
	temperature = temperature + 256*rbyte();
	
	if ((temperature & 0x8000) != 0x00)
	{
		sign = 1;
		temperature = 0xffff - temperature + 1;
	}
	else 
	{
		// Line 569
		sign = 0;
	}
	fract = 0;
	if ((temperature & 0x01) != 0x00)
	{
		fract=fract+65;
	}
	if ((temperature & 0x02) != 0x00)
	{
		fract=fract+125;
	}
	if ((temperature & 0x04) != 0x00)
	{
		fract=fract+250;
	}
	if ((temperature & 0x08) != 0x00)
	{
		fract=fract+500;
	}
	temperature = (temperature >> 4) * 10 + fract / 100;
	if (sign == 1)
	{
		temperature = TEMP_OFFSET - temperature;
	}
	else
	{
		temperature = temperature + TEMP_OFFSET;
	}
	
	PORTB &= ~0x04;
	
	// Line 617
	zad2(1);
	PORTB |= 0x04;
	DDRB &= ~0x04; //DDRB = 0x13;
	
	// Line 623
	zad2(1);
	DDRB |= 0x04; //DDRB = 0x17;
	//skip ROM
	
	// Line 627
	wbyte(0xcc);
	
	// Line 629
	//start conversion
	wbyte(0x44);
	
	//if interrupt occurs during temperature sensor reading - discard this reading
	if (cmdReceived == FALSE)
	{   
		//there is no error in temperature reading
		if ((temperature != 0x0000) && (temperature != 0xffff))
		{
			sensorTemp = temperature;
		}
	}
}

// Call 0x28B
// Line 652 - 682
void reset(void)
{
	PORTB &= ~0x04;
	zad2(1);
	PORTB |= 0x04;
	DDRB &= ~0x04; //DDRB = 0x13;
	zad2(1);
	DDRB |= 0x04; //DDRB = 0x17;
	wbyte(0xcc);
	zad2(1);
	wbyte(0x4e);
	wbyte(0x00);
	wbyte(0x00);
	wbyte(0x7f);
	zad2(1);
}

// Call 0x2AA
// Line 683 - 722
void wbyte(uint8_t byte)
{
	uint8_t i, j, buf;
	buf = byte;
	for (i = 0; i < 8; i++)
	{
		PORTB &= ~0x04;
		// Compensate for the first loop that gets compiled into oblivion.
		// Need 12 cycles.
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		/*
		// This loop is just compiled into two NOPs.
		for (j = 0; j < 2; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
		}
		*/
		PORTB |= ((buf & 1) << 2) & 0x04;
		for (j = 0; j < 50; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 5 cycles each loop.
		}
		PORTB |= 0x04;
		for (j = 0; j < 100; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 5 cycles each loop.
		}
		buf = buf >> 1;
	}
}

// Call 0x2D2
// Line 723 - 766
uint8_t rbyte(void)
{
	uint8_t i, j, buf = 0;
	for (i = 0; i < 8; i++)
	{
		buf = buf >> 1;
		PORTB &= ~0x04;
		// Compensate for the first loop that gets compiled into oblivion.
		// Need 12 cycles.
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		/*
		// This loop is just compiled into two NOPs.
		for (j = 0; j < 2; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
		}
		*/
		PORTB |= 0x04;
		DDRB &= ~0x04; //DDRB = 0x13;
		for (j = 0; j < 12; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 5 cycles each loop.
		}
		buf |= (PINB & 0x04)<< 5;
		for (j = 0; j < 30; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 5 cycles each loop.
		}
		DDRB |= 0x04; //DDRB = 0x17;
		for (j = 0; j < 100; j++)
		{
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 5 cycles each loop.
		}
	}
	return(buf);
}

// Call 0x2FR
// Line 767 - 769
void Timer_0_ResetVal(void)
{
	TCNT0 = 0; // Prescaller 64 5.3us per tick on 12MHz
}

// Call 0x301
// Line 770 - 771
uint8_t Timer_0_GetVal(void)
{
	return TCNT0; // Prescaller 64 5.3us per tick on 12MHz
}

// Call 0x303
// Line 772 - 950
void readDHT22()
{
	uint16_t _time_slot = 0;
	uint8_t bit_cnt, first_falling, next_falling;
	uint8_t raw_data[5];
	TCNT2 = 0;
	TCCR2B = 7;
	DDRB |= DHT22_BIT;
	DHT22_CLR; // start signal
	
	// Line 787
	zad2(1);
	DHT22_SET;
	DDRB &= ~DHT22_BIT;
	// PORTB |= 0x80;
	
	raw_data[0]=0;
	raw_data[1]=0;
	raw_data[2]=0;
	raw_data[3]=0;
	raw_data[4]=0;
	bit_cnt = 0;
	first_falling = 0;
	next_falling = 0;
	
	while (TCNT2 < 80)
	{
		if(!DHT22_PIN  && !first_falling && !next_falling) // wait falling edge present pulse
		{			   
			first_falling = 1;
			Timer_0_ResetVal();
		}
		if(DHT22_PIN  && first_falling && !next_falling) // wait rising edge
		{
			next_falling = 1;
		}
		if(!DHT22_PIN  && first_falling && next_falling) //  wait falling edge
		{
			next_falling = 0;
			_time_slot = Timer_0_GetVal (); //time between two falling edge
			Timer_0_ResetVal();
			if(bit_cnt > 0) // ignore present pulse
			{
				raw_data[(bit_cnt - 1)>>3] <<= 1;
				if((_time_slot > 14) && (_time_slot < 20 )) //110-160us 1, < 110 0 //22 30
				raw_data[(bit_cnt - 1)>>3] |=1 ;
			}
			bit_cnt++;
			if(bit_cnt == 41) // 40 bit data + 1 bit present
			{
				if((uint8_t)(raw_data[0]+ raw_data[1] + raw_data[2] + raw_data[3]) == raw_data[4])
				{
					DHT22_Data.Humidity = 256*raw_data[0] + raw_data[1];
					DHT22_Data.Temperature = 256*raw_data[2] + raw_data[3]; 
					if(DHT22_Data.Temperature & 0x8000) DHT22_Data.Temperature = 1280 - DHT22_Data.Temperature;
					else DHT22_Data.Temperature = 1280 + DHT22_Data.Temperature;
				}
				goto l2;
			}
		}   
	}
	
	l2: TCCR2B = 0;
	// PORTB &= ~0x80;
}

// Call 0x3B6
// Line 951 - 1 028
void enforceTECPowerLimits(void)
{
	if (U > ((float)TECmaxPowerPercentage)/100.0 * ((float)CYCLE))
		U = ((float)TECmaxPowerPercentage)/100.0 * ((float)CYCLE);

	if (U < 0.0)
		U = 0.0;
}

// Call 0x404
// Line 1 029 - 1 317
void PIDLoop (void)
{
	//float U = 0.0;
	float E = 0.0;
	float lastE = 0.0;
	float accE = 0.0;
	while (1)
	{
		if (DHTpr == 0)
		{
			readDHT22();
		}
		DHTpr++; //1 times per 2 second;
		DHTpr &=1;
		
		readDS12B20();
		
		readDS12B20();
		
		if (coolerOn == FALSE)
		{
			coolerPower=0x00;
			U=0.0;
			E=0.0;
			lastE = 0.0;
			accE = 0.0;
			PORTB &=~0x01;
			_delay_ms(CYCLE);
		}
		else
		{
			// Current error   
			E = (float)sensorTemp - (float)targetTemp;
			// Accumulated error
			accE += E;
			// PID 
			U = Kp * E + Ki * accE + Kd * (E - lastE);
			lastE = E;
			
			enforceTECPowerLimits();
			
			if (U > 0.0)
			{
				PORTB |= 0x01;
			}
			_delay_ms(U);  
			if (((uint16_t) U)!= CYCLE)
			{
				PORTB &=~0x01;
			}
			_delay_ms(CYCLE-U); 
			coolerPower=((uint8_t)(U/4.0));
		}
		//critical section
		cli(); // Global interrupt disable.
		sensorTempOutBuffer = sensorTemp;
		coolerPowerOutBuffer = coolerPower;
		coolerOn = coolerOnInBuffer;
		targetTemp = targetTempInBuffer;
		sensorTempDHTOutBuffer = DHT22_Data.Temperature;
		sensorHumOutBuffer = DHT22_Data.Humidity;
		sei(); // Global interrupt enable.
	}
}

// Call 0x525
// Line 1 318 - 1 374
void initCamera (void)
{
	/*
	// Just testing EEPROM functions.
	TECmaxPowerPercentage = eeprom_read_byte(&ETECmaxPowerPercentage);
	TECstartingPowerPercentage = eeprom_read_byte(&ETECstartingPowerPercentage);
	delay_per_line = eeprom_read_word(&Edelay_per_line);
	Kp = eeprom_read_float(&EKp);
	Ki = eeprom_read_float(&EKi);
	Kd = eeprom_read_float(&EKd);
		
	eeprom_write_byte(&ETECmaxPowerPercentage, TECmaxPowerPercentage);
	eeprom_write_byte(&ETECstartingPowerPercentage,TECstartingPowerPercentage);
	eeprom_write_word(&Edelay_per_line,delay_per_line);
	eeprom_write_float(&EKp,Kp);
	eeprom_write_float(&EKi,Ki);
	eeprom_write_float(&EKd,Kd);
	*/
	
	//8MHz
	CLKPR = 0x80;
	CLKPR = 0x00;
	
	command = 0;
	parcom = 0;
	//k0 = 0x14;
	//k1 = 0x2a;
	binning = 0;
	uy0 = 0;
	dy = 1000;
	exposure = 0;
	delay_per_line = 0;
	cmdReceived = FALSE;
	
	DDRB = 0x14+0x01; //+ PWM_BIT? //DDRB = 0x17;
	PORTB = 0x04;
	DDRD = 0xff;
	PORTD = 0xcf;
	DDRC = 0x3f;
	PORTC = 0x00;
	
	reset();
	
	shift3();
	
	clearframe();
}

// Call 0x55E
// Line 1 375 - 1 377
void Timer_0_Init(void)
{
	TCCR0B = 3; // Prescaller 64 5.3us per tick on 12MHz
}

// Call 0x561
// Line 1 378 - 1 437
int main(void)
{
	// Line 1 378
	initCamera();
	
	// Line 1 379
	Timer_0_Init();
	//init cooler basic variables
	sensorTemp = TEMP_OFFSET;
	targetTemp = FACTORY_TARGET_TEMP;
	coolerOn = FALSE;
	coolerPower = 0;
	
	coolerOnDuringReading = FALSE;
	
	// Line 1 399
	//init temperature sensor, perform first reading
	initDS12B20();
	
	// Line 1 400
	readDS12B20();
	
	// Line 1 401
	_delay_ms(800); //1200);
	//readDS12B20();
	//delay_ms(1200);
	
	//populate buffers
	sensorTempOutBuffer = sensorTemp;
	targetTempInBuffer = targetTemp;
	coolerOnInBuffer = coolerOn;
	coolerPowerOutBuffer = coolerPower;;
	
	// set PCIE0 to enable PCMSK0 scan
	PCICR = 0x01;
	// set PCINT5 to trigger an interrupt on state change
	PCMSK0 = 0x20; // Turn on pin interrupt.
	// turn on interrupts
	sei(); // Global interrupt enable.
	
	PIDLoop();
}

// Call 0x59D
// Line 1 438 - 13881 -> 14 040 - 14 042
void frame(void)
{
	uint16_t y;
	
	// turn off cooling
	if (coolerOn == TRUE)
	{
		if (coolerOnDuringReading == TRUE)
			PORTB |=0x01;
		else
			PORTB &=~0x01;
	}
	if (exposure > 55)
	{
		if (exposure <= 1000) // If exposure is greater than 1000 shift3() has been carried out by the driver.
		{
			// Line 1 465
			shift3(); // Shift pixels to drain to begin the exposure.
		}
		
		// Line 1 472
		zad2(exposure-55);
		clearline(); // Clear the horizontal shift register.
		clearframe(); // Clear the vertical shift registers.
	}
	else
	{
		// Line 1 479
		clearline(); // Clear the horizontal shift register.
		clearframe(); // Clear the vertical shift registers.
		shift3(); // Shift pixels to drain to begin the exposure.
		zad2(exposure);
	}
	
	// Line 1 491
	shift2(); // Move pixels to the shift register.
	
	// Line 1 492
	// Reset the first 24 rows + ROI and cleaning of the horizontal register
	y = 10+uy0;
	do
	{
		shift();
	} while (--y);
	
	// Line 1 505
	clearline();
	
	// Line 1 507
	// To equalize brightness 1 line
	shift();
	
	// Line 1 509
	clearline();
	
	y = dy;
	do
	{
		register unsigned char k0;
		register unsigned char k1;
		register unsigned char k2;
		register unsigned char k3;
		register unsigned char k4;
		register unsigned char k5;
		register unsigned char k6;
		register unsigned char k7;
		register unsigned char k8;
		k8 = 0x28;
		
		// Line 1 519
		shift();
		
		// Line 1 521
		k0 = 0x14;
		k1 = 0x28; // Off s2, no write pulse
		k2 = 0x14;
		k3 = 0x28;
		k4 = 0x14;
		k5 = 0x28;
		k6 = 0x14;
		k7 = 0x28;
		
		// Line 1 537
		pixel40
		pixel8
		
		// 1 633
		k0 = 0x14;
		k1 = 0x2a;
		k2 = 0x14;
		k3 = 0x2a;
		k4 = 0x14;
		k5 = 0x2a;
		k6 = 0x14;
		k7 = 0x2a;
		
		// 4 pix blank (ad9826)
		// Line 1 649
		pixel4
		
		// 1 657
		if (binning == FALSE)
		{
			k0 = 0x14;
			k1 = 0x2a;
			k2 = 0x14;
			k3 = 0x2a;
			k4 = 0x14;
			k5 = 0x2a;
			k6 = 0x14;
			k7 = 0x2a;
		} else
		{
			k0 = 0x14;
			k1 = 0x2a; // Only pulse S2 once.
			k2 = 0x1a;
			k3 = 0x2a;
			k4 = 0x1a;
			k5 = 0x2a;
			k6 = 0x1a;
			k7 = 0x2a;
		}
		
		// Line 1 689
		// Read 6000 bytes without binning, and 1500 bytes with binning.
		pixel1000
		pixel1000
		pixel1000
		pixel1000
		pixel1000
		pixel1000
		PORTC = k8;
		
		// Line 13 690
		k0 = 0x14;
		k1 = 0x28; // Off s2, no write pulse
		k2 = 0x14;
		k3 = 0x28;
		k4 = 0x14;
		k5 = 0x28;
		k6 = 0x14;
		k7 = 0x28;
		
		// Line 13 706
		pixel40
		pixel40
		pixel4
		
		// Line 13 874
		zad3();
	}while (--y);
}

// Call 0x3639
// Line 13 882 - 13 895 -> 13 945 - 13 956
// Shift pixels.
void shift(void)
{
	PORTD = 0xcb;
	zad();
	PORTD = 0xdb;
	zad();
	PORTD = 0xda;
	zad();
	PORTD = 0xfa;
	zad();
	PORTD = 0xea;
	zad();
	PORTD = 0xee;
	zad();
	PORTD = 0xce;
	zad();
	PORTD = 0xcf;
	zad();
}

// Call 0x3647
// Line 13 896 - 13 931 -> 13 945 - 13 956
// Shift pixels to the shift register.
void shift2(void)
{
	shift();
	
	PORTD = 0xc7;
	zad();
	PORTD = 0xc7;
	zad();
	PORTD = 0xc7;
	zad();
	PORTD = 0xc7;
	zad();
	
	PORTD = 0xcb;
	zad();
	
	PORTD = 0xd9;
	zad();
	PORTD = 0xd9;
	zad();
	PORTD = 0xd9;
	zad();
	PORTD = 0xd9;
	zad();
	
	PORTD = 0xdb;
	zad();
	
	PORTD = 0xfa;
	zad();
	PORTD = 0xea;
	zad();
	PORTD = 0xee;
	zad();
	PORTD = 0xce;
	zad();
	PORTD = 0xcf;
	zad();
}

// Call 0x366B
// Line 13 932 - 13 956
// Shift pixels to the drain.
void shift3(void)
{
	PORTD = 0xcb;
	zad();
	PORTD = 0xdb;
	zad();
	PORTD = 0x9a;
	zad();
	PORTD = 0xba;
	zad();
	PORTD = 0xaa;
	zad();
	PORTD = 0xee;
	zad();
	PORTD = 0xce;
	zad();
	PORTD = 0xcf;
	zad();
}

// Call 0x3684
// Line 13 957 - 13 963
void zad()
{
	uint8_t x;
	//x = 7;
	x = 8; // Increased to get 39 cycles.
	do {
		asm("NOP"); // The compiler removes empty loops.
	} while(--x);
} 

// Call 0x368B
// Line 13 964 - 13 986 -> 14 259 - 14 260
void zad2(uint16_t expp)
{
	uint16_t x, y;
	for (y = 0; y < expp; y++)
	{
		x = 1347; 
		do {
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 6 cycles each loop.
		} while(--x);
	}
}

// Call 0x36A2
// Line 13 987 - 14 007 -> 14 026 - 14 029
void zad3()
{
	uint16_t x, y;
	for (y = 0; y < delay_per_line; y++)
	{
		x = 135;
		do {
			asm("NOP"); // The compiler removes empty loops.
			asm("NOP"); // Need 2 NOPs for 6 cycles each loop.
		} while(--x);
	}
}

// Call 0x36B7
// Line 14 008 - 14 029
void clearline()
// Clear the horizontal shift. If it is not cleaned,
// Then gained it a parasitic charge will be added to the first row of image
{
	uint16_t x;
	uint8_t l0,l1;
	
	l0 = 0x14;
	l1 = 0x20;
	
	x = 1600;
	do
	{
		PORTC = l0;
		PORTC = l1;
		PORTC = l0;
		PORTC = l1;
		PORTC = l0;
		PORTC = l1;
		PORTC = l0;
		PORTC = l1;
	} while (--x); 
}

// Call 0x36CD
// Line 14 030 - 14 042
void clearframe(void)
// Clear the shift register. If it is not cleaned,
// The charge accumulated in it will be added to the image.
// Clear the register together with the "dark" and unused lines.
// The operation is performed before "drain" the image into the shift register.
{
	uint16_t x;
	x = 1012;
	do
	{
		shift();
	} while (--x);
	clearline();
}  

// Call 0x36DA
// Line 14 043 - 14 260
uint16_t resi(void)
{
	uint8_t x, count;
	uint16_t buf, buf2;
	
	buf = 0;
	
	// Serial command reading
	for (x = 0;x < 8;x++)
	{
		count = 0;
		while ((PINB & 0x20) == 0)
		{
			count++;
			if (count > 50)
			{
				return buf;
			}
		}
		buf = buf << 1;
		if ((PINB & 0x08) == 0x08)
		{
			buf = buf + 0x0001;
		}
		count = 0;
		while ((PINB & 0x20) != 0)
		{
			count++;
			if (count > 50)
			{
				return buf;
			}
		}
	}
	command = buf;
	
	buf = 0;
	switch (command)
	{
		//send back current Hum & Temp2
		case COMMAND_GET_CASETEMP:
			buf2 = sensorTempDHTOutBuffer;
			break;
		case COMMAND_GET_CASEHUM:
			buf2 = sensorHumOutBuffer;
			break;
		//send back current temperature
		case COMMAND_GET_CCDTEMP:
			buf2 = sensorTempOutBuffer;
			break;
		//send back current target temperature
		case COMMAND_GET_TARGETTEMP:
			buf2 = targetTempInBuffer;
			break;
		//send back current coolen On/Off status
		case COMMAND_GET_COOLERSTATUS:
			if (coolerOnInBuffer == TRUE)
			{
				buf2 = TRUE_INV_PROT;
			}
			else
			{
				buf2 = FALSE_INV_PROT;
			}
			break;
		//send back current cooler power percentage
		case COMMAND_GET_COOLERPOWER:
			buf2 = coolerPowerOutBuffer | HIGH_MASK_PROT;
			break;
		// send back version
		case COMMAND_GET_VERSION:
			buf2 = VERSION;
			break;
		// send back TEC starting power
		case COMMAND_GET_COOLERPOWERSTART:
			buf2 = TECstartingPowerPercentage;
			break;
		// send back TEC max power
		case COMMAND_GET_COOLERPOWERMAX:
			buf2 = TECmaxPowerPercentage;
			break;
		// send back the PID proportional gain lower word
		// need to do it this way as converting float to int sometimes fails for unknown reasons   
		// for example
		// float x = 0.4*1000.0 equals 400.0 while
		// float a = 0.4; 
		// float x = a * 1000.0 equals 100.0
		// ??????
		// we read 4-byte long byte array and convert it to a floating point
		case COMMAND_GET_PIDKP_LW:
			((unsigned char*) (&buf2))[0] = ((unsigned char*) (&Kp))[0];
			((unsigned char*) (&buf2))[1] = ((unsigned char*) (&Kp))[1];
			break;
		// send back the PID proportional gain higher word
		case COMMAND_GET_PIDKP_HW:		
			((unsigned char*) (&buf2))[0] = ((unsigned char*) (&Kp))[2];
			((unsigned char*) (&buf2))[1] = ((unsigned char*) (&Kp))[3];
			break;
		// we read 4-byte long byte array and convert it to a floating point
		case COMMAND_GET_PIDKI_LW:
			((unsigned char*) (&buf2))[0] = ((unsigned char*) (&Ki))[0];
			((unsigned char*) (&buf2))[1] = ((unsigned char*) (&Ki))[1];
			break;
		// send back the PID proportional gain higher word
		case COMMAND_GET_PIDKI_HW:
			((unsigned char*) (&buf2))[0] = ((unsigned char*) (&Ki))[2];
			((unsigned char*) (&buf2))[1] = ((unsigned char*) (&Ki))[3];
			break;
		// we read 4-byte long byte array and convert it to a floating point
		case COMMAND_GET_PIDKD_LW:
			((unsigned char*) (&buf2))[0] = ((unsigned char*) (&Kd))[0];
			((unsigned char*) (&buf2))[1] = ((unsigned char*) (&Kd))[1];
			break;
		// send back the PID proportional gain higher word
		case COMMAND_GET_PIDKD_HW:
			((unsigned char*) (&buf2))[0] = ((unsigned char*) (&Kd))[2];
			((unsigned char*) (&buf2))[1] = ((unsigned char*) (&Kd))[3];
			break;
		default:
			buf2 = 0;
			break;
	}
	// Serial parameter reading
	for (x = 0;x < 16;x++)
	{
		count = 0;
		while ((PINB & 0x20) == 0)
		{
			count++;
			if (count > 50)
			{
				return buf;
			}
		}
		buf = buf << 1;
		if ((PINB & 0x08) == 0x08)
		{
			buf = buf + 0x0001;
		}
		if ((buf2 & 0x8000) == 0)
		{
			PORTB = PORTB & ~0x10;
		}
		else
		{
			PORTB = PORTB | 0x10;
		}
		buf2 = buf2 << 1;
		count = 0;
		while ((PINB & 0x20) != 0)
		{
			count++;
			if (count > 50)
			{
				return buf;
			}
		}
	}
	return buf; 
}