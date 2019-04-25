/*
 * MyFunc.c
 *
 *  Created on: 01 Mar 2019
 *      Author: 20336020
 */

#include "MyFunc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSTime[], volatile double GPSLatF, volatile double GPSLongF, volatile float GPSAltF, volatile double CPrint, volatile double VPrint)
{
		TimeOn = HAL_GetTick()/1000;
		sprintf(display, "$20336020,%5d,%2.2s:%2.2s:%2.2s,  0,  0,  0,   0,   0,   0,%10.6f,%11.6f,%7.1f,%3.0lf,%3.1lf\n", (uint8_t)TimeOn, &GPSTime[0], &GPSTime[2], &GPSTime[4], GPSLatF, GPSLongF, GPSAltF, CPrint, VPrint);
		HAL_UART_Transmit(&huart1, (uint8_t*)display, 91, 1000);
}

int MyCheckSum(volatile char GPSCo[])
{
	int i;
	int x = 0;
	int y = 0;
	int MyCheck;
	int Checksum = 0;
	char Character;

	for (i=0; i<(j-3); ++i)
	{
		Character = GPSCo[i];
		switch(Character)
		{
		case '$':
			break;
		case '*':
			i = (j-3);
			x = GPSCo[i-1];
			y = GPSCo[i];
			continue;
		default:
			if (Checksum == 0)
			{
				Checksum = Character;
			}
			else
			{
				Checksum = Checksum ^ Character;
			}
			break;
		}
	}
	if(x > 64) {
		x = x - 55;
	} else {
		x = x - 48;
	}
	if(y > 64) {
		y = y - 55;
	} else {
		y = y - 48;
	}
	MyCheck = (x*16) + y;
	if (MyCheck == Checksum) {
		return 1;
	}
	else {
		return 0;
	}
}

void MyGPSTime(volatile char GPSCo[])
{
	int ComCnt = 0;
	int ci = 0;
	int x = 0;
	while(ComCnt <= 13) {
		if(GPSCo[ci] == ',') {
			ComCnt++;
			ci++;
			x = 0;
		}
		if(ComCnt == 1 && GPSCo[ci] != ',') {
			GPSTime[x] = GPSCo[ci];
			x++;
		} else if(ComCnt == 2 && GPSCo[ci] != ',') {
			GPSLat[x] = GPSCo[ci];
			if (x == 0) {
				GPSLatF = ((GPSLat[x] - 48)*10);
			} else if (x == 1) {
				GPSLatF = GPSLatF + (GPSLat[x] - 48);
			} else if(x == 2) {
				GPSLatF = GPSLatF + ( (float)(GPSLat[x] - 48)/6 );
			} else if (x == 3) {
				GPSLatF = GPSLatF + ( (float)(GPSLat[x] - 48)/60 );
			} else if (x == 4) {
				//Do nothing, it's a full stop
			} else {
				GPSLatF = GPSLatF + ( (float)(GPSLat[x] - 48)/(60*(pow(10, (x-4)))) );
			}
			x++;
		} else if(ComCnt == 3 && (GPSCo[ci] == 'S' || GPSCo[ci] == 'N') && GPSCo[ci] != ',') {
			if(GPSCo[ci] == 'S') {
				GPSLatF = GPSLatF*(-1);
			} else {
				//Do nothing, it's positive
			}
			// CHECK THIS sprintf(GPSLatS, "%f", GPSLatF);
		} else if(ComCnt == 4 && GPSCo[ci] != ',') {
			GPSLong[x] = GPSCo[ci];
			if (x == 0) {
				GPSLongF = ((GPSLong[x] - 48)*100);
			} else if (x == 1) {
				GPSLongF = GPSLongF + ((GPSLong[x] - 48)*10);
			} else if(x == 2) {
				GPSLongF = GPSLongF + ((GPSLong[x] - 48));
			} else if(x == 3) {
				GPSLongF = GPSLongF + ( (float)(GPSLong[x] - 48)/6 );
			} else if (x == 4) {
				GPSLongF = GPSLongF + ( (float)(GPSLong[x] - 48)/60 );
			} else if (x == 5) {
				//Do nothing, it's a full stop
			} else {
				GPSLongF = GPSLongF + ( (float)(GPSLong[x] - 48)/(60*(pow(10, (x-5)))) );
			}
			x++;
		} else if(ComCnt == 5 && (GPSCo[ci] == 'E' || GPSCo[ci] == 'W') && GPSCo[ci] != ',') {
			if(GPSCo[ci] == 'W') {
				GPSLongF = GPSLongF*(-1);
			} else {
				//Do nothing, east is positive
			}
		} else if(ComCnt == 9 && GPSCo[ci] != ',') {
			GPSAlt[x] = GPSCo[ci];
			GPSAltF = atof(GPSAlt);
			x++;
		} else {
			//Do nothing
		}
		if(GPSCo[ci-1] == ',' && GPSCo[ci] == ',') {
			//Do nothing
		} else {
		ci++;
		}
	}
}

void CVProcess(volatile double VMeas, volatile double CMeas) {
	//Process Voltage
	VPro = VMeas/4096;
	VPro = VPro*12.6;
	CPro = CMeas/4096;
	CPro = CPro*100;
	if(PollCnt <= 19) {
		VPrev = VPrev + VPro;
		CPrev = CPrev + CPro;
		TickTimePrev = TickTime;
		PollCnt++;
	} else {
		VPrint = VPrev/20;
		VPrev = 0;
		CPrint = CPrev/20;
		CPrev = 0;
		TickTimePrev = TickTime;
		PollCnt = 0;
	}
}

void LCD_Write(volatile int RS, volatile int RW, volatile int DB7, volatile int DB6, volatile int DB5, volatile int DB4) {
	GPIOB -> ODR |= GPIO_PIN_2;
	if(RS == 1) {
		GPIOB -> ODR |= GPIO_PIN_15;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_15;
	}

	if(RW == 1) {
		GPIOB -> ODR |= GPIO_PIN_1;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_1;
	}

	if(DB7 == 1) {
		GPIOA -> ODR |= GPIO_PIN_12;
	} else {
		GPIOA -> ODR &= ~GPIO_PIN_12;
	}

	if(DB6 == 1) {
		GPIOA -> ODR |= GPIO_PIN_11;
	} else {
		GPIOA -> ODR &= ~GPIO_PIN_11;
	}

	if(DB5 == 1) {
		GPIOB -> ODR |= GPIO_PIN_12;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_12;
	}

	if(DB4 == 1) {
		GPIOB -> ODR |= GPIO_PIN_11;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_11;
	}
	HAL_Delay(2);
	GPIOB -> ODR &= ~GPIO_PIN_2;
}

void LCD_Init() {
	GPIOB -> ODR |= GPIO_PIN_2;

	HAL_Delay(20);
	LCD_Write(0,0,0,0,1,1);

	HAL_Delay(5);
	LCD_Write(0,0,0,0,1,1);

	HAL_Delay(1);
	LCD_Write(0,0,0,0,1,1);

	LCD_Write(0,0,0,0,1,0);

	LCD_Write(0,0,0,0,1,0);
	LCD_Write(0,0,1,1,0,0);

	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,1,1,1,1);

	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,0,0,0,1);

	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,0,1,1,0);
}

void LCD_Conv(char d, int Burn) {
	for(int i=0;i<7;i++) {
		int a6 = (i >> 6) & 0b00000001;
		int a5 = (i >> 5) & 0b00000001;
		int a4 = (i >> 4) & 0b00000001;
		int a3 = (i >> 3) & 0b00000001;
		int a2 = (i >> 2) & 0b00000001;
		int a1 = (i >> 1) & 0b00000001;
		int a0 = (i >> 0) & 0b00000001;

		LCD_Write(0,0,1,a6,a5,a4);
		LCD_Write(0,0,a3,a2,a1,a0);

		int d7 = (d >> 7) & 0b00000001;
		int d6 = (d >> 6) & 0b00000001;
		int d5 = (d >> 5) & 0b00000001;
		int d4 = (d >> 4) & 0b00000001;
		int d3 = (d >> 3) & 0b00000001;
		int d2 = (d >> 2) & 0b00000001;
		int d1 = (d >> 1) & 0b00000001;
		int d0 = (d  >> 0) & 0b00000001;

		LCD_Write(1,0,d7,d6,d5,d4);
		LCD_Write(1,0,d3,d2,d1,d0);
	}

	if(Burn == 1) {
		LCD_Write(0,0,1,1,0,0);
		LCD_Write(0,0,0,0,0,1);

		LCD_Write(1,0,0,1,0,0);
		LCD_Write(1,0,0,0,1,0);
	} else {
		LCD_Write(0,0,1,1,0,0);
		LCD_Write(0,0,0,0,0,1);

		LCD_Write(1,0,1,0,0,0);
		LCD_Write(1,0,0,0,0,0);
	}
//Temp
	LCD_Write(0,0,1,1,0,0);
	LCD_Write(0,0,0,1,0,0);

	LCD_Write(1,0,0,1,0,1);
	LCD_Write(1,0,0,1,0,0);

	LCD_Write(0,0,1,1,0,0);
	LCD_Write(0,0,0,1,0,1);

	LCD_Write(1,0,0,1,0,1);
	LCD_Write(1,0,0,1,0,0);

	LCD_Write(0,0,1,1,0,0);
	LCD_Write(0,0,0,1,1,0);

	LCD_Write(1,0,0,1,0,1);
	LCD_Write(1,0,0,1,0,0);

	LCD_Write(0,0,1,1,0,0);
	LCD_Write(0,0,0,1,1,1);

	LCD_Write(1,0,0,1,0,0);
	LCD_Write(1,0,0,0,1,1);
}

void LCD_Print(volatile char GPSAlt[], volatile int Burn) {
	LCD_Write(1,0,GPSAlt[0],GPSAlt[1],GPSAlt[2],GPSAlt[3]);
	LCD_Write(1,0,GPSAlt[4],GPSAlt[5],GPSAlt[6],GPSAlt[7]);
}
