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

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSTime[], volatile double GPSLatF, volatile double GPSLongF, volatile char GPSAlt[])
{
		TimeOn = HAL_GetTick()/1000;
		sprintf(display, "$20336020,%5d,%2.2s:%2.2s:%2.2s,  0,  0,  0,   0,   0,   0,%10.6f,%11.6f,%7s,  0,  0\n", (uint8_t)TimeOn, &GPSTime[0], &GPSTime[2], &GPSTime[4], GPSLatF, GPSLongF, &GPSAlt[0]);
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

