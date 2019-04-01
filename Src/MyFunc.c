/*
 * MyFunc.c
 *
 *  Created on: 01 Mar 2019
 *      Author: 20336020
 */

#include "MyFunc.h"
#include <math.h>

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSTime[], volatile float GPSLatF)
{
		TimeOn = HAL_GetTick()/1000;
		sprintf(display, "$20336020,%5d,%2.2s:%2.2s:%2.2s,  0,  0,  0,   0,   0,   0,%10.6f,          0,      0,  0,  0\n", (uint8_t)TimeOn, &GPSTime[0], &GPSTime[2], &GPSTime[4], GPSLatF);
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

char MyGPSTime(volatile char GPSCo[])
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
		if(ComCnt == 1) {
			GPSTime[x] = GPSCo[ci];
			x++;
		} else if(ComCnt == 2) {
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
		} else if(ComCnt == 3 && (GPSCo[ci] == 'S' || GPSCo[ci] == 'N')) {
			if(GPSCo[ci] == 'S') {
				GPSLatF = GPSLatF*(-1);
			} else {
				//Do nothing, it's positive
			}
			sprintf(GPSLatS, "%f", GPSLatF);
		}
		ci++;
	}
	return 0;
}

