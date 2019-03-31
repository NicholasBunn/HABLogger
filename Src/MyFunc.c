/*
 * MyFunc.c
 *
 *  Created on: 01 Mar 2019
 *      Author: 20336020
 */

#include "MyFunc.h"

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSCo[])
{
		TimeOn = HAL_GetTick()/1000;
		sprintf(display, "$20336020,%5d,00:00:00,  0,  0,  0,   0,   0,   0,        0,          0,      0,  0,  0\n", (uint8_t)TimeOn);
		HAL_UART_Transmit(&huart1, (uint8_t*)display, 91, 1000);
}

int MyCheckSum(volatile char tempbuf[])
{
	int i;
	int Checksum = 0;
	char Character;
	while(tempbuf[j-2] != '\r' && tempbuf[j-1] != '\n') {
		GPSCo[j] = tempbuf[j];
		j++;
	}
	for (i=0; i<(j-3); ++i)
	{
		Character = GPSCo[i];
		switch(Character)
		{
		case '$':
			break;
		case '*':
			i = (j-3);
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
return (Checksum);
}

char MyGPSTime(volatile char GPSCo[])
{
	int ComCnt = 0;
	int ci = 0;
	int x = 0;
	while(x < 6) {
		if(GPSCo[ci] == ',') {
			ComCnt++;
			ci++;
		}
		if(ComCnt == 1) {
			GPSCo[ci] = GPSTime[x];
			x++;
		}
		ci++;
	}
	return 0;
}
