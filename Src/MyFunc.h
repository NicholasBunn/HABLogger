/*
 * MyFunc.h
 *
 *  Created on: 01 Mar 2019
 *      Author: 20336020
 */

#ifndef MYFUNC_H_
#define MYFUNC_H_
#include "stm32f3xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern char display[91];
extern volatile uint8_t TimeOn;
extern volatile char tempbuf[];
extern volatile char GPSCo[];
extern volatile uint8_t j;
extern volatile char GPSTime[];

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSCo[]);
int MyCheckSum(volatile char tempbuf[]);
char MyGPSTime(volatile char GPSCo[]);

#endif /* MYFUNC1_H_ */

