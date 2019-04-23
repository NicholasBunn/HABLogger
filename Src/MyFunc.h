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
extern volatile char GPSLat[];
extern volatile double GPSLatF;
extern volatile char GPSLatS[];
extern volatile char GPSLong[];
extern volatile double GPSLongF;
extern volatile char GPSLats[];
extern volatile char GPSAlt[];
extern volatile float GPSAltF;
extern volatile float VMeas;
extern volatile float CMeas;

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSTime[], volatile double GPSLatF, volatile double GPSLongF, volatile float GPSAltF, volatile float CMeas, volatile float VMeas);
int MyCheckSum(volatile char GPSCo[]);
void MyGPSTime(volatile char GPSCo[]);
void VProcess(volatile float VMeas);
void CProcess(volatile float CMeas);

#endif /* MYFUNC1_H_ */

