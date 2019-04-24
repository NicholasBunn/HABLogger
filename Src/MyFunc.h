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
extern volatile double VMeas;
extern volatile double CMeas;
extern volatile double VPrint;
extern volatile double CPrint;
extern volatile double VPro;
extern volatile double CPro;
extern volatile int PollCnt;
extern volatile int TickTime;
extern volatile int TickTimePrev;
extern volatile double VPrev;
extern volatile double CPrev;
extern volatile int RS;
extern volatile int RW;
extern volatile int DB7;
extern volatile int DB6;
extern volatile int DB5;
extern volatile int DB4;

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSTime[], volatile double GPSLatF, volatile double GPSLongF, volatile float GPSAltF, volatile double CPrint, volatile double VPrint);
int MyCheckSum(volatile char GPSCo[]);
void MyGPSTime(volatile char GPSCo[]);
void CVProcess(volatile double VMeas, volatile double CMeas);
void LCD_Write(volatile int RS, volatile int RW, volatile int DB7, volatile int DB6, volatile int DB5, volatile int DB4);
void LCD_Init();
void Pin_Lift();

#endif /* MYFUNC1_H_ */

