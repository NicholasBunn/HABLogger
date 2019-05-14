/*
 * MyFunc.c
 *
 *  Created on: 01 Mar 2019
 *      Author: 20336020
 */

#include "MyFunc.h"
#include "main.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "lis2dh12_reg.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
I2C_HandleTypeDef hi2c1;

void MyPrintFunc(volatile uint8_t TimeOn, volatile char GPSTime[], volatile double GPSLatF, volatile double GPSLongF, volatile float GPSAltF, volatile double CPrint, volatile double VPrint, volatile int8_t BME_T, volatile double BME_P, volatile double BME_H)
{
		TimeOn = HAL_GetTick()/1000;
		sprintf(display, "$20336020,%5d,%2.2s:%2.2s:%2.2s,%3d,%3.0f,%3.0f,%4.0f,%4.0f,%4.0f,%10.6f,%11.6f,%7.1f,%3.0lf,%3.1lf\n", (uint8_t)TimeOn, &GPSTime[0], &GPSTime[2], &GPSTime[4], BME_T, BME_H,BME_P, acceleration_mg[2], acceleration_mg[1], acceleration_mg[0], GPSLatF, GPSLongF, GPSAltF, CPrint, VPrint);
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
	VPro = VPro*12.1;
	CPro = CMeas/4096;
	CPro = CPro*95.8;
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
	if(RS) {
		GPIOB -> ODR |= GPIO_PIN_15;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_15;
	}

	if(RW) {
		GPIOB -> ODR |= GPIO_PIN_1;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_1;
	}

	if(DB7) {
		GPIOA -> ODR |= GPIO_PIN_12;
	} else {
		GPIOA -> ODR &= ~GPIO_PIN_12;
	}

	if(DB6) {
		GPIOA -> ODR |= GPIO_PIN_11;
	} else {
		GPIOA -> ODR &= ~GPIO_PIN_11;
	}

	if(DB5) {
		GPIOB -> ODR |= GPIO_PIN_12;
	} else {
		GPIOB -> ODR &= ~GPIO_PIN_12;
	}

	if(DB4) {
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
	LCD_Write(0,0,1,1,0,0);

	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,0,0,0,1);

	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,0,1,1,0);
}

void LCD_Conv(int Burn) {
	/*LCD_Write(0,0,1,0,0,0);
	LCD_Write(0,0,0,0,0,0);*/
	//Clear display
	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,0,0,0,1);
	GPSAltI = 0;
	uint8_t a0;
	uint8_t a1;
	uint8_t a2;
	uint8_t a3;
	uint8_t a4;
	uint8_t a5;
	uint8_t a6;
	uint8_t a7;
	uint8_t b0;
	uint8_t b1;
	uint8_t b2;
	uint8_t b3;
	uint8_t b4;
	uint8_t b5;
	uint8_t b6;
	uint8_t b7;
	for(int i=0;i<7;i++) {
		GPSAltI = (int)GPSAlt[i];
		if(GPSAltI < 48) {
			i = 7;
		} else {
			a0 = GPSAltI&0b10000000;
			a1 = GPSAltI&0b01000000;
			a2 = GPSAltI&0b00100000;
			a3 = GPSAltI&0b00010000;
			a4 = GPSAltI&0b00001000;
			a5 = GPSAltI&0b00000100;
			a6 = GPSAltI&0b00000010;
			a7 = GPSAltI&0b00000001;

			LCD_Write(1,0,a0,a1,a2,a3);
			LCD_Write(1,0,a4,a5,a6,a7);
		}
	}
	LCD_Write(1,0,0,1,1,0);
	LCD_Write(1,0,1,1,0,1);


//Burn
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
	/*LCD_Write(0,0,1,1,0,0);
	LCD_Write(0,0,0,1,0,0);

	LCD_Write(1,0,0,1,0,1);
	LCD_Write(1,0,0,1,0,0);*/

	LCD_Write(0,0,1,1,0,0);
	LCD_Write(0,0,0,1,0,1);

	for(int i=0;i<7;i++) {
			BME_T_I = (int)BME_T_s[i];

			b0 = BME_T_I&0b10000000;
			b1 = BME_T_I&0b01000000;
			b2 = BME_T_I&0b00100000;
			b3 = BME_T_I&0b00010000;
			b4 = BME_T_I&0b00001000;
			b5 = BME_T_I&0b00000100;
			b6 = BME_T_I&0b00000010;
			b7 = BME_T_I&0b00000001;

			LCD_Write(1,0,b0,b1,b2,b3);
			LCD_Write(1,0,b4,b5,b6,b7);
		}
//Reset Cursor
	LCD_Write(0,0,0,0,0,0);
	LCD_Write(0,0,0,0,1,0);

}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
	HAL_Delay(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */
    dev_id = dev_id << 1; //11101101
    HAL_I2C_Master_Transmit(&hi2c1,0b11101101,&reg_addr,1,5);
    HAL_I2C_Master_Receive(&hi2c1,0b11101101,reg_data,len,5);
    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */
    uint8_t *buf;
    buf = malloc(len +1);
    buf[0] = reg_addr;
    memcpy(buf +1, reg_data, len);
    HAL_I2C_Master_Transmit(&hi2c1,0b11101100,buf,len +1,5); //write(fd, buf, len +1)
    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}

void Struct_Init() {
	rslt = BME280_OK;

	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);
}

int8_t My_BME_Config() {
	int8_t rslt;
	uint8_t settings_sel;

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	return rslt;
}

void Get_BME_Data() {
	bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	BME_H = (comp_data.humidity)/1024;
	BME_P = (comp_data.pressure)/100000;
	BME_T = (comp_data.temperature)/100;
	sprintf(BME_T_s, "%dC", BME_T);
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LIS2DH12_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Write multiple command */
    reg |= 0x40;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LIS2DH12_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Read multiple command */
    reg |= 0xC0;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

static void tx_com(volatile uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

static void platform_init(void)
{
#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_Delay(1000);
#endif
}

void Accel_Init() {

	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;

	platform_init();

	lis2dh12_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LIS2DH12_ID)
	{
	  while(1)
	  {
	    /* manage here device not found */
	  }
	}

	/*
	*  Enable Block Data Update
	*/
	lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/*
	* Set Output Data Rate to 1Hz
	*/
	lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);
	/*
	* Set full scale to 2g
	*/
	lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);

	lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);
}

void Accel_Process() {
	lis2dh12_reg_t reg;

	/*
    * Read output only if new value available
    */
    lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
    if(reg.byte)
    {
      /* Read accelerometer data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] =
        lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] =
        lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] =
        lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[2]);

      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));

      acceleration_mg[0] = -acceleration_mg[0]; //z
      acceleration_mg[1] = acceleration_mg[1] - 18; //y
      acceleration_mg[2] = -acceleration_mg[2] - 8; //x
      for(int lc=0;lc<3;lc++) {
    	  if(acceleration_mg[lc] >= 999) {
    		  acceleration_mg[lc] = 999;
    	  } else if(acceleration_mg[lc] <= -999) {
    		  acceleration_mg[lc] = -999;
    	  } else {
    		  //Do nothing
    	  }
      }
    }
}


