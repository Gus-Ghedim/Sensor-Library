/**
 * @brief: Library for Temperature Sensor PCT 2075 from NXP
 * @autor: Gustavo Ghedim
 * @date: 11/01/2022
 * @version: v1.0
 */

/**
 * @details: At main.c add the structure "PCT2075_t", from there it is possible to
 * have access to the raw data, temperature, and hysteresis and over temperature.
 *
 * In the PCT2075_Init() function you can enter the sensor settings, described in the enumeration at the idle time (maximum value: 15)
 *
 * In the setTemperatureBoundary() is used to set the hysteresis and over temperature.
 */

/**
 * @note: It is necessary to enter the correct sensor address at **PCT2075_ADDR**
 */
#ifndef _PCT2075_H
#define _PCT2075_H

 #include "stm32f4xx_hal.h"
 #include "stdint.h"

#define PCT2075_ADDR    0x00////ENTER ADDRESS

#define PCT2075_CONF	0x01
#define PCT2075_TEMP	0x00
#define PCT2075_Tos		0x03
#define PCT2075_Thyst	0x02
#define PCT2075_Tiddle	0x04

typedef enum{
	SHUTDOWN_ON,
	SHUTDOWN_OFF
}Shutdown_t;

typedef enum{
	OS_F_QUE_1,
	OS_F_QUE_2,
	OS_F_QUE_4,
	OS_F_QUE_6
}OS_F_Queue_t;

typedef enum{
	OS_POL_LOW,
	OS_POL_HIGH
}OS_POL_t;

typedef enum{
	OS_COMP,
	OS_INT
}OS_COMP_INT_t;

typedef enum{
	PCT2075_OK,
	PCT2075_ERROR,
	PCT2075_INVALID
}PCT2075_STATUS_t;

typedef struct{
	uint8_t temp_data[2];
	int16_t Tos_data;
	int16_t Thyst_data;
	float temperature;	
}PCT2075_t;

HAL_StatusTypeDef  PCT2075_ReadRegister(uint8_t reg_addr, uint8_t* reg_value, uint8_t size);
HAL_StatusTypeDef  PCT2075_WriteRegister(uint8_t reg_addr, uint8_t reg_value, uint8_t size);
PCT2075_STATUS_t PCT2075_Init (I2C_HandleTypeDef *I2Cx, OS_F_Queue_t OS_F_Queue, OS_POL_t OS_POL, OS_COMP_INT_t OS_COMP_INT, uint16_t time);
PCT2075_STATUS_t iddleTime(uint8_t value);
PCT2075_STATUS_t readTemperature (void);
PCT2075_STATUS_t setTemperatureBoundary (int16_t Tos, int16_t Thyst);
PCT2075_STATUS_t PCT2075_shutDown(void);
PCT2075_STATUS_t PCT2075_wakeUp(void);
uint16_t twoComplement(uint16_t value);


#endif
