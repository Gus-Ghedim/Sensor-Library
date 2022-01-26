#ifndef MAX_30105_H
#define MAX_30105_H

#include "stdint.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define BUFFER_STORAGE_SIZE 8
#define BUFFER_LENGTH		32
#define FIFO_SIZE 			32

#define MAX30105_OK			1
#define MAX30105_ERROR 		2
#define MAX30105_NOTFOUND 	3


void MAX30105_Init(void);
HAL_StatusTypeDef MAX30105_ReadRegister (uint8_t reg_addr, uint8_t* pData, uint16_t len);
HAL_StatusTypeDef MAX30105_WriteRegister(uint8_t reg_addr, uint8_t reg_value);
void setFIFOAverage(uint8_t numberOfSamples);
void setMAX30105_Config(uint8_t adcRange, uint8_t sampleRate, uint8_t pulseWidth);
void setPulseAmplitudeRed(uint8_t amplitude);
void setPulseAmplitudeIR(uint8_t amplitude);
void setPulseAmplitudeGreen(uint8_t amplitude);
void setLEDMode(uint8_t mode);
void enableSlot(uint8_t slotNumber, uint8_t device);
void clearFIFO(void);
uint8_t WhoIAm(void);
uint8_t softReset(void);
void MAX30105_sleepMode(void);
void MAX30105_WakeUp(void);
uint8_t getWritePointer(void) ;
uint8_t getReadPointer(void);
uint16_t readData(void);

uint8_t getFIFOdata(void);

#endif
