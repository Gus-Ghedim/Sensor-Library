#include "MAX30105.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32f1xx_hal.h"

// Status Registers
static const uint8_t MAX30105_INTSTAT1   = 0x00;
static const uint8_t MAX30105_INTSTAT2   = 0x01;
static const uint8_t MAX30105_INTENABLE1 = 0x02;
static const uint8_t MAX30105_INTENABLE2 = 0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 0x05;
static const uint8_t MAX30105_FIFOREADPTR  = 0x06;
static const uint8_t MAX30105_FIFODATA     = 0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG      = 0x08;
static const uint8_t MAX30105_MODECONFIG      = 0x09;
static const uint8_t MAX30105_PARTICLECONFIG  = 0x0A;
static const uint8_t MAX30105_LED1_PULSEAMP   = 0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP   = 0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP   = 0x0E;
static const uint8_t MAX30105_LED_PROX_AMP    = 0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT    = 0x1F;
static const uint8_t MAX30105_DIETEMPFRAC   = 0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 0xFE;
static const uint8_t MAX30105_PARTID     = 0xFF;  

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_ENABLE  = 	0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30105_INT_DATA_RDY_ENABLE  = 0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_INT_ALC_OVF_ENABLE   = 0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE  = 0x00;

static const uint8_t MAX30105_INT_PROX_INT_ENABLE  = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE  = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_1    = 	0x00;
static const uint8_t MAX30105_SAMPLEAVG_2    = 	0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 	 = 	0x40;
static const uint8_t MAX30105_SAMPLEAVG_8    = 	0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 	 = 	0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 	 = 	0xA0;

static const uint8_t MAX30105_ROLLOVER_ENABLE  = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN 		 = 	0x80;
static const uint8_t MAX30105_WAKEUP 		 = 	0x00;

static const uint8_t MAX30105_RESET 		 = 	0x40;

static const uint8_t MAX30105_MODE_REDONLY   = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED  = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_2048  = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096  = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192  = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_50   = 0x00;
static const uint8_t MAX30105_SAMPLERATE_100  = 0x04;
static const uint8_t MAX30105_SAMPLERATE_200  = 0x08;
static const uint8_t MAX30105_SAMPLERATE_400  = 0x0C;
static const uint8_t MAX30105_SAMPLERATE_800  = 0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_69   = 0x00;
static const uint8_t MAX30105_PULSEWIDTH_118  = 0x01;
static const uint8_t MAX30105_PULSEWIDTH_215  = 0x02;
static const uint8_t MAX30105_PULSEWIDTH_411  = 0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t SLOT_NONE        = 0x00;
static const uint8_t SLOT_RED_LED     = 0x01;
static const uint8_t SLOT_IR_LED      = 0x02;
static const uint8_t SLOT_GREEN_LED   = 0x03;
static const uint8_t SLOT_NONE_PILOT  = 0x04;
static const uint8_t SLOT_RED_PILOT   =	0x05;
static const uint8_t SLOT_IR_PILOT    = 0x06;
static const uint8_t SLOT_GREEN_PILOT = 0x07;

static const uint8_t MAX30105_ADDR = (uint16_t)(0x57<<1);
static const uint8_t MAX30105_ID   = 0x15;

/**
*   \brief Circular buffer for MAX30105 data.
*/
typedef struct 
{
	uint32_t red[BUFFER_STORAGE_SIZE];      ///< Data from RED channel.
	uint32_t IR[BUFFER_STORAGE_SIZE];       ///< Data from IR channel.
	uint32_t green[BUFFER_STORAGE_SIZE];    ///< Data from GREEN channel.
	uint8_t head;   ///< Current head of the circular buffer.
	uint8_t tail;   ///< Current tail of the circular buffer.
} MAX30105_Data; //This is our circular buffer of readings from the sensor

MAX30105_Data sensor_data;

uint8_t activeLEDs = 0;
/*---------------------------------------------------------------------------------------------------------------*/
void MAX30105_Init(void){
	uint8_t SampleAverage = MAX30105_SAMPLEAVG_4;
	uint8_t SampleRate 	  = MAX30105_SAMPLERATE_400;
	uint8_t PulseWidth 	  = MAX30105_PULSEWIDTH_411;
	uint8_t ADCRange      = MAX30105_ADCRANGE_4096;
	uint8_t LedMode 	  = 0x03;
	uint8_t LedPA   	  = 0x1F;
	
	setFIFOAverage(SampleAverage);
	setLEDMode(LedMode);
	setMAX30105_Config(ADCRange, SampleRate, PulseWidth);
	
	setPulseAmplitudeGreen(LedPA);
	setPulseAmplitudeRed(LedPA);
	setPulseAmplitudeIR(LedPA);	
	
	enableSlot(1, SLOT_RED_LED);
	if (LedMode > 1) enableSlot(2, SLOT_IR_LED);
	if (LedMode > 2) enableSlot(3, SLOT_GREEN_LED);
	activeLEDs  = LedMode;
	
	clearFIFO();
}

//Communication Funtions
/*---------------------------------------------------------------------------------------------------------------*/
HAL_StatusTypeDef MAX30105_ReadRegister(uint8_t reg_addr, uint8_t* pData, uint16_t len){
	HAL_StatusTypeDef returnValue = HAL_ERROR;

	//Check if the MAX30105 is ready in a trial of 5 times
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, MAX30105_ADDR, 5, 10);
	if (result == HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	{
		returnValue = (HAL_I2C_Master_Transmit(&hi2c1, MAX30105_ADDR, &reg_addr, 1, 10));
		if(returnValue != HAL_OK)
			return returnValue;
		returnValue = (HAL_I2C_Master_Receive (&hi2c1, MAX30105_ADDR, pData, len, 10));
		return returnValue;
	}
    return returnValue;
}
/*---------------------------------------------------------------------------------------------------------------*/
HAL_StatusTypeDef MAX30105_WriteRegister(uint8_t reg_addr, uint8_t reg_value){
	HAL_StatusTypeDef returnValue = HAL_ERROR;

	//Check if the MAX30105 is ready in a trial of 5 times
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, MAX30105_ADDR, 5, 10);
	if (result == HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	{
		returnValue = HAL_I2C_Mem_Write(&hi2c1, MAX30105_ADDR, reg_addr	, 1, &reg_value, sizeof(reg_value), 10);
		if(returnValue != HAL_OK)
			return returnValue;
	}
    return returnValue;
}
/*---------------------------------------------------------------------------------------------------------------*/

//Set Sample Avarage - See datasheet page 18
/*---------------------------------------------------------------------------------------------------------------*/
void setFIFOAverage(uint8_t numberOfSamples) {
  MAX30105_WriteRegister(MAX30105_FIFOCONFIG, numberOfSamples);
}
/*---------------------------------------------------------------------------------------------------------------*/

//Set ADC Range & Sample Rate & Pulse Width - See datasheet page 19
/*---------------------------------------------------------------------------------------------------------------*/
void setMAX30105_Config(uint8_t adcRange, uint8_t sampleRate, uint8_t pulseWidth) {
  MAX30105_WriteRegister(MAX30105_PARTICLECONFIG, adcRange |sampleRate |pulseWidth);
}
/*---------------------------------------------------------------------------------------------------------------*/

//LED Pulse Amplitude Configuration - See datasheet page 21
/*---------------------------------------------------------------------------------------------------------------*/
void setPulseAmplitudeRed(uint8_t amplitude) {
  MAX30105_WriteRegister(MAX30105_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
  MAX30105_WriteRegister(MAX30105_LED2_PULSEAMP, amplitude);
}

void setPulseAmplitudeGreen(uint8_t amplitude) {
  MAX30105_WriteRegister(MAX30105_LED3_PULSEAMP, amplitude);
}

//Mode Configuration - See datasheet page 19
/*---------------------------------------------------------------------------------------------------------------*/
void setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  MAX30105_WriteRegister(MAX30105_MODECONFIG, mode);
}
/*---------------------------------------------------------------------------------------------------------------*/

//Slot COnfiguration - See datasheet page 22
/*---------------------------------------------------------------------------------------------------------------*/
void enableSlot(uint8_t slotNumber, uint8_t device) {

  switch (slotNumber) {
    case (1):
      MAX30105_WriteRegister(MAX30105_MULTILEDCONFIG1, device);
      break;
    case (2):
      MAX30105_WriteRegister(MAX30105_MULTILEDCONFIG1, device << 4);
      break;
    case (3):
      MAX30105_WriteRegister(MAX30105_MULTILEDCONFIG2, device);
      break;
    case (4):
      MAX30105_WriteRegister(MAX30105_MULTILEDCONFIG2, device << 4);
      break;
    default:
      break;
  }
}
/*---------------------------------------------------------------------------------------------------------------*/

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
/*---------------------------------------------------------------------------------------------------------------*/
void clearFIFO(void) {
  MAX30105_WriteRegister(MAX30105_FIFOWRITEPTR, 0);
  MAX30105_WriteRegister(MAX30105_FIFOOVERFLOW, 0);
  MAX30105_WriteRegister(MAX30105_FIFOREADPTR,  0);
}
/*---------------------------------------------------------------------------------------------------------------*/

//Who I Am (Part ID) - See datasheet page 12
/*---------------------------------------------------------------------------------------------------------------*/
uint8_t WhoIAm(void) {
	uint8_t partID = 0;
	uint8_t status;
	if ((status = MAX30105_ReadRegister(MAX30105_PARTID, &partID,1 )) != (HAL_OK))
		//RETURN FLAG READ ERROR
		return (MAX30105_NOTFOUND);
	else
	{
		if (partID == 0x15)
			return (MAX30105_OK);
		else
			return (MAX30105_ERROR);
	}
}
/*---------------------------------------------------------------------------------------------------------------*/

//Reset MAX30105 - See datasheet page 19
/*---------------------------------------------------------------------------------------------------------------*/
uint8_t softReset(void) {
	uint8_t* response;
	
	MAX30105_WriteRegister(MAX30105_MODECONFIG, MAX30105_RESET);
	HAL_Delay(100);
	
	MAX30105_ReadRegister(MAX30105_MODECONFIG, response, 1);
    if (((int)response & MAX30105_RESET) == 0) {
    	return (MAX30105_OK);
    }
    return (MAX30105_ERROR);
}
/*---------------------------------------------------------------------------------------------------------------*/

//Sleep Mode - See datasheet page 19
/*---------------------------------------------------------------------------------------------------------------*/
void MAX30105_sleepMode(void) {
	MAX30105_WriteRegister(MAX30105_MODECONFIG, MAX30105_SHUTDOWN);
}
/*---------------------------------------------------------------------------------------------------------------*/

//Wake Up MAX30105 - See datasheet page 19
/*---------------------------------------------------------------------------------------------------------------*/
void MAX30105_WakeUp(void) {
	MAX30105_WriteRegister(MAX30105_MODECONFIG, MAX30105_WAKEUP);
}
/*---------------------------------------------------------------------------------------------------------------*/


//Read the FIFO Write Pointer
/*---------------------------------------------------------------------------------------------------------------*/
uint8_t getWritePointer(void) {
	uint8_t fifoReadptr = 0;
	MAX30105_ReadRegister(MAX30105_FIFOWRITEPTR, &fifoReadptr,1);
	return (fifoReadptr);
}

//Read the FIFO Read Pointer
/*---------------------------------------------------------------------------------------------------------------*/
uint8_t getReadPointer(void) {
	uint8_t fifoWrtptr = 0;
	MAX30105_ReadRegister(MAX30105_FIFOREADPTR, &fifoWrtptr,1);
	return (fifoWrtptr);
}
/*---------------------------------------------------------------------------------------------------------------*/

uint8_t getFIFOdata(void) {
	uint8_t fifoWrtptr = 0;
	MAX30105_ReadRegister(MAX30105_FIFODATA, &fifoWrtptr,1);
	return (fifoWrtptr);
}
/*---------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------*/
uint16_t readData(void)
{
  uint8_t readPointer = getReadPointer();
  uint8_t writePointer = getWritePointer();
  int numberOfSamples = 0;

  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += FIFO_SIZE; //Wrap condition

    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
	uint8_t* data;
	MAX30105_ReadRegister(MAX30105_FIFODATA, data, 1);


    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.
		//Trim toGet to be a multiple of the samples we need to read
        toGet = BUFFER_LENGTH - (BUFFER_LENGTH % (activeLEDs * 3)); 
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
	  uint8_t* data_value;
	  HAL_I2C_Mem_Read(&hi2c1, MAX30105_ADDR, MAX30105_FIFODATA	, toGet, data_value, sizeof(data_value), 100);
      
      while (toGet > 0)
      {
    	sensor_data.head++; //Advance the head of the storage struct
    	sensor_data.head %= BUFFER_STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        HAL_I2C_Master_Receive(&hi2c1, MAX30105_ADDR, temp, sizeof(uint32_t), 100);
        //temp[3] = 0;
        //temp[2] = _i2cPort->read();
        //temp[1] = _i2cPort->read();
        //temp[0] = _i2cPort->read();

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
		
		tempLong &= 0x3FFFF; //Zero out all but 18 bits

		sensor_data.red[sensor_data.head] = tempLong; //Store this reading into the MAX30105_Data array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          //temp[3] = 0;
          //temp[2] = _i2cPort->read();
          //temp[1] = _i2cPort->read();
          //temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits
          
		  sensor_data.IR[sensor_data.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          //temp[3] = 0;
          //temp[2] = _i2cPort->read();
          //temp[1] = _i2cPort->read();
          //temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits

		  sensor_data.green[sensor_data.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } 

  }

  return (numberOfSamples);
}
/*---------------------------------------------------------------------------------------------------------------*/
