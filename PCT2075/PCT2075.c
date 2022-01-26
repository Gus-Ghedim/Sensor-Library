#include "PCT2075.h"

PCT2075_t PCT2075;

I2C_HandleTypeDef *I2CHndl;

HAL_StatusTypeDef  PCT2075_ReadRegister(uint8_t reg_addr, uint8_t* reg_value, uint8_t size){
	if (HAL_I2C_Master_Transmit(I2CHndl, PCT2075_ADDR, &reg_addr, sizeof(reg_addr), HAL_MAX_DELAY) != HAL_OK) {
			return HAL_ERROR;
	}
	else{
		if(HAL_I2C_Mem_Read(I2CHndl, PCT2075_ADDR, reg_addr, 1, reg_value, size, HAL_MAX_DELAY) != HAL_OK){
			return HAL_ERROR;
		}
	}
    return HAL_OK;
}

HAL_StatusTypeDef  PCT2075_WriteRegister(uint8_t reg_addr, uint8_t reg_value, uint8_t size ){
	
	if (HAL_I2C_Mem_Write(I2CHndl, PCT2075_ADDR, reg_addr, size, &reg_value, sizeof(reg_value), HAL_MAX_DELAY) != HAL_OK){
		return HAL_ERROR;
	}
	return HAL_OK;
}

PCT2075_STATUS_t PCT2075_Init (I2C_HandleTypeDef *I2Cx, OS_F_Queue_t OS_F_Queue, OS_POL_t OS_POL, OS_COMP_INT_t OS_COMP_INT, uint16_t time){
	I2CHndl = I2Cx;
	
	if (PCT2075_WriteRegister(PCT2075_CONF, OS_F_Queue<<3 | OS_POL<<2 | OS_COMP_INT<<1, 1) != HAL_OK)
		return PCT2075_ERROR;
	
	iddleTime(time);
	
	return PCT2075_OK;	
}

uint16_t twoComplement(uint16_t value){
	value = (~value)+1;
	return value;
}

PCT2075_STATUS_t iddleTime(uint8_t value){
	if (value > 15) return PCT2075_INVALID;
	
	if (PCT2075_WriteRegister(PCT2075_Tiddle, value, 1) != HAL_OK)
		return PCT2075_ERROR;
	
	return PCT2075_OK;	
}

PCT2075_STATUS_t readTemperature (void){
	if (PCT2075_ReadRegister(PCT2075_TEMP, PCT2075.temp_data, 2))
	return PCT2075_ERROR;
	
	PCT2075.temperature = (PCT2075.temp_data[1]|(PCT2075.temp_data[0]<<5))*0.125;
	
	return PCT2075_OK;
}

PCT2075_STATUS_t setTemperatureBoundary (int16_t Tos, int16_t Thyst){
	PCT2075.Tos_data = Tos;
	PCT2075.Thyst_data = Thyst;
	
	if (Thyst > Tos)
		return PCT2075_INVALID;
	
	if (PCT2075_WriteRegister(PCT2075_Tos, Tos<<7, 2) != HAL_OK)
		return PCT2075_ERROR;
		
	if (PCT2075_WriteRegister(PCT2075_Thyst, Thyst<<7, 2) != HAL_OK)
		return PCT2075_ERROR;
	
	return PCT2075_OK;
}


PCT2075_STATUS_t PCT2075_shutDown(void){
	
	if (PCT2075_WriteRegister(PCT2075_CONF, SHUTDOWN_ON, 1) != HAL_OK)
		return PCT2075_ERROR;
	
	return PCT2075_OK;	
}

PCT2075_STATUS_t PCT2075_wakeUp(void){
	
	if (PCT2075_WriteRegister(PCT2075_CONF, SHUTDOWN_OFF, 1) != HAL_OK)
		return PCT2075_ERROR;
	
	return PCT2075_OK;	
}



