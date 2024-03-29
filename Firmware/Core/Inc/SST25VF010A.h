/*
 * SST25VF010A.h
 *
 *  Created on: Jan 11, 2024
 *      Author: rhett
 */

#ifndef INC_SST25VF010A_H_
#define INC_SST25VF010A_H_

#include "stm32f4xx_hal.h"

#define CS_MEM GPIO_PIN_3
#define MEM_LED GPIO_PIN_1

extern SPI_HandleTypeDef hspi1;


uint8_t MEM_Status_R(void); //Read status register

HAL_StatusTypeDef MEM_Status_W(uint8_t value); //Write to status register

HAL_StatusTypeDef  MEM_R(uint32_t addr,uint32_t bytes, uint8_t* data); //Read from memory starting at address addr and ending after bytes bytes are received

HAL_StatusTypeDef MEM_Byte_W(uint32_t addr,uint8_t data); //Byte program

HAL_StatusTypeDef MEM_AAI_W(uint32_t addr, uint32_t bytes,uint8_t* data); //Auto-increment program

HAL_StatusTypeDef W_EN(void); //Write enable

HAL_StatusTypeDef W_DI(void); //Write disable

uint8_t* ID_R(void); //Read ID (Device and manufacturer)

HAL_StatusTypeDef Chip_Erase(void); //Total erase to 0xFF

HAL_StatusTypeDef Page_Erase(uint32_t start_addr);

void delay(uint16_t delay);



#endif /* INC_SST25VF010A_H_ */
