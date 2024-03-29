/*
 * LIS3DHTR.h
 *
 *  Created on: Jan 11, 2024
 *      Author: rhett
 */

#ifndef INC_LIS3DHTR_H_
#define INC_LIS3DHTR_H_

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define LIS3DH_I2C_ADDR 0b0011000
#define I2C_LED GPIO_PIN_5

HAL_StatusTypeDef ACC_INIT(void); //Initialise accelerometer to 400Hz, all axes, set range to +/- 16g and check whoami register

HAL_StatusTypeDef ACC_R(float* result); //Read accelerometer data, values stored in result and given in g's

#endif /* INC_LIS3DHTR_H_ */
