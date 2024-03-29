/*
 * LIS3DHTR.c
 *
 *  Created on: Jan 11, 2024
 *      Author: rhett
 */

#include "LIS3DHTR.h"

HAL_StatusTypeDef ACC_INIT(void){
	uint8_t WHOAMI_REG; //Expect 0x33
	uint8_t ctr_reg1=0x77; // 400Hz all axes enabled, low power mode disabled
	uint8_t ctr_reg4=0x30; // Set range to +/- 16g
	char I2C_ERR[]="Error initialising accelerometer";
	char CONFIG_ERR[]="Error configuring control register";
	char CONFIG_CPLT[]="Control registers configured successfully";
	char I2C_CPLT[100];

	if(HAL_I2C_Mem_Read(&hi2c1,LIS3DH_I2C_ADDR<<1 , 0x0F, 1, &WHOAMI_REG, 1, 100)!=HAL_OK){
				//CDC_Transmit_FS((uint8_t*)I2C_ERR, strlen(I2C_ERR));
				HAL_GPIO_WritePin(GPIOB, I2C_LED, 1);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, I2C_LED, 0);
				HAL_Delay(100);
				return(HAL_ERROR);
		}

		else{
				sprintf(I2C_CPLT,"Accelerometer connection successful. The value of the who am I register is 0x%x\n\r",WHOAMI_REG);
				//CDC_Transmit_FS((uint8_t*)I2C_CPLT, strlen(I2C_CPLT));
				HAL_GPIO_WritePin(GPIOB, I2C_LED, 1);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOB, I2C_LED, 0);

			if(HAL_I2C_Mem_Write(&hi2c1, LIS3DH_I2C_ADDR<<1, 0x20, 1, &ctr_reg1, 1, 100)!=HAL_OK || HAL_I2C_Mem_Write(&hi2c1, LIS3DH_I2C_ADDR<<1, 0x23, 1, &ctr_reg4, 1, 100)!=HAL_OK){
						//CDC_Transmit_FS(CONFIG_ERR, strlen(CONFIG_ERR));
						HAL_GPIO_WritePin(GPIOB, I2C_LED, 1);
						HAL_Delay(100);
						HAL_GPIO_WritePin(GPIOB, I2C_LED, 0);
						HAL_Delay(100);
						return(HAL_ERROR);
					}

			else{
					//CDC_Transmit_FS(CONFIG_CPLT, strlen(CONFIG_CPLT));
					return(HAL_OK);
				}

			}
}

HAL_StatusTypeDef ACC_R(float* result){
	uint8_t X_H,X_L,Y_H,Y_L,Z_H,Z_L;
	int16_t X_INT,Y_INT,Z_INT;
	uint8_t x_stat,y_stat,z_stat;
	float X_ACC,Y_ACC,Z_ACC;

	char mess[100];

	x_stat=HAL_I2C_Mem_Read(&hi2c1,  LIS3DH_I2C_ADDR<<1,0x28, 1, &X_L, 1, 100); //X_L
	HAL_I2C_Mem_Read(&hi2c1,  LIS3DH_I2C_ADDR<<1,0x29, 1, &X_H, 1, 100); //X_H
	y_stat=HAL_I2C_Mem_Read(&hi2c1,  LIS3DH_I2C_ADDR<<1,0x2A, 1, &Y_L, 1, 100); //Y_L
	HAL_I2C_Mem_Read(&hi2c1,  LIS3DH_I2C_ADDR<<1,0x2B, 1, &Y_H, 1, 100); //Y_H
	z_stat=HAL_I2C_Mem_Read(&hi2c1,  LIS3DH_I2C_ADDR<<1,0x2C, 1, &Z_L, 1, 100); //Z_L
	HAL_I2C_Mem_Read(&hi2c1,  LIS3DH_I2C_ADDR<<1,0x2D, 1, &Z_H, 1, 100); //Z_H

	if(x_stat!=HAL_OK || y_stat!=HAL_OK || z_stat!=HAL_OK){
			return(HAL_ERROR);
		}

	else{
			X_INT=X_H;
			X_INT<<=8;
			X_INT|=X_L;
			X_INT>>=6;

			Y_INT=Y_H;
			Y_INT<<=8;
			Y_INT|=Y_L;
			Y_INT>>=6;

			Z_INT=Z_H;
			Z_INT<<=8;
			Z_INT|=Z_L;
			Z_INT>>=6;

			X_ACC=(float)X_INT*0.048;
			Y_ACC=(float)Y_INT*0.048;
			Z_ACC=(float)Z_INT*0.048;

			result[0]=X_ACC;
			result[1]=Y_ACC;
			result[2]=Z_ACC;

			return(HAL_OK);
		}
}
