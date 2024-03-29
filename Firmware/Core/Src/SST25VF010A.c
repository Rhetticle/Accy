/*
 * SST25VF010A.c
 *
 *  Created on: Jan 11, 2024
 *      Author: rhett
 */

#include "SST25VF010A.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim1;

void delay(uint16_t delay){

	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<delay);

}

uint8_t MEM_Status_R(void){

	uint8_t data[2]={0x05,0x00};
	uint8_t stat_reg[2];


	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

	HAL_SPI_TransmitReceive(&hspi1, data, stat_reg, 2, 100);

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);

	return(stat_reg[1]);


}

HAL_StatusTypeDef W_EN(void){
	uint8_t write_en=0x06;
	uint8_t write_en_rec;

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

	if(HAL_SPI_TransmitReceive(&hspi1, &write_en, &write_en_rec, 1, 100)!=HAL_OK){
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		return(HAL_ERROR);
	}

	else{
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		HAL_Delay(1);
		if(MEM_Status_R()==0x00){
			return(HAL_OK);
		}

		else{
			return(HAL_ERROR);
		}
	}


}

HAL_StatusTypeDef W_DI(void){
	uint8_t write_dis=0x04;
	uint8_t write_dis_rec;

		HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

		if(HAL_SPI_TransmitReceive(&hspi1, &write_dis, &write_dis_rec, 1, 100)!=HAL_OK){
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			return(HAL_ERROR);
		}

		else{
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			HAL_Delay(1);
			if(MEM_Status_R()==0x00){

				return(HAL_OK);
			}

			else{
				return(HAL_ERROR);
			}
		}
}

HAL_StatusTypeDef MEM_Status_W(uint8_t value){

	uint8_t EWSR=0x50;
	uint8_t EWSR_rec; //ignore
	uint8_t WRSR[2]={0x01,value};
	uint8_t WRSR_rec[2]; //ignore

	while(MEM_Status_R()!=value){


		HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

		HAL_SPI_TransmitReceive(&hspi1, &EWSR, &EWSR_rec, 1, 100);

		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

		HAL_SPI_TransmitReceive(&hspi1, WRSR, WRSR_rec, 2, 100);

		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);


		HAL_Delay(100);
	}


	if(MEM_Status_R()!=value){
		return(HAL_ERROR);
	}

	else{
		return(HAL_OK);
	}

}

HAL_StatusTypeDef MEM_R(uint32_t addr,uint32_t bytes,uint8_t* data){

	uint8_t rec_data[bytes+4];
	uint8_t transaction[bytes+4];
	memset(transaction,0,bytes+4);
	transaction[0]=0x03;
	transaction[1]=addr>>16;
	transaction[2]=addr>>8;
	transaction[3]=addr;



	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

	if(HAL_SPI_TransmitReceive(&hspi1, transaction, rec_data, bytes+4, 100)!=HAL_OK){
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		return(HAL_ERROR);

	}

	else{
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		for(int i=0;i<bytes;i++){
			data[i]=rec_data[i+4];
		}
		return(HAL_OK);
	}



}

HAL_StatusTypeDef MEM_Byte_W(uint32_t addr,uint8_t data){

	uint8_t transaction[5]={0x02,addr>>16,addr>>8,addr,data};
	uint8_t dummy[5];

	while(MEM_Status_R()!=0x00){

	}

	W_EN();

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

	if(HAL_SPI_TransmitReceive(&hspi1, transaction, dummy, 5, 100)!=HAL_OK){
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		return(HAL_ERROR);
	}

	else{
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		return(HAL_OK);
	}
}

HAL_StatusTypeDef MEM_AAI_W(uint32_t addr, uint32_t bytes,uint8_t* data){

	uint8_t init_transac[5]={0xAF,addr>>16,addr>>8,addr,data[0]};

	while(MEM_Status_R()!=0x00){

	}

	W_EN();

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

	HAL_SPI_Transmit(&hspi1, init_transac, 5, 100);

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
	delay(100);


	for(int i=1;i<bytes;i++){
		uint8_t data_write[2]={0xAF,data[i]};
		W_EN();
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);
		HAL_SPI_Transmit(&hspi1, data_write, 2, 100);
		HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
		delay(100);

	}

	W_DI();

	if(MEM_Status_R()==0x00){
		return(HAL_OK);
	}

	else{
		return(HAL_ERROR);
	}
}

uint8_t* ID_R(void){
	uint8_t transaction_data[6]={0x90,0x00,0x00,0x01,0x00,0x00}; //0x90 (or 0xAB) is read-ID instruction, then 24 bits for address (1Mbit). Final two bytes are dummy bytes.
	uint8_t dev_data[6];
	char MEM_ERR[]="Error initialising external memory";
	char MEM_CPLT[100];

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);

	if(HAL_SPI_TransmitReceive(&hspi1, transaction_data, dev_data, 6, 100)!=HAL_OK){
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			//CDC_Transmit_FS((uint8_t*)MEM_ERR, strlen(MEM_ERR));
			HAL_GPIO_WritePin(GPIOB, MEM_LED, 1);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, MEM_LED, 0);
			HAL_Delay(100);
			return(HAL_ERROR);

	}

	else{
		if(dev_data[4]==0x49 && dev_data[5]==0xBF){
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			sprintf(MEM_CPLT,"External memory connection successful. The device ID is 0x%x. The manufacturer ID is 0x%x\n\r",dev_data[4],dev_data[5]);
			//CDC_Transmit_FS((uint8_t*)MEM_CPLT, strlen(MEM_CPLT));
			HAL_GPIO_WritePin(GPIOB, MEM_LED, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOB, MEM_LED, 0);
			return(HAL_OK);
		}

		else{
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			//CDC_Transmit_FS((uint8_t*)MEM_ERR, strlen(MEM_ERR));
			HAL_GPIO_WritePin(GPIOB, MEM_LED, 1);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, MEM_LED, 0);
			HAL_Delay(100);
			return(HAL_ERROR);

		}
	}
}

HAL_StatusTypeDef Chip_Erase(void){
	uint8_t erase=0x60;
	uint8_t erase_rec; //ignore

	W_EN();

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);



	if(HAL_SPI_TransmitReceive(&hspi1, &erase, &erase_rec, 1, 100)!=HAL_OK){
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			return(HAL_ERROR);
	}

	else{
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			return(HAL_OK);

	}
}

HAL_StatusTypeDef Page_Erase(uint32_t start_addr){
	uint8_t transaction[4]={0x52,start_addr>>16,start_addr>>8,start_addr};

	while(MEM_Status_R()!=0x00){

		}

	W_EN();

	HAL_GPIO_WritePin(GPIOA, CS_MEM, 0);


	if(HAL_SPI_Transmit(&hspi1, transaction, 4, 100)!=HAL_OK){
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			return(HAL_ERROR);
	}

	else{
			HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
			return(HAL_OK);

	}

}
