/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SST25VF010A.h"
#include "LIS3DHTR.h"
#include "fatfs_sd.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIS3DH_I2C_ADDR 0b0011000 //(SA0 connected to GND)
#define I2C_LED GPIO_PIN_5
#define MEM_LED GPIO_PIN_1
#define SD_LED GPIO_PIN_2
#define CS_MEM GPIO_PIN_3
#define CS_SD GPIO_PIN_4
#define DETECT GPIO_PIN_0

#define X_PAGE 0x000000 //(32kB) Use pre-determined blocks from datasheet in order to be able to use block_erase instruction
#define Y_PAGE 0x008000 //(32kB)
#define Z_PAGE 0x010000 //(32kB)
#define t_PAGE 0x018000 //(32kB)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t buffer[64];
uint8_t rec_cplt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS fs;
FIL fil;
FRESULT fres;
uint8_t bw;
float t_last=0;
uint8_t card_pres=0;

void Array_test(void){

	uint32_t value=0x00FF00CC;

	uint8_t var8bit[4];

	var8bit[0]=value;
	var8bit[1]=value>>8;
	var8bit[2]=value>>16;
	var8bit[3]=value>>24;

	CDC_Transmit_FS(&var8bit[0], 1);
	HAL_Delay(200);
	CDC_Transmit_FS(&var8bit[1], 1);
	HAL_Delay(200);
	CDC_Transmit_FS(&var8bit[2], 1);
	HAL_Delay(200);
	CDC_Transmit_FS(&var8bit[3], 1);
	HAL_Delay(200);

}

void float_8bit(float value, uint8_t* data){
	int16_t a=value*100;
	data[0]=a;
	data[1]=a>>8;

}

float bit8_to_float(uint8_t* data){
	int16_t a = ((uint16_t)data[1]<<8)|data[0];
	return(((float)a)/100);
}

void write_seq(uint32_t x_addr,uint32_t y_addr, uint32_t z_addr,uint32_t t_addr){

	float ACC[3];
	float t;
	uint8_t x_acc[2];
	uint8_t y_acc[2];
	uint8_t z_acc[2];
	uint8_t t_8bit[2];
	char mess[100];

	ACC_R(ACC);
	t_last+=((float)__HAL_TIM_GET_COUNTER(&htim2))*(0.000001);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	float_8bit(ACC[0], x_acc);
	float_8bit(ACC[1], y_acc);
	float_8bit(ACC[2], z_acc);
	float_8bit(t_last, t_8bit);
	MEM_AAI_W(x_addr, 2, x_acc);
	MEM_AAI_W(y_addr, 2, y_acc);
	MEM_AAI_W(z_addr, 2, z_acc);
	MEM_AAI_W(t_addr, 2, t_8bit);
	sprintf(mess,"Writing %.2f, %.2f,  %.2f, %.2f at address 0x%X\r\n",ACC[0],ACC[1],ACC[2],t_last,x_addr);
	//CDC_Transmit_FS(mess, strlen(mess));
}

/*Function to read acceleration data from flash memory
 *
 *Parameters:
 *	x_addr(uint32_t):24 bit address from within X_PAGE to begin reading from
 *	y_addr(uint32_t):24 bit address from within Y_PAGE to begin reading from
 *	z_addr(uint32_t):24 bit address from within Z_PAGE to begin reading from
 *	t (float): Time in seconds since beginning read
 *
 * */

uint8_t read_seq(uint32_t x_addr,uint32_t y_addr, uint32_t z_addr,uint32_t t_addr){

	char mess[100];
	float x_f,y_f,z_f,t;
	uint8_t x_rec[2];
	uint8_t y_rec[2];
	uint8_t z_rec[2];
	uint8_t t_rec[2];

	MEM_R(x_addr, 2, x_rec);
	HAL_Delay(1);
	MEM_R(y_addr, 2, y_rec);
	HAL_Delay(1);
	MEM_R(z_addr, 2, z_rec);
	HAL_Delay(1);
	MEM_R(t_addr, 2, t_rec);
	HAL_Delay(1);
	x_f=bit8_to_float(x_rec);
	y_f=bit8_to_float(y_rec);
	z_f=bit8_to_float(z_rec);
	t=bit8_to_float(t_rec);
	if((int)(t*100)==-1){
		return(1);
	}
	sprintf(mess,"%.2f, %.2f, %.2f, %.2f \r\n",t,x_f,y_f,z_f);
	f_write(&fil, mess, strlen(mess), &bw);
	//CDC_Transmit_FS(mess, strlen(mess));
	return(0);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
  HAL_TIM_Base_Start(&htim1); //used for delay() function
  HAL_TIM_Base_Start(&htim2); //used for timing acceleration measurements
  HAL_Delay(3000);
  HAL_GPIO_WritePin(GPIOA, CS_SD, 1);
  HAL_GPIO_WritePin(GPIOA, CS_MEM, 1);
  while(ACC_INIT()!=HAL_OK){
	  HAL_GPIO_WritePin(GPIOB, I2C_LED, 1);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, I2C_LED, 0);
	  HAL_Delay(100);
  }

  HAL_GPIO_WritePin(GPIOB, I2C_LED, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, I2C_LED, 0);
  HAL_Delay(1000);

  while(MEM_Status_W(0x00)!=HAL_OK){
	  HAL_GPIO_WritePin(GPIOB, MEM_LED, 1);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, MEM_LED, 0);
	  HAL_Delay(100);
  }

  HAL_GPIO_WritePin(GPIOB, MEM_LED, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, MEM_LED, 0);
  HAL_Delay(1000);

  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)!=1){
	 Page_Erase(X_PAGE); //25ms max
	 HAL_Delay(100);
	 Page_Erase(Y_PAGE); //25ms max
	 HAL_Delay(100);
	 Page_Erase(Z_PAGE); //25ms max
	 HAL_Delay(100);
	 Page_Erase(t_PAGE); //25ms max
	 HAL_Delay(100);
  }

  int i=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_Base_Start(&htim2);
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==1){ //SD card can interrupt to retrieve whatever data has been written so far
		  card_pres=1;
		  HAL_Delay(1000);
		  int j=0;
		  float t=0;
		  f_mount(&fs, "", 0);
		  f_unlink("Acceleration.csv");
		  f_open(&fil, "Acceleration.csv", FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
		  f_puts("t,x,y,z \r\n", &fil);
			  while(read_seq(X_PAGE+j, Y_PAGE+j, Z_PAGE+j, t_PAGE+j)!=1){
				  j+=2;
			  }
			  f_close(&fil);
			  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==1){ //Hang until sd card is removed
				  HAL_GPIO_WritePin(GPIOB, SD_LED, 1);
			  }
			  HAL_GPIO_WritePin(GPIOB, SD_LED, 0);
			  card_pres=0;

	  }

	  else{
		  if(X_PAGE+i==0x1000){
			  HAL_GPIO_WritePin(GPIOB, MEM_LED, 1);
		  }
		  else{
			  write_seq(X_PAGE+i, Y_PAGE+i, Z_PAGE+i,t_PAGE+i);
			  i+=2;
		  }

	  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, uC_ON_Pin|CS_MEM_Pin|CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_DETECT_Pin|MEM_CPLT_Pin|SD_CPLT_Pin|I2C_CPLT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : uC_ON_Pin CS_MEM_Pin CS_SD_Pin */
  GPIO_InitStruct.Pin = uC_ON_Pin|CS_MEM_Pin|CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_DETECT_Pin MEM_CPLT_Pin SD_CPLT_Pin I2C_CPLT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin|MEM_CPLT_Pin|SD_CPLT_Pin|I2C_CPLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
