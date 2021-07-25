/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "File_Handling.h"
#include <stdbool.h>
#include "fonts.h"
#include "ssd1306.h"
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
HAL_StatusTypeDef statue =0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern ApplicationTypeDef Appli_state ;
extern char USBHPath[4];   /* USBH logical drive path */
extern FATFS USBHFatFS;    /* File system object for USBH logical drive */
extern FIL USBHFile;       /* File object for USBH */
uint8_t Buffer[8096];
int IndexFile =  0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(unsigned int SampleRate);
static void MX_I2C1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
typedef struct WAV_Formate
{
	char FileMark[5] ;
	char FileType[5];
	char * FileName;
	uint32_t FileLength ;
	uint16_t NumOfChannels ;
	uint32_t SampleRate ;
	uint16_t BitPerSample ;
}WAV_Formate;
uint8_t data[100];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t  i ;

DIR dir;

char * FileArray ;
WAV_Formate * file = NULL ;

volatile enum _PlayState {State_Play ,State_Puse , State_Stop , State_Next,State_Previous,State_Idel}PlayState ;

volatile enum _I2SDMAState {I2SDMA_Ready=1 ,I2SDMA_Running }I2SDMAState ;

/*
 *this function used to get all .wav files in the directory
 *param pPath the specific path
 *AllDirFiles pointer to pointer to WAV_Formate struct
 *return -1 if the function fail or return 1  on success
 */

int ReadDIRWavFiles(char * pPath, WAV_Formate ** AllDirFiles);



/* this function used to start the I2S operation */
void StartRunningWavFile(uint8_t  fileIndex );

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PlayState = State_Next ;
	I2SDMAState = I2SDMA_Running ;
	uint8_t CurrentIndex = 0 ;
	uint32_t CurrentByte =0 ;
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
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_I2S2_Init(8000);
  MX_USB_HOST_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */



  SSD1306_Init();  // initialise


  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("Welcome :)", &Font_16x26, 1);
  SSD1306_UpdateScreen(); //display
  SSD1306_ScrollRight(0,26);
  HAL_Delay (2000);
  SSD1306_Stopscroll();
  SSD1306_UpdateScreen(); //display

  while(Appli_state != APPLICATION_READY)
  {
	    MX_USB_HOST_Process();
	    if(Appli_state == APPLICATION_READY)
	    {

	    	ReadDIRWavFiles(USBHPath ,&file ); /* collect all file in give path */

	    }
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */


    if(Appli_state == APPLICATION_READY)
    {

        MX_USB_HOST_Process();


    	switch(PlayState)
    	{

    	case State_Next:
        		StartRunningWavFile(CurrentIndex);

          		CurrentIndex ++;
            	if((CurrentIndex >  IndexFile)||(CurrentIndex == 0))
            		CurrentIndex=0;

    	break ;
    	case State_Previous:
    		CurrentIndex --;
    		if((CurrentIndex >  IndexFile)||(CurrentIndex == 0))
    			CurrentIndex=1;

    		StartRunningWavFile(CurrentIndex-1);


    	break ;
    	case State_Puse:

    		SSD1306_GotoXY (0,26);
    		SSD1306_Puts ("Pause", &Font_11x18, 1);
    		SSD1306_UpdateScreen(); //display
    		HAL_I2S_DMAPause(&hi2s2);
    		while(PlayState == State_Puse);

        break ;
    	case State_Play:

    		SSD1306_GotoXY (0,26);
    		SSD1306_Puts ("Run  ", &Font_11x18, 1);
    		SSD1306_UpdateScreen(); //display
    		HAL_I2S_DMAResume(&hi2s2);
    	 	PlayState = State_Idel;

    	break ;
    	case State_Idel:
    		i =0 ;
    		while(PlayState == State_Idel)
    		{

    			if(I2SDMAState == I2SDMA_Running)
    			{
    			f_read(&USBHFile, Buffer , sizeof(Buffer), &i);
    			if(i <=0 )
    			{
    				PlayState = State_Next ;
    				break  ;
    			}
    				I2SDMAState = 0 ;
    			}
    			else if(I2SDMAState == I2SDMA_Ready)
    			{
        		HAL_I2S_Transmit_DMA(&hi2s2, Buffer, sizeof(Buffer)/2);
        		I2SDMAState = I2SDMA_Running ;
    			}
    			else
    			{

    			}
    	}


    	break ;

    	}

    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(unsigned int SampleRate)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = SampleRate;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Mute_GPIO_Port, Mute_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Next_Pin Previous_Pin Pause_Pin MuteExt_Pin */
  GPIO_InitStruct.Pin = Next_Pin|Previous_Pin|Pause_Pin|MuteExt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Mute_Pin */
  GPIO_InitStruct.Pin = Mute_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mute_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(hi2s);
	I2SDMAState = I2SDMA_Ready ;
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2S_TxCpltCallback could be implemented in the user file
   */
}


/*
 * brief : this function used to get WAV files from the given directory
 * pDIR input parameter pointer to  director
 * Files output parameter hold the files
 * return 1 if true or -1 on failed
 */
int ReadDIRWavFiles(char * pPath, WAV_Formate ** AllDirFiles)
{
	FRESULT res ;
	FILINFO fileinfo;
	char Buffer[50] = {0};
	uint32_t ReadNumOfButes = 0 ;
	/*
	 * _MAX_LFN maximum file length define by file system
	 * first we only allocate space for 10 files
	 */
	uint16_t Len = 60 ,FileNameLen=0;
	WAV_Formate * FileStruct =(WAV_Formate *)malloc(sizeof(WAV_Formate) * Len); /* first create 10 elements */




	/* dir is defined as global var for all file */
	res = f_opendir(&dir, pPath);
	if(res != FR_OK)
		return -1 ;




	do{

		if(IndexFile+1 == Len )
		{
			Len *= 2 ;
			if(realloc(FileStruct ,sizeof(WAV_Formate) * Len) == NULL)
				return -1 ;
		}
		res = f_readdir(&dir, &fileinfo);
		FileNameLen = strlen(fileinfo.fname);

		if((strlen(fileinfo.fname) == 0))
		{
    	  	f_close(&USBHFile);

			break;  /* break condition */

		}

		if((fileinfo.fname[FileNameLen-1] == 'v')&&
			(fileinfo.fname[FileNameLen-2] == 'a')&&
			(fileinfo.fname[FileNameLen-3] == 'w'))        /* only we interested with WAV extension files */
		{
			if(f_open(&USBHFile, fileinfo.fname , FA_READ) != FR_OK)
				return -1 ;

			if(f_read(&USBHFile,Buffer ,50 ,&ReadNumOfButes) != FR_OK)
				return -1 ;

    	  	memcpy( FileStruct[IndexFile].FileMark,Buffer , 4);
    	  	FileStruct[IndexFile].FileMark[4] = 0;
    	  	memcpy(FileStruct[IndexFile].FileType,Buffer+8,3);
    	  	FileStruct[IndexFile].FileType[3] = 0;
    	  	FileStruct[IndexFile].SampleRate = *(uint32_t*)(Buffer+24);
    	  	FileStruct[IndexFile].BitPerSample = *(uint16_t *)(Buffer+34);
    	  	FileStruct[IndexFile].NumOfChannels = *(uint16_t *)(Buffer+22);
    	  	FileStruct[IndexFile].FileLength = *(uint32_t *)(Buffer+4);

    	  	FileStruct[IndexFile].FileName =    (char *)malloc(FileNameLen); ;

    	  	strcpy(FileStruct[IndexFile].FileName, fileinfo.fname );
    	  	FileStruct[IndexFile].FileName[FileNameLen]=0;

    	  	if(f_close(&USBHFile) != FR_OK)
    	  		return -1 ;

    	  	IndexFile++;
		}


	}while( res == FR_OK);

	*AllDirFiles = FileStruct ;
	return 1 ;   /* after successful operation return 1 to caller */

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	if(GPIO_Pin == MuteExt_Pin)
	{
		if(HAL_GPIO_ReadPin(GPIOG, Mute_Pin))
		{

			SSD1306_GotoXY (80,26);
			SSD1306_Puts ("      ", &Font_11x18, 1);
			SSD1306_UpdateScreen(); //display
			HAL_GPIO_WritePin(GPIOG, Mute_Pin, 0);
		}else{

			SSD1306_GotoXY (80,26);
			SSD1306_Puts ("Mute   ", &Font_11x18, 1);
			SSD1306_UpdateScreen(); //display
			HAL_GPIO_WritePin(GPIOG, Mute_Pin, 1);
		}


	}

	if(GPIO_Pin == Next_Pin)
	{

		PlayState = State_Next ;

	}

	if(GPIO_Pin == Previous_Pin)
	{

		PlayState = State_Previous ;

	}

	if(GPIO_Pin == Pause_Pin)
	{



		if(PlayState == State_Puse )
		PlayState = State_Play ;
		else
		PlayState = State_Puse ;



	}

	HAL_Delay(4);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}


void StartRunningWavFile(uint8_t  fileIndex )
{
	HAL_I2S_DMAStop(&hi2s2);
	f_close(&USBHFile);
	int i = 0;

	if(fileIndex == IndexFile)
		fileIndex = 0 ;


  	if(f_open(&USBHFile, file[fileIndex].FileName, FA_READ) != FR_OK)
  		Error_Handler();


    MX_I2S2_Init(file[fileIndex].SampleRate);

	SSD1306_Clear();
	SSD1306_GotoXY (0,0);
	SSD1306_Puts (file[fileIndex].FileName, &Font_7x10, 1);

	SSD1306_GotoXY (0,26);
	SSD1306_Puts ("Run   ", &Font_11x18, 1);
	SSD1306_UpdateScreen(); //display
  	PlayState = State_Idel;

	f_read(&USBHFile, Buffer , sizeof(Buffer), &i);
	HAL_I2S_Transmit_DMA(&hi2s2, Buffer, sizeof(Buffer)/2);

}




/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
