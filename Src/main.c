/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

char RxData[11]={'0'};
uint8_t MyColor[3]={0};

#define UserLED_Toggle()	 HAL_GPIO_TogglePin(UserLED_GPIO_Port,UserLED_Pin)
#define UserLED_ON()  		 HAL_GPIO_WritePin(UserLED_GPIO_Port,UserLED_Pin,GPIO_PIN_SET)
#define UserLED_OFF() 		 HAL_GPIO_WritePin(UserLED_GPIO_Port,UserLED_Pin,GPIO_PIN_RESET)

#define numLEDs 9

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TIM_OC_InitTypeDef mysConfigOC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,100);

  return ch;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Din_1(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 26);
}
void Din_0(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 13);
}

void Send_8bits(uint8_t dat) 
{   
		uint8_t i; 
		Din_0();
		for(i=0;i<8;i++)   
		{ 
			if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
			{      
				Din_1();				
			} 		
			else 	//0 ,for "0",H:0.4us,L:	0.85us			
			{ 
			 Din_0();					
			}
		   dat=dat<<1; 
	 }
}
//G--R--B
//MSB first	
void Send_2811_24bits(uint8_t GData,uint8_t RData,uint8_t BData)
 {   
	Send_8bits(GData);  
	Send_8bits(RData);  
	Send_8bits(BData);
 } 
 

 void rst() 
{ 
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);	
	 //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
	 HAL_Delay (1);
}	

uint8_t AscillToNum(char mychar)
{
	uint8_t num=0;
	switch(mychar)
	{
		case '0':num=0;break;
		case '1':num=1;break;
		case '2':num=2;break;
		case '3':num=3;break;
		case '4':num=4;break;
		case '5':num=5;break;
		case '6':num=6;break;
		case '7':num=7;break;
		case '8':num=8;break;
		case '9':num=9;break;
	}
	return num;
}

typedef uint8_t  echoDeng_u8Type;

echoDeng_u8Type rBuffer[numLEDs]={0};
echoDeng_u8Type gBuffer[numLEDs]={0};
echoDeng_u8Type bBuffer[numLEDs]={0};

void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{ 
				echoDeng_u8Type i=0;
				for(i=0;i<numLEDs;i++)
				{
					rBuffer[i]=r;
					gBuffer[i]=g;
					bBuffer[i]=b;
				}
				for(i=0;i<numLEDs;i++)
				{							  
						Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
				}
 }
 void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
 {	 
				echoDeng_u8Type i=0;
				rBuffer[n]=r;
				gBuffer[n]=g;
				bBuffer[n]=b;
				for(i=0;i<numLEDs;i++)
				{							  
						Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
				}
 }
  void SetPixelColor(uint16_t n, uint32_t c)
 {	 
				echoDeng_u8Type i=0;
				rBuffer[n]=(uint8_t)(c>>16);
				gBuffer[n]=(uint8_t)(c>>8);
				bBuffer[n]=(uint8_t)c;
				for(i=0;i<numLEDs;i++)
				{							  
						Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
				}
 }
void PixelUpdate()
{
	rst();
}
uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}
uint32_t Wheel(uint8_t WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) 
	{
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
//²Êºç
void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for(j=0; j<256; j++) 
	{
    for(i=0; i<numLEDs; i++)
		{
      SetPixelColor(i, Wheel((i+j) & 255));
    }
		PixelUpdate();
    HAL_Delay (wait);
  }
}
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) 
{
  uint16_t i, j;

  for(j=0; j<256*5; j++) 
	{ // 5 cycles of all colors on wheel
    for(i=0; i< numLEDs; i++) 
	 {
     SetPixelColor(i, Wheel(((i * 256 / numLEDs) + j) & 255));
    }
	  PixelUpdate();
    HAL_Delay (wait);
  }
}
//Theatre-style crawling lights.ºôÎüµÆ
void theaterChase(uint32_t c, uint8_t wait) 
{
  for (int j=0; j<10; j++) 
	{  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) 
		{
      for (uint16_t i=0; i < numLEDs; i=i+3)
			{
        SetPixelColor(i+q, c);    //turn every third pixel on
      }
			PixelUpdate();
      HAL_Delay(wait);

      for (uint16_t i=0; i < numLEDs; i=i+3) 
			{
       SetPixelColor(i+q, 0);        //turn every third pixel off
      }
			PixelUpdate();
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) 
{
  for (int j=0; j < 256; j++) 
	{     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++)
		{
      for (uint16_t i=0; i < numLEDs; i=i+3) 
			{
        SetPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      PixelUpdate();

      HAL_Delay(wait);

      for (uint16_t i=0; i < numLEDs; i=i+3)
			{
        SetPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) 
{
	uint16_t i=0;
  for( i=0; i<numLEDs; i++) 
	{
    SetPixelColor(i, c);
    PixelUpdate();
    HAL_Delay(wait);
  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
		//HAL_FLASH_Unlock();	
	//printf("\n\r UART OK! \n\r");
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
			
//		for(i=0;i<11;i++)
//		{
//		 RxData[i]=0;
//		}
//		for(i=0;i<3;i++)
//		{
//		 MyColor[i]=0;
//		}
		//test code 
		    UserLED_Toggle();
		
//			  rst();
//				for(i=0;i<36;i++)
//				{							  
//						Send_2811_24bits(255,0,0);
//				}
//				HAL_Delay (500);
//				rst();
//				for(i=0;i<36;i++)
//				{
//						Send_2811_24bits(0,255,0);
//				}
//				HAL_Delay (500);
//				rst();
//				for(i=0;i<36;i++)
//				{
//						Send_2811_24bits(0,0,255);
//				}
//				HAL_Delay (500);
//setAllPixelColor(100,0,0);
//PixelUpdate();
//HAL_Delay (1000);
//setPixelColor(8, 0, 255, 0);
////setAllPixelColor(0,200,0);
//PixelUpdate();
//HAL_Delay (1000);
//setPixelColor(0, 0, 0, 255);
////setAllPixelColor(0,0,150);
//PixelUpdate();
//HAL_Delay (1000);

		// Some example procedures showing how to display to the pixels:
		colorWipe(Color(255, 0, 0), 50); // Red
		colorWipe(Color(0, 255, 0), 50); // Green
		colorWipe(Color(0, 0, 255), 50); // Blue
		// Send a theater pixel chase in...
		theaterChase(Color(127, 127, 127), 50); // White
		theaterChase(Color(127, 0, 0), 50); // Red
		theaterChase(Color(0, 0, 127), 50); // Blue

		rainbow(20);//²Êºç
		rainbowCycle(20);//Ñ­»·
		theaterChaseRainbow(50);//ºôÎüµÆ
		//test code over

//		if(HAL_UART_Receive(&huart2, RxData, 11, 10000)==HAL_OK)
//		{
//			  //printf("\n\r ½ÓÊÕµ½µÄÊý¾Ý:%s \n\r",RxData);
//				if(RxData[0]=='A' && RxData[1]=='A')
//				{
////					printf("\n\r begin explain data! \n\r");
////					for(i=0;i<11;i++)
////					{
////						printf("\n\r RxData[%d]=%d \n\r",i,RxData[i]);
////					}
//				  LED_G_Toggle();
//					MyColor[0]=AscillToNum(RxData[2])*100 + AscillToNum(RxData[3])*10 + AscillToNum(RxData[4]);
//					MyColor[1]=AscillToNum(RxData[5])*100 + AscillToNum(RxData[6])*10 + AscillToNum(RxData[7]);
//					MyColor[2]=AscillToNum(RxData[8])*100 + AscillToNum(RxData[9])*10 + AscillToNum(RxData[10]);
////					printf("\n\r RX GRB=%d %d %d \n\r",MyColor[0],MyColor[1],MyColor[2]);
//		      rst();
//				for(i=0;i<36;i++)
//				{
//						Send_2811_24bits(MyColor[0],MyColor[1],MyColor[2]);
//				}

//			 }			
//		}			
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 41;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_MultiProcessor_Init(&huart2, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UserLED_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserLED_Pin */
  GPIO_InitStruct.Pin = UserLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(UserLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
		UserLED_Toggle();
		HAL_Delay (50);
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
