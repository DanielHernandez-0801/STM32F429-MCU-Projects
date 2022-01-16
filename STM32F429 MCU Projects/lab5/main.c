/****** 



 1. both OD mode and PP mode can drive the motor! However, some pin can not output  high in OD mode!!! 
   (maybe because those pins have other alternate functions)). 

 2. the signals do not need to be inverted before feeded in H-bridge! 
*/




#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


// Motor Pins
#define GPIO_PORT GPIOC	//If this changes, change clock enable in config function
#define A1 GPIO_PIN_3
#define A2 GPIO_PIN_2
#define B1 GPIO_PIN_5
#define B2 GPIO_PIN_4

TIM_HandleTypeDef    Tim3_Handle;
uint16_t Tim3_PrescalerValue;

uint8_t step = 1;
uint8_t cw = 1;
uint8_t full_step = 1;
uint16_t rpm = 5;

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);


static void SystemClock_Config(void);
static void TIM3_Config(void);
static void GPIO_Config(void);
static void Error_Handler(void);

int main(void){
	
		/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
		HAL_Init();
		
	
		 /* Configure the system clock to 180 MHz */
		SystemClock_Config();
		
		HAL_InitTick(0x0000); // set systick's priority to the highest.
	
		BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);


		BSP_LCD_Init();
		//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
		BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
															// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
		//BSP_LCD_SelectLayer(uint32_t LayerIndex);
		BSP_LCD_SelectLayer(0);
		//BSP_LCD_SetLayerVisible(0, ENABLE); //do not need this line.
		BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
		BSP_LCD_DisplayOn();
	 
		BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
	
		LCD_DisplayString(2, 3, (uint8_t *)"Lab");
		LCD_DisplayInt(2, 8, 5);
		
		// Display motor RPM
		LCD_DisplayString(12, 2, (uint8_t *)"RPM:  ");
		LCD_DisplayInt(12, 6, rpm);
		
		BSP_LED_Init(LED3);
		BSP_LED_Init(LED4);
		
		// Configure timer
		TIM3_Config();
		
		// Configure GPIO
		GPIO_Config();
		
		
		while(1) {	

			
		} // end of while loop
	
}  //end of main


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

// Initialize stepper timer
static void TIM3_Config(void)
{
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3;
   

	// Calculate period from RPM starting point
  Tim3_Handle.Init.Period = (int)(10000*60/(rpm*45));	
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
	HAL_TIM_Base_MspInit(&Tim3_Handle);
	
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

// Initialize GPIO pins
static void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	
	// Setup Output Pins 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	GPIO_InitStruct.Pin = A1|A2|B1|B2;
	HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIO_PORT, A2|B1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT, A1|B2, GPIO_PIN_RESET);
	
	
	// Setup push buttons
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull =GPIO_PULLUP;

	// Button 1 (PC1)
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);

  /* Enable and set EXTI Line1 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	
	// Button 2 (PD2)
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);

  /* Enable and set EXTI Line2 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
	// Button 3 (PE3)
	GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);

  /* Enable and set EXTI Line2 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}
	


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		// User button
		if(GPIO_Pin == KEY_BUTTON_PIN)
		{
				full_step = full_step ^ 0x1; //Toggle stepping modes
			//Double the speed of the clock if half-stepping
			__HAL_TIM_SET_PRESCALER(&Tim3_Handle, Tim3_PrescalerValue/(2-full_step)); 
			
		}
		
		// External button 1 (PC1)
		if(GPIO_Pin == GPIO_PIN_1)
		{
			cw = cw ^ 0x1;	//Toggle stepping direction
		}

		// External button 2 (PD2)
		if(GPIO_Pin == GPIO_PIN_2)
		{
			// Increase motor speed
			if(rpm < 20){
				rpm++;
				__HAL_TIM_SET_AUTORELOAD(&Tim3_Handle, (int)(10000*60/(rpm*45)));
				
				LCD_DisplayString(12, 2, (uint8_t *)"RPM:  ");
				LCD_DisplayInt(12, 6, rpm);
			}
		}
		
		// External button 3 (PE3)
		if(GPIO_Pin == GPIO_PIN_3)
		{	
			// Decrease motor speed
			if(rpm > 1){
				rpm--;
				__HAL_TIM_SET_AUTORELOAD(&Tim3_Handle, (int)(10000*60/(rpm*45)));
				
				LCD_DisplayString(12, 2, (uint8_t *)"RPM:  ");
				LCD_DisplayInt(12, 6, rpm);
			}
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   
{
	// TIM3 inturrupt handler
	if ((*htim).Instance==TIM3){
		switch(step)
		{
			case 1:
				// Step 1
				HAL_GPIO_WritePin(GPIO_PORT, A1|A2|B2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, B1, GPIO_PIN_SET);
				break;
			case 2:
				// Step 2
				HAL_GPIO_WritePin(GPIO_PORT, A1|B2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, A2|B1, GPIO_PIN_SET);
				break;
			case 3:
				// Step 3
				HAL_GPIO_WritePin(GPIO_PORT, A1|B1|B2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, A2, GPIO_PIN_SET);
				break;
			case 4:
				// Step 4
				HAL_GPIO_WritePin(GPIO_PORT, A1|B1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, A2|B2, GPIO_PIN_SET);
				break;
			case 5:
				// Step 5
				HAL_GPIO_WritePin(GPIO_PORT, A1|A2|B1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, B2, GPIO_PIN_SET);
				break;
			case 6:
				// Step 6
				HAL_GPIO_WritePin(GPIO_PORT, A2|B1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, A1|B2, GPIO_PIN_SET);
				break;
			case 7:
				// Step 7
				HAL_GPIO_WritePin(GPIO_PORT, A2|B1|B2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, A1, GPIO_PIN_SET);
				break;
			case 8:
				// Step 8
				HAL_GPIO_WritePin(GPIO_PORT, A2|B2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_PORT, A1|B1, GPIO_PIN_SET);
				break;
		}
		
		// Go to next step in sequence
		if(cw){
			// clockwise rotation
			step = step%8;
			step++;
			if(full_step){
				step += step%2;
			}
		}
		else{
			// counter clockwise rotation
			step--;
			if(full_step){
				step -= step%2;
			}
			if(step<=0){
				step = 8;
			}
		}
		LCD_DisplayInt(8, 0, step);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
			
	__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
}
 
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */



