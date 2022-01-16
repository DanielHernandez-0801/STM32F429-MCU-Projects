/****** 
1.The Fist Extern button (named extBtn1)  connected to PC1, ExtBtn1_PC1_Config  //
		2014: canot use pin PB1, for 429i-DISCO ,pb1 is used by LCD. if use this pin, always interrupt by itself
					can not use pin PA1, since it is used by gyro. if use this pin, never interrupt.
					pd1----WILL ACT AS PC13, To trigger the RTC timestamp event
					....ONLY PC1 CAN BE USED TO FIRE EXTI1 !!!!
2. the Second external button (extBtn2)
		2014: 
		PA2: NOT OK. (USED BY LCD??)
		PB2: ????.
		PC2: ok, BUT sometimes (every 5 times around), press pc2 will trigger exti1, which is configured to use PC1. (is it because of using internal pull up pin config?)
		      however, press PC1 does not affect exti 2. sometimes press PC2 will also affect time stamp (PC13)
		PD2: OK,     
		PE2:  OK  (PE3, PE4 PE5 , seems has no other AF function, according to the table in manual for discovery board)
		PF2: NOT OK. (although PF2 is used by SDRAM, it affects LCD. press it, LCD will flick and displayed chars change to garbage)
		PG2: OK
		
*/

#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


// ADC value store
__IO uint16_t ADC3ConvertedValue=0;

__IO uint16_t CCR_Val = 0;

uint8_t readButton = 0;
uint8_t fanStatus = 1;
int holdCount = 0;
int mult;

 volatile double currentTemp = 23.5;
 volatile double  setPoint=23.5;  //NOTE: if declare as float, when +0.5, the compile will give warning:
															//"single_precision perand implicitly converted to double-precision"
															//ALTHOUGH IT IS A WARNING, THIS WILL MAKE THE PROGRAM not WORK!!!!!!
															//if declare as float, when setPoint+=0.5, need to cast : as setPioint+=(float)0.5, otherwise,
															//the whole program will not work! even this line has  not been used/excuted yet
															//BUT, if declare as double, there is no such problem.
															
	    														
	//			Project Options -> Target -> Floating Point Hardware -> Use FPU

	//You must then have code to enable the FPU hardware prior to using any FPU instructions. This is typically in the ResetHandler or SystemInit()

	//            system_stm32f4xx.c
  ////            void SystemInit(void)
	//							{
	//								/* FPU settings ------------------------------------------------------------*/
	//								#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	//									SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//								#endif
	//							...											
	//		-----MODIFY the system_stm32f4xx.c in the above way, will also fix the "float" type problem mentioned above. 												
	//   the is noted in 2016. by 2021, this problem may be fixed by STM.											

double measuredTemp;
ADC_HandleTypeDef adc_handle;

TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle, Tim5_Handle;
uint16_t Tim3_PrescalerValue, Tim4_PrescalerValue, Tim5_PrescalerValue;
TIM_OC_InitTypeDef Tim3_OCInitStructure;


void  LEDs_Config(void);
void  LEDs_On(void);
void  LEDs_Off(void);
void  LEDs_Toggle(void);

void  TIM4_Config(void);
void  TIM5_Config(void);
void  TIM3_PWM_Config(void);
void ExtBtn1_Config(void);
void ExtBtn2_Config(void);

int buttonHeld(GPIO_TypeDef *port, uint16_t pin);

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);

static void SystemClock_Config(void);
static void ADC_Config(void);
static void ADC_Read(void);
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
	
		//Configure LED3 and LED4 ======================================
		LEDs_Config();
	
		// Configure external buttons
		ExtBtn1_Config();
		ExtBtn2_Config();
		
		//configure the USER button as exti mode. 
		BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
																			
		
	
	
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
		
		//configure ADC
		ADC_Config();
		
		//configure timers
		TIM3_PWM_Config();
		TIM4_Config();
		TIM5_Config();
		
		
	
		LCD_DisplayString(3, 2, (uint8_t *) "Lab4 Starter ");
	
		LCD_DisplayString(9, 0, (uint8_t *) "Current ");
		LCD_DisplayString(10, 0, (uint8_t *)"setPoint");
		
		//get initial temperature
		ADC_Read();
		
		//start with set point above temperature reading
		setPoint = currentTemp + 0.5;
		LCD_DisplayFloat(9, 10, currentTemp, 2);
		LCD_DisplayFloat(10, 10, setPoint, 2);
		
		
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

// Configure pwm timer on PA7
void  TIM3_PWM_Config(void)
{

  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3;
   
  Tim3_Handle.Init.Period = 1000;
  Tim3_Handle.Init.Prescaler = 1800-1;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim3_Handle.Init.RepetitionCounter = 0; 
	

	Tim3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
 
 
	if(HAL_TIM_PWM_Init(&Tim3_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
	//configure the PWM channel
	Tim3_OCInitStructure.OCMode=  TIM_OCMODE_PWM1;
	Tim3_OCInitStructure.OCFastMode=TIM_OCFAST_DISABLE;
	Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
	
	Tim3_OCInitStructure.Pulse=CCR_Val;   // 0 to start
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
	
	
	
if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }   
	
}

// Timer 4 interrupts every 0.1s
void  TIM4_Config(void)
{
	
	
  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  Tim4_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  
  /* Set TIM4 instance */
  Tim4_Handle.Instance = TIM4;
   

  Tim4_Handle.Init.Period = 1000-1;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
	HAL_TIM_Base_MspInit(&Tim4_Handle);
	
  if(HAL_TIM_Base_Start_IT(&Tim4_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

// Timer 5 interrupts every 0.5s
void  TIM5_Config(void)
{
	
	__HAL_RCC_TIM5_CLK_ENABLE();
	
  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  Tim5_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  
  /* Set TIM4 instance */
  Tim5_Handle.Instance = TIM5;
   

  Tim5_Handle.Init.Period = 10000-1;
  Tim5_Handle.Init.Prescaler = Tim5_PrescalerValue;
  Tim5_Handle.Init.ClockDivision = 0;
  Tim5_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim5_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
	HAL_TIM_Base_MspInit(&Tim5_Handle);
	
  if(HAL_TIM_Base_Start_IT(&Tim5_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

// Configure PA5 for ADC input
static void ADC_Config(void)
{
	/*
		12b resolution
		5V Vref
		LSB = 1.22e-3
		time to reach LSB/2 = 4.05e-5s
	
		Sys clock = 16Mhz
		Prescaler /2
	  Cycles = 480
		ADC period = (12 + 480 cycles)/(16MHz/2) = 6.15e-5s
		
		ADC Period > time to reach LSB/2
		
	*/
	ADC_InitTypeDef adc_init;
	ADC_ChannelConfTypeDef adc_config;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__ADC1_CLK_ENABLE();
	
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	GPIO_InitTypeDef   GPIO_InitStruct;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	adc_init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	adc_init.Resolution = ADC_RESOLUTION12b;
	adc_init.ScanConvMode = DISABLE;
	adc_init.ContinuousConvMode = DISABLE;
  adc_init.DiscontinuousConvMode = DISABLE;
  adc_init.NbrOfDiscConversion = 0;
  adc_init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc_init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc_init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc_init.NbrOfConversion = 1;
  adc_init.DMAContinuousRequests = DISABLE;
  adc_init.EOCSelection = DISABLE;
	
	adc_handle.Instance = ADC1;
	adc_handle.Init = adc_init;
	if(HAL_ADC_Init(&adc_handle) != HAL_OK)
	{
		Error_Handler();
	}
	adc_config.Channel = ADC_CHANNEL_5;
  adc_config.Rank = 1;
  adc_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  adc_config.Offset = 0;
	if(HAL_ADC_ConfigChannel(&adc_handle, &adc_config) != HAL_OK)
	{
		Error_Handler();
	}
	

}

// Configure PC1 for external button 1
void ExtBtn1_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);
	
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

// Configure PD2 for external button 2
void ExtBtn2_Config(void){
 
	GPIO_InitTypeDef   GPIO_InitStructure;
	
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);
	
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

// Read voltage on ADC
static void ADC_Read(void)
{
	if(HAL_ADC_Start(&adc_handle) != HAL_OK)
	{
		Error_Handler(); 
	}
	// Poll for ADC value
	if(HAL_ADC_PollForConversion(&adc_handle,10)!=HAL_OK){
			Error_Handler(); 
	}
	else{
			ADC3ConvertedValue = HAL_ADC_GetValue(&adc_handle);
	}
	HAL_ADC_Stop(&adc_handle);
	
	//Convert from voltage to temperature
	currentTemp = (double)ADC3ConvertedValue/4096*100;
}



void LEDs_Config(void)
{
 /* Initialize Leds mounted on STM32F429-Discovery board */
	BSP_LED_Init(LED3);   //BSP_LED_....() are in stm32f4291_discovery.c
  BSP_LED_Init(LED4);
}

void LEDs_On(void){
/* Turn on LED3, LED4 */
 BSP_LED_On(LED3);
  BSP_LED_On(LED4);
}

void LEDs_Off(void){
/* Turn on LED3, LED4 */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
}
void LEDs_Toggle(void){
/* Turn on LED3, LED4 */
  BSP_LED_Toggle(LED3);
  BSP_LED_Toggle(LED4);
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
	
  if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
  {
			
  }
	
	// Increase set point while holding button 1
	if(GPIO_Pin == GPIO_PIN_1)
  {
		readButton = 1;
		buttonHeld(GPIOC, GPIO_PIN_1);

	}  //end of PIN_1

	// Decrease set point while holding button 2
	if(GPIO_Pin == GPIO_PIN_2)
  {
		readButton = 2;
		buttonHeld(GPIOD, GPIO_PIN_2);
			
	} //end of if PIN_2	
	
	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc_handle)
{
  /* Turn LED3 on: Transfer process is correct */
  BSP_LED_On(LED3);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 																														//for timer 3 , Timer 3 use update event initerrupt
{
	
	// Run every 0.1s
	if ((*htim).Instance==TIM4){ // Handle button held operations
			if(readButton){
				holdCount += 1;
				
				// Increase set point by 1
				if(readButton == 1 && !(holdCount%5)){
					setPoint += 1;
					BSP_LCD_ClearStringLine(10);
					LCD_DisplayString(10, 0, (uint8_t *)"setPoint");
					LCD_DisplayFloat(10, 10, setPoint, 2);
				}
				
				// Decrease set point by 1
				if(readButton == 2 && !(holdCount%5)){
					setPoint -= 1;
					BSP_LCD_ClearStringLine(10);
					LCD_DisplayString(10, 0, (uint8_t *)"setPoint");
					LCD_DisplayFloat(10, 10, setPoint, 2);
				}
			}
			
			//Turn on fan when setPoint > currentTemp
			if(!fanStatus && setPoint < currentTemp){
				LCD_DisplayString(13, 0, (uint8_t *)"Fan On ");
				fanStatus = 1;
				CCR_Val = 500;
				__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_2, CCR_Val);
			}
			//Turn off fan when setPoint <= currentTemp
			else if(fanStatus && setPoint >= currentTemp){
				LCD_DisplayString(13, 0, (uint8_t *)"Fan Off");
				fanStatus = 0;
				CCR_Val = 0;
				__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_2, CCR_Val);
			}
	}
	
	//Run every 0.5s
	if ((*htim).Instance==TIM5){
		
		//Read from ADC
		ADC_Read();
		LCD_DisplayFloat(9, 10, currentTemp, 2);
		
		//Adjust PWM speed based on temperature deviation and run time
		if(fanStatus && CCR_Val<1000){
				mult = 1 + currentTemp - setPoint;
				CCR_Val += 5*mult;
				__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_2, CCR_Val);
			}
	}
	
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer3
		//if ((*htim).Instance==TIM3){
			 //BSP_LED_Toggle(LED4);
		//}
		//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
	
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM3_pwm
	
	//__HAL_TIM_SET_COUNTER(htim, 0x0000);   //not necessary  
}


// Handle button holding function
int buttonHeld(GPIO_TypeDef *port, uint16_t pin){
	uint32_t start_time = HAL_GetTick();
	holdCount = 0;
	uint32_t hold_time = 0;
	uint32_t poll_time = 0;
	uint8_t status;
	
	// Button holding works for limited time to avoid infinite loop
	while(hold_time < 2000){
			status = !HAL_GPIO_ReadPin(port, pin);
			hold_time += status*(HAL_GetTick() - start_time);
			poll_time += (1-status)*(HAL_GetTick() - start_time);
		
			// Exit function when button released
			if(poll_time > 100){
				readButton = 0;
				return holdCount/5;
			}
			start_time = HAL_GetTick();
	}
	readButton = 0;
	return 0;
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

